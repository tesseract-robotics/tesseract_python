# Copyright 2022 Wason Technology, LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: John Wason (wason@wasontech.com)
# Date: 3/25/2022

import json
import struct
import cv2
import numpy as np
import math
from tesseract_robotics import tesseract_geometry
from tesseract_robotics.tesseract_common import Quaterniond, AngleAxisd
import pkgutil
import re
import base64
import io

from .util import tesseract_trajectory_to_list

def tesseract_env_to_gltf(t_env, origin_offset=[0,0,0], name = None, trajectory = None):
    gltf_dict, gltf_buf_io = tesseract_env_to_gltf_dict_and_buf(t_env,origin_offset,name)
    if trajectory is not None:
        _append_trajectory_animation(gltf_dict, gltf_buf_io, trajectory)
    buf_bytes = gltf_buf_io.getvalue()
    buf_uri = "data:application/octet-stream;base64," + base64.b64encode(buf_bytes).decode("ascii")
    gltf_dict["buffers"] = [
        {
            "byteLength": len(buf_bytes),
            "uri": buf_uri
        }
    ]
    return json.dumps(gltf_dict, indent=4)

_GLB_CHUNK_TYPE_JSON = 0x4E4F534A
_GLB_CHUNK_TYPE_BIN = 0x004E4942
_GLB_MAGIC = 0x46546C67

def tesseract_env_to_glb(t_env, origin_offset=[0,0,0], name = None):
    gltf_dict, gltf_buf_io = tesseract_env_to_gltf_dict_and_buf(t_env,origin_offset,name)
    while gltf_buf_io.tell() % 4 != 0:
        gltf_buf_io.write(b'\0')
    buf_bytes = gltf_buf_io.getvalue()

    gltf_dict["buffers"] = [
        {
            "byteLength": len(buf_bytes),
        }
    ]

    gltf_str = json.dumps(gltf_dict)
    if len(gltf_str) % 4 != 0:
        gltf_str += " " * (4 - (len(gltf_str) % 4))
    gltf_bytes = gltf_str.encode("utf-8")
    
    full_length = 12 + 8 + len(gltf_bytes) + 8 + len(buf_bytes)
    header_bytes = struct.pack("<III",_GLB_MAGIC,2,full_length)
    chunk1_header_bytes = struct.pack("<II",len(gltf_bytes),_GLB_CHUNK_TYPE_JSON)
    chunk2_header_bytes = struct.pack("<II",len(buf_bytes),_GLB_CHUNK_TYPE_BIN)

    out_io = io.BytesIO()
    out_io.write(header_bytes)
    out_io.write(chunk1_header_bytes)
    out_io.write(gltf_bytes)
    out_io.write(chunk2_header_bytes)
    out_io.write(buf_bytes)
    
    return out_io.getvalue()

def tesseract_env_to_gltf_dict_and_buf(t_env, origin_offset=[0,0,0], name= None):
    
    assert len(list(origin_offset)) == 3

    link_names = t_env.getLinkNames()
    joint_names = t_env.getJointNames()

    link_map = dict()
    joint_map = dict()
    for l in link_names:
        link_map[l] = t_env.getLink(l)

    for j in joint_names:
        joint_map[j] = t_env.getJoint(j)

    root_link_name = t_env.getRootLinkName()

    gltf_scene = {
        "nodes": [0]
    }
    if name is not None:
        gltf_scene["name"] = name

    gltf_dict = {
        "asset": {
            "generator": "Tesseract Robotics Scene Viewer Python glTF",
            "version": "2.0"
        },
        "scene": 0,
        "scenes": [gltf_scene]
    }

    gltf_buf_io = io.BytesIO()
    
    _append_link_recursive(gltf_dict, gltf_buf_io, link_map, joint_map, root_link_name)

    return gltf_dict, gltf_buf_io

def _find_child_joints(joint_map, parent_link_name):
    return [j for j in joint_map.values() if j.parent_link_name == parent_link_name]

def _env_frame_to_node(eigen_tf, name=None):
    p = eigen_tf.translation().flatten().tolist()
    q0 = Quaterniond(eigen_tf.rotation())
    q = [q0.x(), q0.y(), q0.z(), q0.w()]
    ret = {
        "rotation": q,
        "translation": p        
    }
    if name is not None:
        ret["name"] = name
    return ret

def _append_dict_list(gltf_dict, key, val):
    if not key in gltf_dict:
        gltf_dict[key] = []
    _ind = len(gltf_dict[key])
    gltf_dict[key].append(val)
    return val, _ind

def _append_node(gltf_dict, eigen_tf = None, name = None, extras = None):
    assert not (eigen_tf is None and name is None)
    if eigen_tf is None:
        node = { "name": name }
    else:
        node = _env_frame_to_node(eigen_tf, name)
    if extras is not None:
        node["extras"] = extras
    return _append_dict_list(gltf_dict, "nodes", node)
    

def _append_link_recursive(gltf_dict, gltf_buf_io, link_map, joint_map, link_name, shapes_mesh_inds = None):

    if shapes_mesh_inds is None:
        shapes_mesh_inds = dict()
    
    link = link_map[link_name]

    link_extras = {"tesseract_link": {"name": link_name}}
    link_node, link_ind = _append_node(gltf_dict, name="link_" + link_name, extras=link_extras)

    child_inds = []
    visual_i = 0
    for visual in link.visual:
        visual_i += 1
        _, visual_ind = _append_link_visual(gltf_dict, gltf_buf_io, link_name, visual, visual_i, shapes_mesh_inds)
        child_inds.append(visual_ind)

    child_joints = _find_child_joints(joint_map, link_name)
    for j in child_joints:
        _tf_joint = j.parent_to_joint_origin_transform
        parentjoint_node, parentjoint_ind = _append_node(gltf_dict, _tf_joint, "jointparent_" + j.getName())
        joint_extras = {"tesseract_joint": {"axis": list(j.axis.flatten()), "type": int(j.type), "name": j.getName()}}
        joint_node, joint_id = _append_node(gltf_dict, name="joint_" + j.getName(),extras=joint_extras)
        parentjoint_node["children"] = [joint_id]
        _, j_link_ind = _append_link_recursive(gltf_dict, gltf_buf_io, link_map, joint_map, j.child_link_name, shapes_mesh_inds)
        joint_node["children"] = [j_link_ind]
        child_inds.append(parentjoint_ind)
    
    if len(child_inds) > 0:
        link_node["children"] = child_inds

    return link_node, link_ind

def _append_link_visual(gltf_dict, gltf_buf_io, link_name, visual, visual_i, shapes_mesh_inds):

    visual_u_name = visual.name + str(visual_i)
    visual_name = "link_" + link_name + "_visual_" + visual_u_name
    visual_node, visual_ind = _append_node(gltf_dict, visual.origin, visual_name)

    tf_material = None

    visual_geom = visual.geometry
    if (isinstance(visual_geom,tesseract_geometry.PolygonMesh)):
    
        mesh=visual_geom
        vertices = mesh.getVertices()
        positions = np.zeros((len(vertices),3),dtype=np.float32)
        for i in range(len(vertices)):
            positions[i,:] = vertices[i].flatten()
        
        indices = mesh.getFaces().flatten().astype(np.uint32).reshape((-1,4))[:,1:4].flatten()
        
        _, positions_ind = _append_accessor(gltf_dict, gltf_buf_io, positions)
        _, indices_ind = _append_accessor(gltf_dict, gltf_buf_io, indices)

        normals_ind = None
        normals = mesh.getNormals()
        if normals is not None:
            normals2 = np.zeros((len(normals),3),dtype=np.float32)
            for i in range(len(normals)):
                normals2[i,:] = normals[i].flatten()
            _, normals_ind = _append_accessor(gltf_dict, gltf_buf_io, normals2)
        
        mesh_dict, mesh_ind = _append_dict_list(gltf_dict, "meshes", {
            "primitives": [
                {
                    "attributes": {
                        "POSITION": positions_ind
                    },
                    "indices": indices_ind
                }
            ],
            "name": visual_name + "_mesh"
        })

        # if normals_ind is not None:
        #     mesh_dict["primitives"][0]["attributes"]["NORMAL"] = normals_ind

        visual_node["scale"] = list(mesh.getScale().flatten())

        if not mesh.getResource().getUrl().lower().endswith('.stl'):
            mesh_material = mesh.getMaterial()
            if mesh_material is not None:
            
                tf_material = {
                    "name": "material_" + visual_name,
                    "alphaMode": "MASK",
                    "alphaCutoff": 0.4
                }
            
                base_color_factor = mesh_material.getBaseColorFactor().flatten().tolist()
                
                tf_material["pbrMetallicRoughness"] = {
                    "baseColorFactor": base_color_factor,
                    "roughnessFactor": mesh_material.getRoughnessFactor(),
                    "metallicFactor": mesh_material.getMetallicFactor(),                    
                }

                mesh_textures = mesh.getTextures()
                if mesh_textures is not None and len(mesh_textures) > 0:
                    mesh_tex = mesh_textures[0]
                    mesh_tex_image = mesh_tex.getTextureImage()
                    tex_name = mesh_tex_image.getUrl()
                    tex_mimetype = "image/jpg"
                    if tex_name.endswith('png'):
                        tex_mimetype = "image/png"

                    mesh_tex_image_bytes = bytearray(mesh_tex_image.getResourceContents())
                    tex_img = cv2.imdecode(np.frombuffer(mesh_tex_image_bytes,dtype=np.uint8),flags=1)
                    img_h, img_w, _ = tex_img.shape
                    _, image_bufview_ind = _append_bufview(gltf_dict, gltf_buf_io, mesh_tex_image_bytes)
                    _, image_ind = _append_dict_list(gltf_dict, "images", {
                        "mimeType": tex_mimetype,
                        "bufferView": image_bufview_ind
                    })

                    _, tex_ind = _append_dict_list(gltf_dict, "textures", {
                        "source": image_ind
                    })

                    tf_material["pbrMetallicRoughness"]["baseColorTexture"] = {
                        "index": tex_ind,
                        "texCoord": 0
                    }

                    mesh_uvs = mesh_tex.getUVs()
                    tf_uvs = np.zeros((len(mesh_uvs),2),dtype=np.float32)
                    for i in range(len(mesh_uvs)):
                        tf_uvs1 = mesh_uvs[i].flatten()
                        tf_uvs[i,:] = [tf_uvs1[0], 1.0-tf_uvs1[1]]
                    _, tex_ind = _append_accessor(gltf_dict, gltf_buf_io, tf_uvs)

                    mesh_dict["primitives"][0]["attributes"]["TEXCOORD_0"] = tex_ind

    elif (isinstance(visual_geom,tesseract_geometry.Box)):
        box=visual_geom
        mesh_dict, mesh_ind = _append_shape_mesh(gltf_dict, gltf_buf_io, "cube_geometry", visual_name, shapes_mesh_inds)
        visual_node["scale"] = [0.5*box.getX(), 0.5*box.getY(), 0.5*box.getZ()]

    elif (isinstance(visual_geom,tesseract_geometry.Sphere)):
        sphere=visual_geom
        mesh_dict, mesh_ind = _append_shape_mesh(gltf_dict, gltf_buf_io, "sphere_geometry", visual_name, shapes_mesh_inds)
        visual_node["scale"] = [sphere.getRadius(), sphere.getRadius(), sphere.getRadius()]

    elif (isinstance(visual_geom,tesseract_geometry.Cylinder)):
        cylinder=visual_geom
        mesh_dict, mesh_ind = _append_shape_mesh(gltf_dict, gltf_buf_io, "cylinder_geometry", visual_name, shapes_mesh_inds)
        visual_node["scale"] = [cylinder.getRadius(), cylinder.getRadius(), 0.5*cylinder.getLength()]

    if tf_material is None:
        tf_material = {
            "name": "material_" + visual_name,
            "alphaMode": "MASK",
            "alphaCutoff": 0.4
        }

        material = visual.material

        if material is None:
            tf_color = [0.5,0.5,0.5,1]
        else:
            tf_color = material.color.flatten().tolist()
        
        tf_material["pbrMetallicRoughness"] = {
            "baseColorFactor": tf_color,
            "roughnessFactor": 0.3,
            "metallicFactor": 0.3
        }

    material_dict, material_ind = _append_dict_list(gltf_dict, "materials", tf_material)
        
    mesh_dict["primitives"][0]["material"] = material_ind

    visual_node["mesh"] = mesh_ind


    return visual_node, visual_ind

_COMPONENT_TYPE_INT8 = 5120
_COMPONENT_TYPE_UINT8 = 5121
_COMPONENT_TYPE_INT16 = 5122
_COMPONENT_TYPE_UINT16 = 5123
_COMPONENT_TYPE_UINT32 = 5125
_COMPONENT_TYPE_FLOAT32 = 5126

_TYPE_SCALAR = "SCALAR" # 1 component
_TYPE_VEC2 = "VEC2" # 2 components
_TYPE_VEC3 = "VEC3" # 3 components
_TYPE_VEC4 = "VEC4" # 4 components
_TYPE_MAT2 = "MAT2" # 4 components
_TYPE_MAT2 = "MAT3" # 3 components
_TYPE_MAT2 = "MAT4" # 16 components

def _append_accessor(gltf_dict, gltf_buf_io, dat):

    dat_min = None
    dat_max = None

    if isinstance(dat,bytearray) or isinstance(dat,bytes):
        dat_count = len(dat)
        componentType = _COMPONENT_TYPE_UINT8
        type_ = _TYPE_SCALAR
    elif isinstance(dat,np.ndarray):
                
        if dat.dtype == np.int8:
            componentType = _COMPONENT_TYPE_INT8
        elif dat.dtype == np.uint8:
            componentType = _COMPONENT_TYPE_UINT8
        elif dat.dtype == np.int16:
            componentType = _COMPONENT_TYPE_INT16
        elif dat.dtype == np.uint16:
            componentType = _COMPONENT_TYPE_UINT16
        elif dat.dtype == np.uint32:
            componentType = _COMPONENT_TYPE_UINT32
        elif dat.dtype == np.float32:
            componentType = _COMPONENT_TYPE_FLOAT32
        else:
            assert False, "Invalid accessor componentType for np array"

        dat_count = dat.shape[0]

        if dat.ndim == 1:
            dat_min = [dat.min(0).tolist()]
            dat_max = [dat.max(0).tolist()]
            type_ = _TYPE_SCALAR
        elif dat.ndim == 2:
            dat_min = dat.min(0).tolist()
            dat_max = dat.max(0).tolist()
            if dat.shape[1] == 2:
                type_ = _TYPE_VEC2
            elif dat.shape[1] == 3:
                type_ = _TYPE_VEC3
            elif dat.shape[1] == 4:
                type_ = _TYPE_VEC4
            else:
                assert False, "Invalid accessor np array shape"
        else:
            assert False, "Invalid accessor np array shape"

        dat = dat.flatten().tobytes()
    
    else:
        assert False, "Invalid accessor data type"
    
    assert isinstance(dat, bytes) or isinstance(dat,bytearray)

    byteOffset = gltf_buf_io.tell()
    byteLength = len(dat)

    _, bufview_ind = _append_dict_list(gltf_dict, "bufferViews", {
        "buffer": 0,
        "byteLength": byteLength,
        "byteOffset": byteOffset
    })

    gltf_buf_io.write(dat)

    accessor, accessor_ind = _append_dict_list(gltf_dict, "accessors", {
        "bufferView": bufview_ind,
        "componentType": componentType,
        "type": type_,
        "count": dat_count
    })

    if dat_min is not None and dat_max is not None:
        accessor["min"] = dat_min
        accessor["max"] = dat_max

    return accessor, accessor_ind

def _append_bufview(gltf_dict, gltf_buf_io, dat):
    byteOffset = gltf_buf_io.tell()
    byteLength = len(dat)

    bufview_dict, bufview_ind = _append_dict_list(gltf_dict, "bufferViews", {
        "buffer": 0,
        "byteLength": byteLength,
        "byteOffset": byteOffset
    })

    gltf_buf_io.write(dat)

    return bufview_dict, bufview_ind

def _append_trajectory_animation(gltf_dict, gltf_buf_io, tesseract_trajectory):

    joint_names, traj = tesseract_trajectory_to_list(tesseract_trajectory)

    traj_np = np.asarray(traj)

    trajectory_time2_np = traj_np[:,-1].flatten().astype(np.float32)
    trajectory2_np = traj_np[:,:-1].astype(np.float32)


    joint_inds = {}
    joint_axes = {}
    joint_types = {}

    for i in range(len(gltf_dict["nodes"])):
        n = gltf_dict["nodes"][i]
        if "extras" in n:
            if "tesseract_joint" in n["extras"]:
                e1 = n["extras"]["tesseract_joint"]
                joint_inds[e1["name"]] = i
                joint_axes[e1["name"]] = np.array(e1["axis"],dtype=np.float32)
                joint_types[e1["name"]] = e1["type"]

    _, t_ind = _append_accessor(gltf_dict, gltf_buf_io,trajectory_time2_np)

    animation = {
        "channels": [],
        "samplers":  []
    }

    for i in range(len(joint_names)):
        joint_name = joint_names[i]
        trajectory2_i = trajectory2_np[:,i]
        #_, o_ind[i] = _append_accessor(gltf_dict, gltf_buf_io, trajectory2_i)
        joint_type = joint_types[joint_name]
        joint_axis = joint_axes[joint_name]

        if joint_type in (1,2):
            sampler_np = np.zeros((len(trajectory2_i),4),dtype=np.float32)
            joint_axisd = np.array(joint_axis,dtype=np.float64)
            for j in range(len(trajectory2_i)):
                a_ax = AngleAxisd(float(trajectory2_i[j]),joint_axisd)
                qd = Quaterniond(a_ax)
                sampler_np[j,:] = [qd.x(), qd.y(), qd.z(), qd.w()]
            _, s_ind = _append_accessor(gltf_dict, gltf_buf_io, sampler_np)

            animation["samplers"].append({
                "input": t_ind,
                "output": s_ind,
                "interpolation": "LINEAR"
            })

            animation["channels"].append({
                "sampler": i,
                "target": {
                    "node": joint_inds[joint_name],
                    "path": "rotation"
                }
            })
        elif joint_type == 3:
            sampler_np = np.zeros((len(trajectory2_i),3),dtype=np.float32)
            joint_axisd = np.array(joint_axis,dtype=np.float64)
            for j in range(len(trajectory2_i)):
                vec = float(trajectory2_i[j]) * joint_axisd
                sampler_np[j,:] = vec.tolist()
            _, s_ind = _append_accessor(gltf_dict, gltf_buf_io, sampler_np)

            animation["samplers"].append({
                "input": t_ind,
                "output": s_ind,
                "interpolation": "LINEAR"
            })

            animation["channels"].append({
                "sampler": i,
                "target": {
                    "node": joint_inds[joint_name],
                    "path": "translation"
                }
            })

        else:
            assert False, "Unknown joint type " + str(joint_type)

    _append_dict_list(gltf_dict, "animations", animation)

        
    print(animation)

def _append_shapes_mesh_accessors(gltf_dict, gltf_buf_io, name):
    o = _geometry_shapes[name]

    _, positions_ind = _append_accessor(gltf_dict, gltf_buf_io, o["positions"])
    _, normals_ind = _append_accessor(gltf_dict, gltf_buf_io, o["normals"])
    _, indices_ind = _append_accessor(gltf_dict, gltf_buf_io, o["indices"])

    return {
        "positions_ind": positions_ind,
        "normals_ind": normals_ind,
        "indices_ind": indices_ind
    }

def _append_shape_mesh(gltf_dict, gltf_buf_io, shape_name, visual_name, shapes_mesh_inds):
    accessors_inds = shapes_mesh_inds.get(shape_name, None)
    if accessors_inds is None:
        accessors_inds = _append_shapes_mesh_accessors(gltf_dict, gltf_buf_io, shape_name)
        shapes_mesh_inds[shape_name] = accessors_inds
    
    mesh_dict, mesh_ind = _append_dict_list(gltf_dict, "meshes", {
            "primitives": [
                {
                    "attributes": {
                        "POSITION": accessors_inds["positions_ind"],
                        "NORMALS": accessors_inds["normals_ind"]
                    },
                    "indices": accessors_inds["indices_ind"]
                }
            ],
            "name": visual_name + "_mesh"
        })
    return mesh_dict, mesh_ind

def _load_shapes():
    geometries = json.loads(pkgutil.get_data("tesseract_robotics_viewer.resources","geometries.json"))["geometries"]

    geometry_names = ["cube_geometry", "sphere_geometry", "cylinder_geometry"]

    for name in geometry_names:
        g = next(filter(lambda x: x["id"] == name, geometries["vertexData"]))
        o = dict()
        o["name"] = name
        o["positions"] = np.array(g["positions"],dtype=np.float32).reshape((-1,3))
        o["normals"] = np.array(g["normals"],dtype=np.float32).reshape((-1,3))
        #o["uvs"] = np.array(g["uvs"],dtype=np.float32).reshape((-1,2))
        o["indices"] = np.array(g["indices"],dtype=np.uint32)

        _geometry_shapes[name] = o

_geometry_shapes = dict()

_load_shapes()

    

