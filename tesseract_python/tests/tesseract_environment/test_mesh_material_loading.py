from tesseract_robotics import tesseract_scene_graph
from tesseract_robotics import tesseract_collision
from tesseract_robotics import tesseract_environment
from tesseract_robotics.tesseract_common import Isometry3d, Translation3d, AngleAxisd
from tesseract_robotics import tesseract_common
from tesseract_robotics import tesseract_collision
from tesseract_robotics import tesseract_urdf
from ..tesseract_support_resource_locator import TesseractSupportResourceLocator
import os
import re
import traceback
import numpy.testing as nptest

mesh_urdf="""
<robot name="mesh_viewer">
  
  <link name="world"/>
  <link name="mesh_dae_link">
    <visual>
      <geometry>
        <mesh filename="package://tesseract_support/meshes/tesseract_material_mesh.dae"/>
      </geometry>
    </visual>
  </link>

  <joint name="mesh_dae_joint" type="revolute">
    <parent link="world"/>
    <child link="mesh_dae_link"/>
    <axis xyz="0 1 0"/>
    <origin xyz="0 1 0.25"/>
    <limit effort="0" lower="-2.0944" upper="2.0944" velocity="6.2832"/>
  </joint>

</robot>
"""

def get_scene_graph():
    locator = TesseractSupportResourceLocator()
    return tesseract_urdf.parseURDFString(mesh_urdf, locator).release()
    
def test_mesh_material_loading():
    scene = get_scene_graph()
    visual = scene.getLink("mesh_dae_link").visual
    assert len(visual) == 1

    print("Geometry type")
    print(visual[0].geometry.getType())

    meshes = visual[0].geometry.getMeshes()

    mesh0 = meshes[0]
    mesh1 = meshes[1]
    mesh2 = meshes[2]
    mesh3 = meshes[3]

    assert mesh0.getFaceCount() == 2
    assert mesh0.getVertexCount() == 4
    assert mesh1.getFaceCount() == 34
    assert mesh1.getVertexCount() == 68
    assert mesh2.getFaceCount() == 15
    assert mesh2.getVertexCount() == 17
    assert mesh3.getFaceCount() == 15
    assert mesh3.getVertexCount() == 17

    mesh0_normals = mesh0.getNormals()
    assert mesh0_normals is not None
    assert len(mesh0_normals) == 4
    mesh1_normals = mesh1.getNormals()
    assert mesh1_normals is not None
    assert len(mesh1_normals) ==  68
    mesh2_normals = mesh2.getNormals()
    assert mesh2_normals is not None
    assert len(mesh2_normals) == 17
    mesh3_normals = mesh3.getNormals()
    assert mesh3_normals is not None
    assert len(mesh3_normals) == 17

    mesh0_material = mesh0.getMaterial()
    nptest.assert_allclose(mesh0_material.getBaseColorFactor().flatten(),[1,1,1,1], atol=0.01)
    nptest.assert_almost_equal(mesh0_material.getMetallicFactor(), 0.0)
    nptest.assert_almost_equal(mesh0_material.getRoughnessFactor(), 0.5)
    nptest.assert_allclose(mesh0_material.getEmissiveFactor().flatten(), [0,0,0,1], atol = 0.01)

    mesh1_material = mesh1.getMaterial()
    nptest.assert_allclose(mesh1_material.getBaseColorFactor().flatten(),[0.7,0.7,0.7,1], atol=0.01)
    nptest.assert_almost_equal(mesh1_material.getMetallicFactor(), 0.0)
    nptest.assert_almost_equal(mesh1_material.getRoughnessFactor(), 0.5)
    nptest.assert_allclose(mesh1_material.getEmissiveFactor().flatten(), [0,0,0,1], atol = 0.01)

    mesh2_material = mesh2.getMaterial()
    nptest.assert_allclose(mesh2_material.getBaseColorFactor().flatten(),[0.8,0 ,0, 1], atol=0.01)
    nptest.assert_almost_equal(mesh2_material.getMetallicFactor(), 0.0)
    nptest.assert_almost_equal(mesh2_material.getRoughnessFactor(), 0.5)
    nptest.assert_allclose(mesh2_material.getEmissiveFactor().flatten(), [0,0,0,1], atol = 0.01)

    mesh3_material = mesh3.getMaterial()
    nptest.assert_allclose(mesh3_material.getBaseColorFactor().flatten(),[0.05,0.8,0.05,1], atol=0.01)
    nptest.assert_almost_equal(mesh3_material.getMetallicFactor(), 0.0)
    nptest.assert_almost_equal(mesh3_material.getRoughnessFactor(), 0.5)
    nptest.assert_allclose(mesh3_material.getEmissiveFactor().flatten(), [0.1,0.1,0.5,1], atol = 0.01)

    assert mesh1.getTextures() is None
    assert mesh2.getTextures() is None
    assert  mesh3.getTextures() is None

    assert mesh0.getTextures() is not None
    assert len(mesh0.getTextures()) == 1

    texture = mesh0.getTextures()[0]
    assert len(texture.getTextureImage().getResourceContents()) == 38212
    assert len(texture.getUVs()) == 4
