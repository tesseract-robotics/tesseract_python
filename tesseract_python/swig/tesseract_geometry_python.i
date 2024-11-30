/**
 * @file tesseract_geometry_python.i
 * @brief The tesseract_common_python SWIG master file.
 *
 * @author John Wason
 * @date December 8, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Wason Technology, LLC
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

%module(directors="1", package="tesseract_robotics.tesseract_geometry") tesseract_geometry_python

#pragma SWIG nowarn=473

%include "tesseract_swig_include.i"

%import "tesseract_common_python.i"

%{


// tesseract_geometry
#include <tesseract_geometry/geometry.h>
#include "tesseract_geometry/impl/mesh_material.h"
#include <tesseract_geometry/geometries.h>
#include <tesseract_geometry/utils.h>
#include <tesseract_geometry/mesh_parser.h>
%}

// Define typemaps for types used by Mesh classes

%define %tesseract_vector_eigen_shared_ptr_adaptor(TYPE)
%typemap(in, noblock=0) std::shared_ptr<const TYPE > (void  *argp = 0, int res = 0, TYPE* temp1) {

  // Override for "in" std::shared_ptr<const TYPE> by value instead of shared_ptr
  res = SWIG_ConvertPtr($input, &argp,$descriptor(TYPE*), $disown | %convertptr_flags);
  if (!SWIG_IsOK(res)) { 
    %argument_fail(res, "$type", $symname, $argnum); 
  }
   temp1 = %reinterpret_cast(argp, TYPE*);
   
   $1 = std::make_shared< TYPE >(*temp1);
}

%typemap(typecheck,precedence=SWIG_TYPECHECK_POINTER,noblock=1) std::shared_ptr<const TYPE > {
  void *vptr = 0;
  int res = SWIG_ConvertPtr($input, &vptr, $descriptor(TYPE&), SWIG_POINTER_NO_NULL);
  $1 = SWIG_CheckState(res);
}

%typemap(out) std::shared_ptr<const TYPE> const & {
    // Override to return std::shared_ptr<const TYPE> by value instead of shared_ptr
    if (!*$1)
    {
        %set_output(SWIG_Py_Void());  
    }
    else
    {
        const TYPE& temp_out = **$1;
        %set_output(SWIG_NewPointerObj(%new_copy(temp_out,TYPE), $descriptor(TYPE&), SWIG_POINTER_OWN | %newpointer_flags));
    }
}
%enddef

%tesseract_vector_eigen_shared_ptr_adaptor(%arg(std::vector< Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> >))
%tesseract_vector_eigen_shared_ptr_adaptor(std::vector< Eigen::Vector3d >)
%tesseract_vector_eigen_shared_ptr_adaptor(%arg(std::vector< Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> >))
%tesseract_vector_eigen_shared_ptr_adaptor(std::vector< std::shared_ptr<tesseract_geometry::MeshTexture> >)

%define %tesseract_eigen_shared_ptr_adaptor(TYPE)

%typemap(in, fragment="Eigen_Fragments") std::shared_ptr<const TYPE > (TYPE temp)
{
  // Override for "in" TYPE by value instead of shared_ptr
  if (!ConvertFromNumpyToEigenMatrix< TYPE >(&temp, $input))
    SWIG_fail;
  $1 = std::make_shared< TYPE >(temp);
}

%typemap(typecheck,precedence=SWIG_TYPECHECK_POINTER,noblock=1) std::shared_ptr<const TYPE >
{
$1 = is_array($input);
}

%typemap(out, fragment="Eigen_Fragments") std::shared_ptr<const TYPE > const & {

    // Override to return TYPE by value instead of shared_ptr
    if (!*$1)
    {
        %set_output(SWIG_Py_Void());
    }
    else
    {
        TYPE temp_out = **$1;
        if (!ConvertFromEigenToNumPyMatrix< TYPE >(&$result, &temp_out))
            SWIG_fail;
    }
}
%enddef

%tesseract_eigen_shared_ptr_adaptor( Eigen::VectorXi )

// tesseract_geometry
#define TESSERACT_GEOMETRY_PUBLIC

%shared_ptr(tesseract_geometry::MeshMaterial)
%shared_ptr(tesseract_geometry::MeshTexture)
%template(VectorMeshTexture) std::vector<std::shared_ptr<tesseract_geometry::MeshTexture>>;
%include "tesseract_geometry/impl/mesh_material.h"


%shared_ptr(tesseract_geometry::Geometry)
%nodefaultctor tesseract_geometry::Geometry;

%include "tesseract_geometry/geometry.h"

%template(Geometries) std::vector<std::shared_ptr<tesseract_geometry::Geometry> >;
%template(GeometriesConst) std::vector<std::shared_ptr<const tesseract_geometry::Geometry> >;

%shared_factory(
    tesseract_geometry::Geometry,
    tesseract_geometry::Box,
    tesseract_geometry::Capsule,
    tesseract_geometry::Cone,
    tesseract_geometry::ConvexMesh,
    tesseract_geometry::Cylinder,
    tesseract_geometry::Octree,
    tesseract_geometry::Plane,
    tesseract_geometry::PolygonMesh,
    tesseract_geometry::Mesh,
    tesseract_geometry::SDFMesh,
    tesseract_geometry::Sphere,
    tesseract_geometry::CompoundMesh
)


%shared_ptr(tesseract_geometry::Box)
%include <tesseract_geometry/impl/box.h>

%shared_ptr(tesseract_geometry::Capsule)
%include <tesseract_geometry/impl/capsule.h>

%shared_ptr(tesseract_geometry::PolygonMesh)
%template(PolygonMeshVector) std::vector<std::shared_ptr<tesseract_geometry::PolygonMesh> >;
%include <tesseract_geometry/impl/polygon_mesh.h>

%shared_ptr(tesseract_geometry::Cone)
%include <tesseract_geometry/impl/cone.h>

%shared_ptr(tesseract_geometry::ConvexMesh)
%template(ConvexMeshVector) std::vector<std::shared_ptr<tesseract_geometry::ConvexMesh> >;
%include <tesseract_geometry/impl/convex_mesh.h>

%shared_ptr(tesseract_geometry::Cylinder)
%include <tesseract_geometry/impl/cylinder.h>

%shared_ptr(tesseract_geometry::Mesh)
%template(MeshVector) std::vector<std::shared_ptr<tesseract_geometry::Mesh> >;
%include <tesseract_geometry/impl/mesh.h>
%include <tesseract_geometry/impl/mesh_material.h>

%shared_ptr(tesseract_geometry::Octree)
%nodefaultctor tesseract_geometry::Octree;
namespace octomap 
{ 
%nodefaultctor OcTree;
class OcTree {}; 
}
%shared_ptr(octomap::OcTree);
%include <tesseract_geometry/impl/octree.h>

%shared_ptr(tesseract_geometry::Plane)
%include <tesseract_geometry/impl/plane.h>

%shared_ptr(tesseract_geometry::SDFMesh)
%template(SDFMeshVector) std::vector<std::shared_ptr<tesseract_geometry::SDFMesh> >;
%include <tesseract_geometry/impl/sdf_mesh.h>

%shared_ptr(tesseract_geometry::Sphere)
%include <tesseract_geometry/impl/sphere.h>

%shared_ptr(tesseract_geometry::CompoundMesh)
%include <tesseract_geometry/impl/compound_mesh.h>

%include "tesseract_geometry/geometries.h"
%include "tesseract_geometry/utils.h"

%include "tesseract_geometry/mesh_parser.h"
%pybuffer_binary(const uint8_t* bytes, size_t bytes_len);
%template(createMeshFromResource) tesseract_geometry::createMeshFromResource<tesseract_geometry::Mesh>;
%template(createSDFMeshFromResource) tesseract_geometry::createMeshFromResource<tesseract_geometry::SDFMesh>;
%template(createConvexMeshFromResource) tesseract_geometry::createMeshFromResource<tesseract_geometry::ConvexMesh>;
%template(createMeshFromPath) tesseract_geometry::createMeshFromPath<tesseract_geometry::Mesh>;
%template(createSDFMeshFromPath) tesseract_geometry::createMeshFromPath<tesseract_geometry::SDFMesh>;
%template(createConvexMeshFromPath) tesseract_geometry::createMeshFromPath<tesseract_geometry::ConvexMesh>;
%template(createMeshFromBytes) tesseract_geometry::createMeshFromBytes<tesseract_geometry::Mesh>;
%template(createSDFMeshFromBytes) tesseract_geometry::createMeshFromBytes<tesseract_geometry::SDFMesh>;
%template(createConvexMeshFromBytes) tesseract_geometry::createMeshFromBytes<tesseract_geometry::ConvexMesh>;
