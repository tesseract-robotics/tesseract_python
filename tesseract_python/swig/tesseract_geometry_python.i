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
#include <tesseract/geometry/geometry.h>
#include "tesseract/geometry/impl/mesh_material.h"
#include <tesseract/geometry/geometries.h>
#include <tesseract/geometry/utils.h>
#include <tesseract/geometry/mesh_parser.h>
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
%tesseract_vector_eigen_shared_ptr_adaptor(std::vector< std::shared_ptr<tesseract::geometry::MeshTexture> >)

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

%shared_ptr(tesseract::geometry::MeshMaterial)
%shared_ptr(tesseract::geometry::MeshTexture)
%template(VectorMeshTexture) std::vector<std::shared_ptr<tesseract::geometry::MeshTexture>>;
%include "tesseract/geometry/impl/mesh_material.h"


%shared_ptr(tesseract::geometry::Geometry)
%nodefaultctor tesseract::geometry::Geometry;

%include "tesseract/geometry/geometry.h"

%template(Geometries) std::vector<std::shared_ptr<tesseract::geometry::Geometry> >;
%template(GeometriesConst) std::vector<std::shared_ptr<const tesseract::geometry::Geometry> >;

%shared_factory(
    tesseract::geometry::Geometry,
    tesseract::geometry::Box,
    tesseract::geometry::Capsule,
    tesseract::geometry::Cone,
    tesseract::geometry::ConvexMesh,
    tesseract::geometry::Cylinder,
    tesseract::geometry::Octree,
    tesseract::geometry::Plane,
    tesseract::geometry::PolygonMesh,
    tesseract::geometry::Mesh,
    tesseract::geometry::SDFMesh,
    tesseract::geometry::Sphere,
    tesseract::geometry::CompoundMesh
)


%shared_ptr(tesseract::geometry::Box)
%include <tesseract/geometry/impl/box.h>

%shared_ptr(tesseract::geometry::Capsule)
%include <tesseract/geometry/impl/capsule.h>

%shared_ptr(tesseract::geometry::PolygonMesh)
%template(PolygonMeshVector) std::vector<std::shared_ptr<tesseract::geometry::PolygonMesh> >;
%include <tesseract/geometry/impl/polygon_mesh.h>

%shared_ptr(tesseract::geometry::Cone)
%include <tesseract/geometry/impl/cone.h>

%shared_ptr(tesseract::geometry::ConvexMesh)
%template(ConvexMeshVector) std::vector<std::shared_ptr<tesseract::geometry::ConvexMesh> >;
%include <tesseract/geometry/impl/convex_mesh.h>

%shared_ptr(tesseract::geometry::Cylinder)
%include <tesseract/geometry/impl/cylinder.h>

%shared_ptr(tesseract::geometry::Mesh)
%template(MeshVector) std::vector<std::shared_ptr<tesseract::geometry::Mesh> >;
%include <tesseract/geometry/impl/mesh.h>
%include <tesseract/geometry/impl/mesh_material.h>

%shared_ptr(tesseract::geometry::Octree)
%nodefaultctor tesseract::geometry::Octree;
namespace octomap 
{ 
%nodefaultctor OcTree;
class OcTree {}; 
}
%shared_ptr(octomap::OcTree);
%include <tesseract/geometry/impl/octree.h>

%shared_ptr(tesseract::geometry::Plane)
%include <tesseract/geometry/impl/plane.h>

%shared_ptr(tesseract::geometry::SDFMesh)
%template(SDFMeshVector) std::vector<std::shared_ptr<tesseract::geometry::SDFMesh> >;
%include <tesseract/geometry/impl/sdf_mesh.h>

%shared_ptr(tesseract::geometry::Sphere)
%include <tesseract/geometry/impl/sphere.h>

%shared_ptr(tesseract::geometry::CompoundMesh)
%include <tesseract/geometry/impl/compound_mesh.h>

%include "tesseract/geometry/geometries.h"
%include "tesseract/geometry/utils.h"

%include "tesseract/geometry/mesh_parser.h"
%pybuffer_binary(const uint8_t* bytes, size_t bytes_len);
// %pybuffer_binary does not automatically create a typecheck
%typemap(typecheck) (const uint8_t* bytes, size_t bytes_len) {
    // Check if the input is a Python bytes or bytearray object
    $1 = PyBytes_Check($input) || PyByteArray_Check($input);
}
%template(createMeshFromResource) tesseract::geometry::createMeshFromResource<tesseract::geometry::Mesh>;
%template(createSDFMeshFromResource) tesseract::geometry::createMeshFromResource<tesseract::geometry::SDFMesh>;
%template(createConvexMeshFromResource) tesseract::geometry::createMeshFromResource<tesseract::geometry::ConvexMesh>;
%template(createMeshFromPath) tesseract::geometry::createMeshFromPath<tesseract::geometry::Mesh>;
%template(createSDFMeshFromPath) tesseract::geometry::createMeshFromPath<tesseract::geometry::SDFMesh>;
%template(createConvexMeshFromPath) tesseract::geometry::createMeshFromPath<tesseract::geometry::ConvexMesh>;
%template(createMeshFromBytes) tesseract::geometry::createMeshFromBytes<tesseract::geometry::Mesh>;
%template(createSDFMeshFromBytes) tesseract::geometry::createMeshFromBytes<tesseract::geometry::SDFMesh>;
%template(createConvexMeshFromBytes) tesseract::geometry::createMeshFromBytes<tesseract::geometry::ConvexMesh>;
