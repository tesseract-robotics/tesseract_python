/**
 * @file tesseract_common_python.i
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

%module(directors="1", package="tesseract_robotics.tesseract_common") tesseract_common_python

#pragma SWIG nowarn=473

%{
//#define SWIG_PYTHON_EXTRA_NATIVE_CONTAINERS
%}

%include "tesseract_swig_include.i"

%{

// tesseract_common
#include <tesseract_common/types.h>
#include <tesseract_common/status_code.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_common/manipulator_info.h>
#include <tesseract_common/joint_state.h>
#include <tesseract_common/collision_margin_data.h>
#include <tesseract_common/allowed_collision_matrix.h>
#include <tesseract_common/kinematic_limits.h>
#include <tesseract_common/timer.h>
#include <tesseract_common/type_erasure.h>
#include <console_bridge/console.h>

#include "tesseract_common_python_std_functions.h"

%}

%include "tinyxml2.i"
%include "boost_filesystem_path.i"
%include "eigen_geometry.i"
%include "console_bridge.i"

%pythondynamic sco::ModelType;

%template(vector_string) std::vector<std::string>;
%template(set_string) std::set<std::string>;
%template(pair_string) std::pair<std::string, std::string>;
%template(vector_pair_string) std::vector<std::pair<std::string, std::string> >;
%template(map_string_vector_pair_string) std::unordered_map<std::string, std::vector<std::pair<std::string, std::string>>>;
%template(pair_vector_string) std::pair<std::vector<std::string>, std::vector<std::string>>;

%template(vector_double) std::vector<double>;
%template(map_string_vector_double) std::unordered_map<std::string, std::vector<double> >;
%template(map_string_double) std::unordered_map<std::string, double>;
%template(map_string_map_string_double) std::unordered_map<std::string, std::unordered_map<std::string, double> >;
%template(map_string_map_string_string) std::unordered_map<std::string, std::unordered_map<std::string, std::string> >;
%template(map_string_map_string_map_string_double) std::unordered_map<std::string, std::unordered_map<std::string, std::unordered_map<std::string, double> > >;

%template(vector_size_t) std::vector<std::size_t>;

%template(array2_int) std::array<int,2>;
%template(array2_string) std::array<std::string,2>;
%template(array2_Vector3d) std::array<Eigen::Vector3d,2>;
%template(array2_Isometry3d) std::array<Eigen::Isometry3d,2>;
%template(array2_double) std::array<double,2>;

%define %tesseract_aligned_vector(name,T)
%template(name) std::vector<T , Eigen::aligned_allocator<T >>;
%enddef

%define %tesseract_aligned_map(name,Key,Value)
%template(name) std::map<Key, Value, std::less<Key>, Eigen::aligned_allocator<std::pair<const Key, Value>>>;
%enddef

%define %tesseract_aligned_map_of_aligned_vector(name,Key,Value)
%tesseract_aligned_map(name, %arg(Key), %arg(std::vector<Value , Eigen::aligned_allocator<Value >>));
%enddef

%define %tesseract_aligned_unordered_map(name,Key,Value)
%template(name) std::unordered_map<Key,Value,std::hash<Key>,std::equal_to<Key>,Eigen::aligned_allocator<std::pair<const Key, Value>>>;
%enddef

%tesseract_aligned_vector(VectorIsometry3d, Eigen::Isometry3d);
%template(VectorVector3d) std::vector<Eigen::Vector3d>;
%template(VectorVectorXd) std::vector<Eigen::VectorXd>;
%tesseract_aligned_vector(VectorVector2d, Eigen::Vector2d);
%tesseract_aligned_vector(VectorVector4d, Eigen::Vector4d);
%tesseract_aligned_map(TransformMap, std::string, Eigen::Isometry3d);

// SWIG is not smart enough to expand templated using, override the behavior
%define %tesseract_aligned_vector_using(name,T)
using name = std::vector<T , Eigen::aligned_allocator<T >>;
%enddef

%define %tesseract_aligned_map_using(name,Key,Value)
using name = std::map<Key, Value, std::less<Key>, Eigen::aligned_allocator<std::pair<const Key, Value>>>;
%enddef

%define %tesseract_aligned_map_of_aligned_vector_using(name,Key,Value)
%tesseract_aligned_map_using(name, %arg(Key), %arg(std::vector<Value , Eigen::aligned_allocator<Value >>));
%enddef

namespace tesseract_common
{
%tesseract_aligned_vector_using(VectorIsometry3d, Eigen::Isometry3d);
%tesseract_aligned_vector_using(VectorVector2d, Eigen::Vector2d);
%tesseract_aligned_vector_using(VectorVector4d, Eigen::Vector4d);
%tesseract_aligned_map_using(TransformMap, std::string, Eigen::Isometry3d);
}

%ignore toXML(tinyxml2::XMLDocument& doc) const;
%ignore CONFIG_KEY;

%typemap(out, fragment="SWIG_From_std_string") std::string& {
  $result = SWIG_From_std_string(*$1);
}

%typemap(out, fragment="SWIG_From_std_string") const std::string& {
  $result = SWIG_From_std_string(*$1);
}

%feature("valuewrapper") std::type_index;
%nodefaultctor std::type_index;
namespace std
{
  class type_index
  {
  public:
    size_t hash_code();
    const char* name();
  };
}

#define BOOST_CLASS_EXPORT_KEY(a)
#define BOOST_CLASS_EXPORT_KEY2(a,b)
#define BOOST_CLASS_VERSION(a,b)
#define BOOST_CLASS_TRACKING(a,b)
#define BOOST_CLASS_EXPORT_IMPLEMENT(a)
#define BOOST_SERIALIZATION_ASSUME_ABSTRACT(a)

#define EIGEN_MAKE_ALIGNED_OPERATOR_NEW
#define TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#define TESSERACT_COMMON_IGNORE_WARNINGS_POP
#define DEPRECATED(msg)

%pythondynamic tesseract_common::ResourceLocator;

%include "tesseract_std_function.i"

%tesseract_std_function(SimpleResourceLocatorFn,tesseract_common,std::string,const std::string&,a);

%shared_ptr(tesseract_common::AllowedCollisionMatrix)

%shared_ptr(tesseract_common::Resource)
%template(vector_uint8) std::vector<uint8_t>;
%pybuffer_binary(const uint8_t* bytes, size_t bytes_len);
%shared_ptr(tesseract_common::BytesResource)
%feature("director") tesseract_common::ResourceLocator;
%shared_ptr(tesseract_common::ResourceLocator)
%shared_ptr(tesseract_common::SimpleResourceLocator)
%shared_ptr(tesseract_common::SimpleLocatedResource)
%shared_ptr(tesseract_common::StatusCategory)
%shared_ptr(tesseract_common::GeneralStatusCategory)
%shared_ptr(tesseract_common::StatusCode)


// tesseract_common
#define TESSERACT_COMMON_PUBLIC
%include "tesseract_common/types.h"
%template(AllowedCollisionEntries) std::unordered_map<std::pair<std::string,std::string>, std::string, tesseract_common::PairHash>;
%template(PluginInfoMap) std::map<std::string, tesseract_common::PluginInfo>;
%include "tesseract_common/status_code.h"
%include "tesseract_common/resource_locator.h"
%ignore tcp_offset;
%include "tesseract_common/manipulator_info.h"
%rename("%s") tcp_offset;
%include "tesseract_common/joint_state.h"
%include "tesseract_common/collision_margin_data.h"
%include "tesseract_common/allowed_collision_matrix.h"
%include "tesseract_common/kinematic_limits.h"
%include "tesseract_common/timer.h"


// TODO: ?
//%template(Instructions) std::vector<tesseract_common::Any>;


%extend tesseract_common::ManipulatorInfo {
  void _setTcpOffset(const Eigen::Isometry3d& offset)
  {
    self->tcp_offset = offset;
  }

  void _setTcpOffset(const std::string& offset)
  {
    self->tcp_offset = offset;
  }

  std::size_t _getTcpOffsetIndex()
  {
    return self->tcp_offset.index();
  }

  Eigen::Isometry3d _getTcpOffsetIsometry3d()
  {
    if (self->tcp_offset.index() != 0)
      return std::get<1>(self->tcp_offset);
    throw std::runtime_error("tcp_offset is not an Isometry3d");
  }

  std::string _getTcpOffsetString()
  {
    if (self->tcp_offset.index() == 0)
      return std::get<0>(self->tcp_offset);
    throw std::runtime_error("tcp_offset is not a string");
  }

  %pythoncode %{
      def _getTcpOffset(self):
        if self._getTcpOffsetIndex() == 0:
          return self._getTcpOffsetString()
        else:
          return self._getTcpOffsetIsometry3d()
      tcp_offset = property(_getTcpOffset, _setTcpOffset)
  %}
}


