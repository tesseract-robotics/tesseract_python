/**
 * @file eigen_types.i
 * @brief Eigen typemaps used by Tesseract
 *
 * @author John Wason
 * @date December 10, 2019
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2019, Wason Technology, LLC
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

// workaround Eigen::Ref argout

namespace Eigen
{
  template <typename T> class Ref;
  class MatrixXd;
}

%feature("valuewrapper") Eigen::Ref<Eigen::MatrixXd>;
%template() Eigen::Ref<Eigen::MatrixXd>;


%eigen_typemaps(Eigen::Vector2d);
%eigen_typemaps(Eigen::Vector3d);
%eigen_typemaps(Eigen::Vector4d);
%eigen_typemaps(Eigen::VectorXd);
%eigen_typemaps(%arg(Eigen::VectorX<double>));
%eigen_typemaps(Eigen::MatrixXd);
%eigen_typemaps(%arg(Eigen::Matrix3Xd));
%eigen_typemaps(%arg(Eigen::MatrixX2d));
%eigen_typemaps(%arg(Eigen::Matrix<uint32_t,3,Eigen::Dynamic>));
%eigen_typemaps(%arg(Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>));
%eigen_typemaps(Eigen::VectorXi);
%eigen_typemaps(Eigen::Matrix3d);
%eigen_typemaps(Eigen::Matrix4d);

// Assume that Eigen::Index can be treated as int64_t
namespace Eigen {
typedef ptrdiff_t Index;
}

/*
//Workaround typemaps for Isometry3d

%typemap(in, fragment="Eigen_Fragments") Eigen::Isometry3d const& (Eigen::Isometry3d temp)
{
  // In: plain, const&
  Eigen::Matrix4d temp_matrix;  
  if (!ConvertFromNumpyToEigenMatrix<Eigen::Matrix4d>(&temp_matrix, $input))
    SWIG_fail;
  temp = temp_matrix;
  $1 = &temp;
}
%typemap(in, fragment="Eigen_Fragments") Eigen::Isometry3d (Eigen::Isometry3d temp)
{
  // In: plain, non-const&, const&
  Eigen::Matrix4d temp_matrix;  
  if (!ConvertFromNumpyToEigenMatrix<Eigen::Matrix4d>(&temp_matrix, $input))
    SWIG_fail;
  temp = temp_matrix;
  $1 = temp;
}
%typemap(out, fragment="Eigen_Fragments") Eigen::Isometry3d, Eigen::Isometry3d const
{
  Eigen::Matrix4d temp_matrix=$1.matrix();
  if (!ConvertFromEigenToNumPyMatrix<Eigen::Matrix4d >(&$result, &temp_matrix))
    SWIG_fail;
}
%typemap(out, fragment="Eigen_Fragments") Eigen::Isometry3d&, Eigen::Isometry3d const&
{
  Eigen::Matrix4d temp_matrix=$1->matrix();
  if (!ConvertFromEigenToNumPyMatrix<Eigen::Matrix4d >(&$result, &temp_matrix))
    SWIG_fail;
}

// Argout: & (for returning values to in-out arguments)
%typemap(argout, fragment="Eigen_Fragments") Eigen::Isometry3d &
{
  // Argout: &
  PyObject* ret1 = $result;
  PyObject* ret2;
  Eigen::Matrix4d temp_matrix=$1->matrix();
  if (!ConvertFromEigenToNumPyMatrix<Eigen::Matrix4d >(&ret2, &temp_matrix))
    SWIG_fail;
  $result = PyTuple_Pack(2, ret1, ret2);
  Py_DECREF(ret1);
  Py_DECREF(ret2);
}

%typemap(in, numinputs=0) Eigen::Isometry3d & (Eigen::Isometry3d temp) {
  $1 = &temp;
}
*/