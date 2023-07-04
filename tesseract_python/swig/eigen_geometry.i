/**
 * @file eigen_geometry.i
 * @brief Eigen geometry types used by Tesseract
 *
 * @author John Wason
 * @date December 18, 2020
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

namespace Eigen
{
    %pythondynamic AngleAxisd;
    %pythondynamic Isometry3d;
    %pythondynamic Quaterniond;
    %pythondynamic Translation3d;
    class AngleAxisd;
    class Quaterniond;
    class Isometry3d;
    class Translation3d;
    /**
        * @brief Eigen::Quaterniond wrapper

        * @details
        * This class is a wrapper around Eigen::Quaterniond that provides additional functionality and easier bindings
    */
    class Quaterniond
    {
    public:
        /**
            * @brief Default constructor, identity quaternion
        */
        Quaterniond();
        /**
            * @brief Construct from angle axis
            * @param aa Angle axis
        */
        Quaterniond(const AngleAxisd& aa);
        /**
            * @brief Construct from rotation matrix
            * @param other Rotation matrix
        */
        Quaterniond(const Eigen::Matrix3d& other);
        /**
            * @brief Construct from vector representing a quaternion
            * @param other Vector
        */
        Quterniondd(const Eigen::Vector4d& other);
        /**
            * @brief Construct from another quaternion
            * @param other Quaternion
        */
        Quaterniond(const Quaterniond& other);
        /**
            * @brief Construct from w, x, y, z
            * @param w W component
            * @param x X component
            * @param y Y component
            * @param z Z component
        */
        Quaterniond(double w, double x, double y, double z);

        Quaterniond FromTwoVectors(const Eigen::Vector3d a, const Eigen::Vector3d b);
        /**
            @brief Create random quaternion
        */
        static Quaterniond UnitRandom();

        /**
            @brief Get the angular distance between this and another quaternions

            @param other Other quaternion
            @return Angular distance in radians
        */
        double angularDistance(const Quaterniond& other);
        /**
            @brief Get the conjugate of this quaternion

            @return Conjugate of this quaternion
        */
        Quaterniond conjugate();

        /**
            @brief Get the dot product of this and another quaternion

            @param other Other quaternion
            @return Dot product
        */
        double dot(const Quaterniond& other);

        /**
            @brief Get the inverse of this quaternion

            @return Inverse of this quaternion
        */
        Quaterniond inverse();

        /**
            @brief Check if this quaternion is approximately equal to another

            @param other Other quaternion
            @return True if approximately equal
        */
        bool isApprox(const Quaterniond& other);
        /**
            @brief Check if this quaternion is approximately equal to another

            @param other Other quaternion
            @param prec Precision
            @return True if approximately equal
        */
        bool isApprox(const Quaterniond& other, double prec);
        /**
            @brief Get the norm of this quaternion

            @return Norm
        */
        double norm();

        /**
            @brief Normalize this quaternion
        */
        void normalize();

        /**
            @brief Get the normalized version of this quaternion

            @return Normalized quaternion
        */
        Quaterniond normalized();
        %rename(__mul__) operator*;

        /**
            @brief Multiply this quaternion by another quaternion

            @param other Other quaternion
            @return Result of multiplication
        */
        Quaterniond operator* (const Quaterniond& other);
        /**
            @brief Multiply this quaternion by an Isometry3d

            @param other Isometry3d
            @return Result of multiplication
        */
        Isometry3d operator* (const Isometry3d& other);
        /**
            @brief Multiply this quaternion by a Vector3d

            @param other Vector3d
            @return Result of multiplication
        */
        Eigen::Vector3d operator* (const Eigen::Vector3d& other);
        /**
            @brief Multiply this quaternion by a Matrix3d

            @param other Matrix3d
            @return Result of multiplication
        */
        Eigen::Matrix3d operator* (const Eigen::Matrix3d& other);
        /**
            @brief Multiply this quaternion by a Translation3d

            @param other Translation3d
            @return Result of multiplication
        */
        Isometry3d operator* (const Translation3d& other);

        Quaterniond setFromTwoVectors(const Eigen::Vector3d a, const Eigen::Vector3d b);

        /**
            @brief Set this quaternion to identity

            @return Reference to this quaternion
        */
        Quaterniond setIdentity();

        /**
            @brief Computer the spherical linear interpolation between this and another quaternion

            @param t Interpolation parameter
            @param other Other quaternion
            @return Result of interpolation
        */
        Quaterniond slerp(double t, const Eigen::Quaterniond& other);

        /**
            @brief Get the squared norm of this quaternion

            @return Squared norm
        */
        double squaredNorm();

        /**
            @brief Get the 3x3 rotation matrix of this quaternion

            @return Rotation matrix
        */
        Eigen::Matrix3d toRotationMatrix();

        /**
            @brief Get the vector representation of this quaternion

            @return Vector representation
        */
        Eigen::Vector3d vec();

        /**
            @brief Get the w component of this quaternion

            @return W component
        */
        double w();

        /**
            @brief Get the x component of this quaternion

            @return X component
        */
        double x();

        /**
            @brief Get the y component of this quaternion

            @return Y component
        */
        double y();

        /**
            @brief Get the z component of this quaternion

            @return Z component
        */
        double z();

        %extend
        {
            /**
                @brief Set the vector representation of this quaternion

                @param vec Vector representation
            */
            void setVec(const Eigen::Vector3d& vec)
            {
                $self->vec() = vec;
            }

            /**
                @brief Set the w component of this quaternion

                @param w W component
            */            
            void setW(double w)
            {
                $self->w() = w;
            }

            /**
                @brief Set the x component of this quaternion

                @param x X component
            */
            void setX(double x)
            {
                $self->x() = x;
            }

            /**
                @brief Set the y component of this quaternion

                @param y Y component
            */
            void setY(double y)
            {
                $self->y() = y;
            }
            /**
                @brief Set the z component of this quaternion

                @param z Z component
            */
            void setZ(double z)
            {
                $self->z() = z;
            }
        }
    };
    
    /**
        @brief Eigen::AngleAxisd bindings

        @details This class is a wrapper around Eigen::AngleAxisd
    */
    class AngleAxisd
    {
    public:
        /**
            @brief Create an identity AngleAxisd
        */
        AngleAxisd();
        /**
            @brief Create an AngleAxisd from another AngleAxisd

            @param other Other AngleAxisd
        */
        AngleAxisd(const AngleAxisd& other);
        /**
            @brief Create an AngleAxisd from a rotation matrix

            @param mat Rotation matrix
        */
        AngleAxisd(const Eigen::Matrix3d& mat);
        /**
            @brief Create an AngleAxisd from an angle and axis

            @param angle Angle
            @param axis Axis
        */
        AngleAxisd(double angle, const Eigen::Vector3d& axis);
        /**
            @brief Create an AngleAxisd from a quaternion

            @param q Quaternion
        */
        AngleAxisd(const Eigen::Quaterniond& q);

        /**
            @brief Get the angle of this AngleAxisd

            @return Angle
        */
        double angle();
        /**
            @brief Get the axis of this AngleAxisd

            @return Axis
        */
        Eigen::Vector3d axis();
        %extend
        {
            /**
                @brief Set the angle of this AngleAxisd

                @param angle Angle
            */
            void setAngle(double angle)
            {
                $self->angle() = angle;
            }

            /**
                @brief Set the axis of this AngleAxisd

                @param axis Axis
            */
            void setAxis(const Eigen::Vector3d& axis)
            {
                $self->axis() = axis;
            }
        }

        /**
            @brief Get the inverse of this AngleAxisd

            @return Inverse
        */
        AngleAxisd inverse();

        /**
            @brief Check if this AngleAxisd is approximately equal to another AngleAxisd

            @param other Other AngleAxisd
            @return True if approximately equal, false otherwise
        */
        bool isApprox(const AngleAxisd& other);

        /**
            @brief Check if this AngleAxisd is approximately equal to another AngleAxisd

            @param other Other AngleAxisd
            @param prec Precision
            @return True if approximately equal, false otherwise
        */
        bool isApprox(const AngleAxisd& other, double prec);

        /**
            @brief Get the 3x3 rotation matrix of this AngleAxisd

            @return Rotation matrix
        */
        Eigen::Matrix3d toRotationMatrix();

        /**
            @brief Set from a rotation matrix

            @param mat Rotation matrix
            @return AngleAxisd
        */
        AngleAxisd fromRotationMatrix(const Eigen::Matrix3d& mat);

        %rename(__mul__) operator*;
        /**
            @brief Multiply this AngleAxisd by another AngleAxisd

            @param other Other AngleAxisd
            @return Result of multiplication
        */
        Quaterniond operator* (const AngleAxisd& other);
        /**
            @brief Multiply this AngleAxisd by a Quaterniond

            @param other Quaterniond
            @return Result of multiplication
        */
        Quaterniond operator* (const Quaterniond& other);
        /**
            @brief Multiply this AngleAxisd by a Translation3d

            @param other Translation3d
            @return Result of multiplication
        */
        Isometry3d operator* (const Isometry3d& other);
        /**
            @brief Multiply this AngleAxisd by a Vector3d

            @param other Vector3d
            @return Result of multiplication
        */
        Eigen::Vector3d operator* (const Eigen::Vector3d& other);
        /**
            @brief Multiply this AngleAxisd by a Matrix3d

            @param other Matrix3d
            @return Result of multiplication
        */
        Eigen::Matrix3d operator* (const Eigen::Matrix3d& other);
        /**
            @brief Multiply this AngleAxisd by a Translation3d

            @param other Translation3d
            @return Result of multiplication
        */
        Isometry3d operator* (const Translation3d& other);
    };

    /**
        @brief Eigen::Translation3d bindings

        @details This class is a wrapper around Eigen::Translation3d
    */
    class Translation3d
    {
    public:
        /**
            @brief Create an identity Translation3d
        */
        Translation3d();
        /**
            @brief Create a Translation3d from another Translation3d

            @param other Other Translation3d
        */
        Translation3d(const Translation3d& other);
        /**
            @brief Create a Translation3d from a vector

            @param vector Vector
        */
        Translation3d(const Eigen::Vector3d& vector);
        /**
            @brief Create a Translation3d from x, y, and z components

            @param x X component
            @param y Y component
            @param z Z component
        */
        Translation3d(double x, double y, double z);

        %rename(__mul__) operator*;
        /**
            @brief Multiply this Translation3d by another Translation3d

            @param other Other Translation3d
            @return Result of multiplication
        */     
        Translation3d operator* (const Translation3d& other);
        /**
            @brief Multiply this Translation3d by a Quaterniond

            @param other Quaterniond
            @return Result of multiplication
        */
        Isometry3d operator* (const Isometry3d& other);
        /**
            @brief Multiply this Translation3d by a Vector3d

            @param other Vector3d
            @return Result of multiplication
        */
        Isometry3d operator* (const AngleAxisd& other);
        /**
            @brief Multiply this Translation3d by a Matrix3d

            @param other Matrix3d
            @return Result of multiplication
        */
        Isometry3d operator* (const Quaterniond& other);

        /**
            @brief Get the x component of this Translation3d

            @return X component
        */           
        double x();
        /**
            @brief Get the y component of this Translation3d

            @return Y component
        */
        double y();
        /**
            @brief Get the z component of this Translation3d

            @return Z component
        */
        double z();
        
        %extend
        {              
            /**
                @brief Set the x component of this Translation3d

                @param x X component
            */
            void setX(double x)
            {
                $self->x() = x;
            }
            /**
                @brief Set the y component of this Translation3d

                @param y Y component
            */
            void setY(double y)
            {
                $self->y() = y;
            }
            /**
                @brief Set the z component of this Translation3d

                @param z Z component
            */
            void setZ(double z)
            {
                $self->z() = z;
            }
        }

    };

    /**
        @brief Eigen::Isometry3d bindings, used for Transforms in Tesseract

        @details This class is a wrapper around Eigen::Isometry3d
    */
    class Isometry3d
    {
    public:
        /**
            @brief Create an identity Isometry3d
        */
        Isometry3d();
        /**
            @brief Create an Isometry3d from a 4x4 holonomic matrix

            @param mat 4x4 holonomic matrix
        */
        Isometry3d(const Eigen::Matrix4d& mat);
        
        /**
            @brief Check if this Isometry3d is approximately equal to another Isometry3d

            @param other Other Isometry3d
            @return True if approximately equal, false otherwise
        */
        bool isApprox(const Eigen::Isometry3d& other);
        /**
            @brief Check if this Isometry3d is approximately equal to another Isometry3d

            @param other Other Isometry3d
            @param prec Precision
            @return True if approximately equal, false otherwise
        */
        bool isApprox(const Eigen::Isometry3d& other, double prec);

        /**
            @brief Get the inverse of this Isometry3d

            @return Inverse of this Isometry3d
        */
        Isometry3d inverse();

        /**
            @brief Get the 4x4 holonomic matrix of this Isometry3d

            @return 4x4 holonomic matrix of this Isometry3d
        */
        Eigen::Matrix4d matrix();
        /**
            @brief Get the rotation matrix of this Isometry3d

            @return Rotation matrix of this Isometry3d
        */
        Eigen::Matrix3d rotation();
        /**
            @brief Get the translation vector of this Isometry3d

            @return Translation vector of this Isometry3d
        */
        Eigen::Vector3d translation();
        /**
            @brief Get the linear matrix of this Isometry3d

            @return Linear matrix of this Isometry3d
        */
        Eigen::Matrix3d linear();

        %extend {
            /**
                @brief Set the 4x4 holonomic matrix of this Isometry3d

                @param matrix 4x4 holonomic matrix
            */
            void setMatrix(const Eigen::Matrix4d& matrix)
            {
                $self->matrix() = matrix;            
            }
            /**
                @brief Set the translation vector of this Isometry3d

                @param translation Translation vector
            */
            void setTranslation(const Eigen::Vector3d& translation)
            {
                $self->translation() = translation;
            }
            /**
                @brief Set the linear matrix of this Isometry3d

                @param linear Linear matrix
            */
            void setLinear(const Eigen::Matrix3d& linear)
            {
                $self->linear() = linear;
            }
        }

        %rename(__mul__) operator*;

        /**
            @brief Multiply this Isometry3d by another Isometry3d

            @param other Other Isometry3d
            @return Result of multiplication
        */
        Isometry3d operator* (const Isometry3d& other);
        /**
            @brief Multiply this Isometry3d by an AngleAxisd

            @param other AngleAxisd
            @return Result of multiplication
        */
        Isometry3d operator* (const AngleAxisd& other);
        /**
            @brief Multiply this Isometry3d by a Quaterniond

            @param other Quaterniond
            @return Result of multiplication
        */
        Isometry3d operator* (const Quaterniond& other);
        /**
            @brief Multiply this Isometry3d by a Translation3d

            @param other Translation3d
            @return Result of multiplication
        */
        Isometry3d operator* (const Translation3d& other);
        /**
            @brief Multiply this Isometry3d by a holonomic matrix

            @param other Matrix4d
            @return Result of multiplication
        */
        Eigen::Matrix4d operator* (const Eigen::Matrix4d& other);

        /**
            @brief Rotate this Isometry3d by a Matrix3d

            @param rotation Matrix3d
        */
        void rotate(const Eigen::Matrix3d& rotation);
        /**
            @brief Rotate this Isometry3d by an AngleAxisd

            @param rotation AngleAxisd
        */
        void rotate(const AngleAxisd& rotation);
        /**
            @brief Rotate this Isometry3d by a Quaterniond

            @param rotation Quaterniond
        */
        void rotate(const Quaterniond& rotation);
        /**
            @brief Translate this Isometry3d by a Vector3d

            @param vec Vector3d
        */
        void translate(const Eigen::Vector3d& vec);
        
        /**
            @brief Pre-rotate this Isometry3d by a Matrix3d

            @param rotation Matrix3d
        */
        void prerotate(const Eigen::Matrix3d& rotation);
        /**
            @brief Pre-rotate this Isometry3d by an AngleAxisd

            @param rotation AngleAxisd
        */
        void prerotate(const AngleAxisd& rotation);
        /**
            @brief Pre-rotate this Isometry3d by a Quaterniond

            @param rotation Quaterniond
        */
        void prerotate(const Quaterniond& rotation);
        /**
            @brief Pre-translate this Isometry3d by a Vector3d

            @param vec Vector3d
        */
        void pretranslate(const Eigen::Vector3d& vec);
        
        /**
            @brief Set this Isometry3d to the identity
        */
        void setIdentity();

        /**
            @brief Get the identity Isometry3d

            @return Identity Isometry3d
        */
        static Isometry3d Identity();

    };
}

// Argout: & (for returning values to in-out arguments)
%typemap(argout) Eigen::Isometry3d &
{
  // Argout: &
  PyObject* ret1 = $result;
  //  PyObject* ret2 = SWIG_NewPointerObj(SWIG_as_voidptr($1), SWIGTYPE_p_Eigen__Isometry3d, SWIG_POINTER_NEW |  0 );  
  PyObject* ret2 = SWIG_NewPointerObj(%new_copy(*$1, $*ltype), $descriptor, SWIG_POINTER_OWN | %newpointer_flags);
  $result = PyTuple_Pack(2, ret1, ret2);
  Py_DECREF(ret1);
  Py_DECREF(ret2);
}

%typemap(in, numinputs=0) Eigen::Isometry3d& (Eigen::Isometry3d temp) {
  $1 = &temp;
}

// Default typemap for const & Eigen::Isometry3d
%typemap(in, noblock=1) Eigen::Isometry3d const& (void *argp = 0, int res = 0) {
  res = SWIG_ConvertPtr($input, &argp, $descriptor, %convertptr_flags);
  if (!SWIG_IsOK(res)) {
    %argument_fail(res, "$type", $symname, $argnum); 
  }
  if (!argp) { %argument_nullref("$type", $symname, $argnum); }
  $1 = %reinterpret_cast(argp, $ltype);
}
%typemap(freearg) Eigen::Isometry3d const& "";
%typemap(argout) Eigen::Isometry3d const& "";
