#pragma once

// Standard library
#include <iostream>
#include <vector>
#include <memory>
#include <string>
#include <unordered_map>
#include <map>
#include <set>
#include <array>
#include <functional>
#include <variant>
#include <stdexcept>

// Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

// nanobind core
#include <nanobind/nanobind.h>
#include <nanobind/trampoline.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/map.h>
#include <nanobind/stl/unordered_map.h>
#include <nanobind/stl/set.h>
#include <nanobind/stl/array.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/unique_ptr.h>
#include <nanobind/stl/function.h>
#include <nanobind/stl/variant.h>
#include <nanobind/stl/pair.h>
#include <nanobind/eigen/dense.h>

// Namespace aliases
namespace nb = nanobind;
using namespace nb::literals;

// Custom type caster for Eigen::Isometry3d (4x4 transform matrix)
namespace nanobind::detail {

template <>
struct type_caster<Eigen::Isometry3d> {
    NB_TYPE_CASTER(Eigen::Isometry3d, const_name("numpy.ndarray[numpy.float64[4, 4]]"))

    // Python -> C++: accept 4x4 numpy array
    bool from_python(handle src, uint8_t flags, cleanup_list* cleanup) noexcept {
        // Use the Matrix4d caster
        type_caster<Eigen::Matrix4d> matrix_caster;
        if (!matrix_caster.from_python(src, flags, cleanup))
            return false;

        value.matrix() = matrix_caster.value;
        return true;
    }

    // C++ -> Python: return as 4x4 numpy array
    static handle from_cpp(const Eigen::Isometry3d& src, rv_policy policy, cleanup_list* cleanup) noexcept {
        return type_caster<Eigen::Matrix4d>::from_cpp(src.matrix(), policy, cleanup);
    }
};

} // namespace nanobind::detail
