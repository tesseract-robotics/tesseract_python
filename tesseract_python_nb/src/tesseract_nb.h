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
