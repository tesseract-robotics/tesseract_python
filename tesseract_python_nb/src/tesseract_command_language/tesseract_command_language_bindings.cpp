/**
 * @file tesseract_command_language_bindings.cpp
 * @brief nanobind bindings for tesseract_command_language
 */

#include "tesseract_nb.h"
#include <nanobind/stl/map.h>
#include <nanobind/stl/unique_ptr.h>
#include <nanobind/stl/set.h>

// tesseract_command_language - only concrete types, avoid Poly wrappers for now
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_command_language/constants.h>

// tesseract_common
#include <tesseract_common/manipulator_info.h>
#include <tesseract_common/joint_state.h>

namespace tp = tesseract_planning;
namespace tc = tesseract_common;

NB_MODULE(_tesseract_command_language, m) {
    m.doc() = "tesseract_command_language Python bindings";

    // ========== JointWaypoint ==========
    nb::class_<tp::JointWaypoint>(m, "JointWaypoint")
        .def(nb::init<>())
        .def(nb::init<std::vector<std::string>, const Eigen::VectorXd&, bool>(),
             "names"_a, "position"_a, "is_constrained"_a = true)
        .def(nb::init<std::vector<std::string>, const Eigen::VectorXd&, const Eigen::VectorXd&, const Eigen::VectorXd&>(),
             "names"_a, "position"_a, "lower_tol"_a, "upper_tol"_a)
        .def("setNames", &tp::JointWaypoint::setNames, "names"_a)
        .def("getNames", [](const tp::JointWaypoint& self) { return self.getNames(); })
        .def("setPosition", &tp::JointWaypoint::setPosition, "position"_a)
        .def("getPosition", [](const tp::JointWaypoint& self) -> Eigen::VectorXd { return self.getPosition(); })
        .def("setUpperTolerance", &tp::JointWaypoint::setUpperTolerance, "upper_tol"_a)
        .def("getUpperTolerance", [](const tp::JointWaypoint& self) -> Eigen::VectorXd { return self.getUpperTolerance(); })
        .def("setLowerTolerance", &tp::JointWaypoint::setLowerTolerance, "lower_tol"_a)
        .def("getLowerTolerance", [](const tp::JointWaypoint& self) -> Eigen::VectorXd { return self.getLowerTolerance(); })
        .def("setIsConstrained", &tp::JointWaypoint::setIsConstrained, "value"_a)
        .def("isConstrained", &tp::JointWaypoint::isConstrained)
        .def("setName", &tp::JointWaypoint::setName, "name"_a)
        .def("getName", &tp::JointWaypoint::getName)
        .def("print", &tp::JointWaypoint::print, "prefix"_a = "");

    // ========== CartesianWaypoint ==========
    nb::class_<tp::CartesianWaypoint>(m, "CartesianWaypoint")
        .def(nb::init<>())
        .def(nb::init<const Eigen::Isometry3d&>(), "transform"_a)
        .def(nb::init<const Eigen::Isometry3d&, const Eigen::VectorXd&, const Eigen::VectorXd&>(),
             "transform"_a, "lower_tol"_a, "upper_tol"_a)
        .def("setTransform", &tp::CartesianWaypoint::setTransform, "transform"_a)
        .def("getTransform", [](const tp::CartesianWaypoint& self) -> Eigen::Isometry3d { return self.getTransform(); })
        .def("setUpperTolerance", &tp::CartesianWaypoint::setUpperTolerance, "upper_tol"_a)
        .def("getUpperTolerance", [](const tp::CartesianWaypoint& self) -> Eigen::VectorXd { return self.getUpperTolerance(); })
        .def("setLowerTolerance", &tp::CartesianWaypoint::setLowerTolerance, "lower_tol"_a)
        .def("getLowerTolerance", [](const tp::CartesianWaypoint& self) -> Eigen::VectorXd { return self.getLowerTolerance(); })
        .def("setSeed", &tp::CartesianWaypoint::setSeed, "seed"_a)
        .def("getSeed", [](const tp::CartesianWaypoint& self) { return self.getSeed(); })
        .def("setName", &tp::CartesianWaypoint::setName, "name"_a)
        .def("getName", &tp::CartesianWaypoint::getName)
        .def("print", &tp::CartesianWaypoint::print, "prefix"_a = "");

    // ========== StateWaypoint ==========
    nb::class_<tp::StateWaypoint>(m, "StateWaypoint")
        .def(nb::init<>())
        .def(nb::init<std::vector<std::string>, const Eigen::Ref<const Eigen::VectorXd>&>(),
             "joint_names"_a, "position"_a)
        .def("setNames", &tp::StateWaypoint::setNames, "names"_a)
        .def("getNames", [](const tp::StateWaypoint& self) { return self.getNames(); })
        .def("setPosition", &tp::StateWaypoint::setPosition, "position"_a)
        .def("getPosition", [](const tp::StateWaypoint& self) -> Eigen::VectorXd { return self.getPosition(); })
        .def("setVelocity", &tp::StateWaypoint::setVelocity, "velocity"_a)
        .def("getVelocity", [](const tp::StateWaypoint& self) -> Eigen::VectorXd { return self.getVelocity(); })
        .def("setAcceleration", &tp::StateWaypoint::setAcceleration, "acceleration"_a)
        .def("getAcceleration", [](const tp::StateWaypoint& self) -> Eigen::VectorXd { return self.getAcceleration(); })
        .def("setEffort", &tp::StateWaypoint::setEffort, "effort"_a)
        .def("getEffort", [](const tp::StateWaypoint& self) -> Eigen::VectorXd { return self.getEffort(); })
        .def("setTime", &tp::StateWaypoint::setTime, "time"_a)
        .def("getTime", &tp::StateWaypoint::getTime)
        .def("setName", &tp::StateWaypoint::setName, "name"_a)
        .def("getName", &tp::StateWaypoint::getName)
        .def("print", &tp::StateWaypoint::print, "prefix"_a = "");

    // ========== Constants ==========
    m.attr("DEFAULT_PROFILE_KEY") = std::string(tp::DEFAULT_PROFILE_KEY);
}
