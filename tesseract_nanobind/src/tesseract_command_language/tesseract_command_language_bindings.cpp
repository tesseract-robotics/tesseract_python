/**
 * @file tesseract_command_language_bindings.cpp
 * @brief nanobind bindings for tesseract_command_language
 */

#include "tesseract_nb.h"
#include <nanobind/stl/map.h>
#include <nanobind/stl/unique_ptr.h>
#include <nanobind/stl/set.h>
#include <nanobind/stl/function.h>

// tesseract_command_language - concrete types
#include <tesseract_command_language/joint_waypoint.h>
#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/profile.h>
#include <tesseract_command_language/profile_dictionary.h>
#include <tesseract_command_language/constants.h>

// tesseract_command_language - Poly types
#include <tesseract_command_language/poly/waypoint_poly.h>
#include <tesseract_command_language/poly/cartesian_waypoint_poly.h>
#include <tesseract_command_language/poly/joint_waypoint_poly.h>
#include <tesseract_command_language/poly/state_waypoint_poly.h>
#include <tesseract_command_language/poly/move_instruction_poly.h>
#include <tesseract_command_language/poly/instruction_poly.h>

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

    // ========== MoveInstructionType enum ==========
    nb::enum_<tp::MoveInstructionType>(m, "MoveInstructionType")
        .value("LINEAR", tp::MoveInstructionType::LINEAR)
        .value("FREESPACE", tp::MoveInstructionType::FREESPACE)
        .value("CIRCULAR", tp::MoveInstructionType::CIRCULAR);

    // For SWIG compatibility
    m.attr("MoveInstructionType_LINEAR") = tp::MoveInstructionType::LINEAR;
    m.attr("MoveInstructionType_FREESPACE") = tp::MoveInstructionType::FREESPACE;
    m.attr("MoveInstructionType_CIRCULAR") = tp::MoveInstructionType::CIRCULAR;

    // ========== CompositeInstructionOrder enum ==========
    nb::enum_<tp::CompositeInstructionOrder>(m, "CompositeInstructionOrder")
        .value("ORDERED", tp::CompositeInstructionOrder::ORDERED)
        .value("UNORDERED", tp::CompositeInstructionOrder::UNORDERED)
        .value("ORDERED_AND_REVERABLE", tp::CompositeInstructionOrder::ORDERED_AND_REVERABLE);

    // ========== WaypointPoly ==========
    nb::class_<tp::WaypointPoly>(m, "WaypointPoly")
        .def(nb::init<>())
        .def("setName", &tp::WaypointPoly::setName, "name"_a)
        .def("getName", &tp::WaypointPoly::getName)
        .def("print", &tp::WaypointPoly::print, "prefix"_a = "")
        .def("isCartesianWaypoint", &tp::WaypointPoly::isCartesianWaypoint)
        .def("isJointWaypoint", &tp::WaypointPoly::isJointWaypoint)
        .def("isStateWaypoint", &tp::WaypointPoly::isStateWaypoint)
        .def("isNull", &tp::WaypointPoly::isNull);

    // ========== CartesianWaypointPoly ==========
    // Note: Not binding as subclass of WaypointPoly due to nanobind limitations
    nb::class_<tp::CartesianWaypointPoly>(m, "CartesianWaypointPoly")
        .def(nb::init<>())
        .def(nb::init<tp::CartesianWaypoint>(), "waypoint"_a)
        .def("getTransform", [](const tp::CartesianWaypointPoly& self) -> Eigen::Isometry3d {
            return self.getTransform();
        })
        .def("setTransform", &tp::CartesianWaypointPoly::setTransform, "transform"_a)
        .def("hasSeed", &tp::CartesianWaypointPoly::hasSeed)
        .def("clearSeed", &tp::CartesianWaypointPoly::clearSeed)
        // Re-expose WaypointPoly methods
        .def("setName", &tp::CartesianWaypointPoly::setName, "name"_a)
        .def("getName", &tp::CartesianWaypointPoly::getName)
        .def("print", &tp::CartesianWaypointPoly::print, "prefix"_a = "")
        .def("isNull", &tp::CartesianWaypointPoly::isNull);

    // ========== JointWaypointPoly ==========
    nb::class_<tp::JointWaypointPoly>(m, "JointWaypointPoly")
        .def(nb::init<>())
        .def(nb::init<tp::JointWaypoint>(), "waypoint"_a)
        .def("getNames", [](const tp::JointWaypointPoly& self) { return self.getNames(); })
        .def("setNames", &tp::JointWaypointPoly::setNames, "names"_a)
        .def("getPosition", [](const tp::JointWaypointPoly& self) -> Eigen::VectorXd {
            return self.getPosition();
        })
        .def("setPosition", &tp::JointWaypointPoly::setPosition, "position"_a)
        .def("isConstrained", &tp::JointWaypointPoly::isConstrained)
        .def("setIsConstrained", &tp::JointWaypointPoly::setIsConstrained, "value"_a)
        // Re-expose WaypointPoly methods
        .def("setName", &tp::JointWaypointPoly::setName, "name"_a)
        .def("getName", &tp::JointWaypointPoly::getName)
        .def("print", &tp::JointWaypointPoly::print, "prefix"_a = "")
        .def("isNull", &tp::JointWaypointPoly::isNull);

    // ========== StateWaypointPoly ==========
    nb::class_<tp::StateWaypointPoly>(m, "StateWaypointPoly")
        .def(nb::init<>())
        .def(nb::init<tp::StateWaypoint>(), "waypoint"_a)
        .def("getNames", [](const tp::StateWaypointPoly& self) { return self.getNames(); })
        .def("setNames", &tp::StateWaypointPoly::setNames, "names"_a)
        .def("getPosition", [](const tp::StateWaypointPoly& self) -> Eigen::VectorXd {
            return self.getPosition();
        })
        .def("setPosition", &tp::StateWaypointPoly::setPosition, "position"_a)
        .def("getVelocity", [](const tp::StateWaypointPoly& self) -> Eigen::VectorXd {
            return self.getVelocity();
        })
        .def("setVelocity", &tp::StateWaypointPoly::setVelocity, "velocity"_a)
        .def("getAcceleration", [](const tp::StateWaypointPoly& self) -> Eigen::VectorXd {
            return self.getAcceleration();
        })
        .def("setAcceleration", &tp::StateWaypointPoly::setAcceleration, "acceleration"_a)
        .def("getTime", &tp::StateWaypointPoly::getTime)
        .def("setTime", &tp::StateWaypointPoly::setTime, "time"_a)
        // Re-expose WaypointPoly methods
        .def("setName", &tp::StateWaypointPoly::setName, "name"_a)
        .def("getName", &tp::StateWaypointPoly::getName)
        .def("print", &tp::StateWaypointPoly::print, "prefix"_a = "")
        .def("isNull", &tp::StateWaypointPoly::isNull);

    // ========== Helper functions for Poly wrapping (SWIG compatibility) ==========
    m.def("CartesianWaypointPoly_wrap_CartesianWaypoint", [](const tp::CartesianWaypoint& wp) {
        return tp::CartesianWaypointPoly(wp);
    }, "waypoint"_a);

    m.def("JointWaypointPoly_wrap_JointWaypoint", [](const tp::JointWaypoint& wp) {
        return tp::JointWaypointPoly(wp);
    }, "waypoint"_a);

    m.def("StateWaypointPoly_wrap_StateWaypoint", [](const tp::StateWaypoint& wp) {
        return tp::StateWaypointPoly(wp);
    }, "waypoint"_a);

    // ========== InstructionPoly ==========
    nb::class_<tp::InstructionPoly>(m, "InstructionPoly")
        .def(nb::init<>())
        .def("getDescription", &tp::InstructionPoly::getDescription)
        .def("setDescription", &tp::InstructionPoly::setDescription, "description"_a)
        .def("print", &tp::InstructionPoly::print, "prefix"_a = "")
        .def("isCompositeInstruction", &tp::InstructionPoly::isCompositeInstruction)
        .def("isMoveInstruction", &tp::InstructionPoly::isMoveInstruction)
        .def("isNull", &tp::InstructionPoly::isNull)
        // Workaround for RTTI issues across shared library boundaries
        // The standard as<T>() uses typeid() which fails when comparing types from different .so files
        // Since isMoveInstruction() works (uses typeid() within C++ lib), we can use getInterface().recover()
        // to get the underlying MoveInstructionPoly* and copy it
        .def("asMoveInstruction", [](tp::InstructionPoly& self) -> tp::MoveInstructionPoly {
            if (!self.isMoveInstruction())
                throw std::runtime_error("InstructionPoly is not a MoveInstruction");
            auto* ptr = static_cast<tp::MoveInstructionPoly*>(self.getInterface().recover());
            return *ptr;
        }, "Cast to MoveInstructionPoly. Raises RuntimeError if not a move instruction.");

    // ========== MoveInstructionPoly ==========
    // Note: Not binding as subclass due to nanobind cross-module limitations
    nb::class_<tp::MoveInstructionPoly>(m, "MoveInstructionPoly")
        .def(nb::init<>())
        .def("getWaypoint", [](tp::MoveInstructionPoly& self) -> tp::WaypointPoly& {
            return self.getWaypoint();
        }, nb::rv_policy::reference_internal)
        .def("assignCartesianWaypoint", &tp::MoveInstructionPoly::assignCartesianWaypoint, "waypoint"_a)
        .def("assignJointWaypoint", &tp::MoveInstructionPoly::assignJointWaypoint, "waypoint"_a)
        .def("assignStateWaypoint", &tp::MoveInstructionPoly::assignStateWaypoint, "waypoint"_a)
        .def("getMoveType", &tp::MoveInstructionPoly::getMoveType)
        .def("setMoveType", &tp::MoveInstructionPoly::setMoveType, "move_type"_a)
        .def("getProfile", &tp::MoveInstructionPoly::getProfile, "ns"_a = "")
        .def("setProfile", &tp::MoveInstructionPoly::setProfile, "profile"_a)
        .def("getPathProfile", &tp::MoveInstructionPoly::getPathProfile, "ns"_a = "")
        .def("setPathProfile", &tp::MoveInstructionPoly::setPathProfile, "profile"_a)
        .def("getManipulatorInfo", [](const tp::MoveInstructionPoly& self) {
            return self.getManipulatorInfo();
        })
        .def("setManipulatorInfo", &tp::MoveInstructionPoly::setManipulatorInfo, "info"_a)
        // Re-expose InstructionPoly-like methods
        .def("getDescription", &tp::MoveInstructionPoly::getDescription)
        .def("setDescription", &tp::MoveInstructionPoly::setDescription, "description"_a)
        .def("print", &tp::MoveInstructionPoly::print, "prefix"_a = "")
        .def("isNull", &tp::MoveInstructionPoly::isNull);

    // Helper for SWIG compatibility
    m.def("MoveInstructionPoly_wrap_MoveInstruction", [](const tp::MoveInstruction& mi) {
        return tp::MoveInstructionPoly(mi);
    }, "instruction"_a);

    // Workaround for RTTI issues across shared library boundaries
    // The C++ as<T>() method uses typeid() which generates different type_info in Python bindings
    // Since isMoveInstruction() works (uses typeid() within C++ lib), we can use getInterface().recover()
    // to get the underlying MoveInstructionPoly* and copy it
    m.def("InstructionPoly_as_MoveInstructionPoly", [](tp::InstructionPoly& ip) -> tp::MoveInstructionPoly {
        if (!ip.isMoveInstruction())
            throw std::runtime_error("InstructionPoly is not a MoveInstruction");
        // The internal value stores a MoveInstructionPoly - retrieve it via recover()
        // recover() returns void* to the actual stored value
        auto* ptr = static_cast<tp::MoveInstructionPoly*>(ip.getInterface().recover());
        return *ptr;  // Copy
    }, "instruction"_a);

    m.def("WaypointPoly_as_StateWaypointPoly", [](tp::WaypointPoly& wp) -> tp::StateWaypointPoly {
        if (!wp.isStateWaypoint())
            throw std::runtime_error("WaypointPoly is not a StateWaypoint");
        auto* ptr = static_cast<tp::StateWaypointPoly*>(wp.getInterface().recover());
        return *ptr;
    }, "waypoint"_a);

    m.def("WaypointPoly_as_CartesianWaypointPoly", [](tp::WaypointPoly& wp) -> tp::CartesianWaypointPoly {
        if (!wp.isCartesianWaypoint())
            throw std::runtime_error("WaypointPoly is not a CartesianWaypoint");
        auto* ptr = static_cast<tp::CartesianWaypointPoly*>(wp.getInterface().recover());
        return *ptr;
    }, "waypoint"_a);

    m.def("WaypointPoly_as_JointWaypointPoly", [](tp::WaypointPoly& wp) -> tp::JointWaypointPoly {
        if (!wp.isJointWaypoint())
            throw std::runtime_error("WaypointPoly is not a JointWaypoint");
        auto* ptr = static_cast<tp::JointWaypointPoly*>(wp.getInterface().recover());
        return *ptr;
    }, "waypoint"_a);

    // ========== MoveInstruction ==========
    nb::class_<tp::MoveInstruction>(m, "MoveInstruction")
        .def(nb::init<tp::CartesianWaypointPoly, tp::MoveInstructionType>(),
             "waypoint"_a, "type"_a)
        .def(nb::init<tp::JointWaypointPoly, tp::MoveInstructionType>(),
             "waypoint"_a, "type"_a)
        .def(nb::init<tp::StateWaypointPoly, tp::MoveInstructionType>(),
             "waypoint"_a, "type"_a)
        // SWIG-compatible constructors with profile parameter
        .def(nb::init<tp::CartesianWaypointPoly, tp::MoveInstructionType, std::string>(),
             "waypoint"_a, "type"_a, "profile"_a)
        .def(nb::init<tp::JointWaypointPoly, tp::MoveInstructionType, std::string>(),
             "waypoint"_a, "type"_a, "profile"_a)
        .def(nb::init<tp::StateWaypointPoly, tp::MoveInstructionType, std::string>(),
             "waypoint"_a, "type"_a, "profile"_a)
        .def("getWaypoint", [](tp::MoveInstruction& self) -> tp::WaypointPoly& {
            return self.getWaypoint();
        }, nb::rv_policy::reference_internal)
        .def("assignCartesianWaypoint", &tp::MoveInstruction::assignCartesianWaypoint, "waypoint"_a)
        .def("assignJointWaypoint", &tp::MoveInstruction::assignJointWaypoint, "waypoint"_a)
        .def("assignStateWaypoint", &tp::MoveInstruction::assignStateWaypoint, "waypoint"_a)
        .def("getMoveType", &tp::MoveInstruction::getMoveType)
        .def("setMoveType", &tp::MoveInstruction::setMoveType, "move_type"_a)
        .def("getProfile", &tp::MoveInstruction::getProfile, "ns"_a = "")
        .def("setProfile", &tp::MoveInstruction::setProfile, "profile"_a)
        .def("getPathProfile", &tp::MoveInstruction::getPathProfile, "ns"_a = "")
        .def("setPathProfile", &tp::MoveInstruction::setPathProfile, "profile"_a)
        .def("getManipulatorInfo", [](const tp::MoveInstruction& self) {
            return self.getManipulatorInfo();
        })
        .def("setManipulatorInfo", &tp::MoveInstruction::setManipulatorInfo, "info"_a)
        .def("print", &tp::MoveInstruction::print, "prefix"_a = "")
        .def("getDescription", &tp::MoveInstruction::getDescription)
        .def("setDescription", &tp::MoveInstruction::setDescription, "description"_a);

    // ========== CompositeInstruction ==========
    nb::class_<tp::CompositeInstruction>(m, "CompositeInstruction")
        .def(nb::init<>())
        .def(nb::init<std::string>(), "profile"_a)  // SWIG-compatible constructor
        .def("getOrder", &tp::CompositeInstruction::getOrder)
        .def("getDescription", &tp::CompositeInstruction::getDescription)
        .def("setDescription", &tp::CompositeInstruction::setDescription, "description"_a)
        .def("print", &tp::CompositeInstruction::print, "prefix"_a = "")
        .def("getProfile", &tp::CompositeInstruction::getProfile, "ns"_a = "")
        .def("setProfile", &tp::CompositeInstruction::setProfile, "profile"_a)
        .def("getManipulatorInfo", [](const tp::CompositeInstruction& self) {
            return self.getManipulatorInfo();
        })
        .def("setManipulatorInfo", &tp::CompositeInstruction::setManipulatorInfo, "info"_a)
        .def("getInstructions", [](tp::CompositeInstruction& self) -> std::vector<tp::InstructionPoly>& {
            return self.getInstructions();
        }, nb::rv_policy::reference_internal)
        .def("setInstructions", &tp::CompositeInstruction::setInstructions, "instructions"_a)
        .def("appendMoveInstruction", [](tp::CompositeInstruction& self, const tp::MoveInstructionPoly& mi) {
            self.appendMoveInstruction(mi);
        }, "mi"_a)
        .def("size", &tp::CompositeInstruction::size)
        .def("empty", &tp::CompositeInstruction::empty)
        .def("clear", &tp::CompositeInstruction::clear)
        .def("__len__", &tp::CompositeInstruction::size)
        .def("__getitem__", [](tp::CompositeInstruction& self, std::size_t i) -> tp::InstructionPoly& {
            if (i >= self.size()) throw std::out_of_range("Index out of range");
            return self[i];
        }, nb::rv_policy::reference_internal)
        .def("__iter__", [](tp::CompositeInstruction& self) {
            return nb::make_iterator(nb::type<tp::CompositeInstruction>(), "iterator", self.begin(), self.end());
        }, nb::keep_alive<0, 1>());

    // ========== Profile (base class) ==========
    nb::class_<tp::Profile>(m, "Profile")
        .def(nb::init<>())
        .def(nb::init<std::size_t>(), "key"_a)
        .def("getKey", &tp::Profile::getKey);

    // ========== ProfileDictionary ==========
    nb::class_<tp::ProfileDictionary>(m, "ProfileDictionary")
        .def(nb::init<>())
        .def("addProfile", nb::overload_cast<const std::string&, const std::string&, const tp::Profile::ConstPtr&>(
            &tp::ProfileDictionary::addProfile), "ns"_a, "profile_name"_a, "profile"_a)
        .def("hasProfile", &tp::ProfileDictionary::hasProfile, "key"_a, "ns"_a, "profile_name"_a)
        .def("getProfile", &tp::ProfileDictionary::getProfile, "key"_a, "ns"_a, "profile_name"_a)
        .def("removeProfile", &tp::ProfileDictionary::removeProfile, "key"_a, "ns"_a, "profile_name"_a)
        .def("hasProfileEntry", &tp::ProfileDictionary::hasProfileEntry, "key"_a, "ns"_a)
        .def("removeProfileEntry", &tp::ProfileDictionary::removeProfileEntry, "key"_a, "ns"_a)
        .def("clear", &tp::ProfileDictionary::clear);

    // Helper function to add profiles from other modules (cross-module inheritance workaround)
    // This casts Profile::ConstPtr from other modules to the base type
    m.def("ProfileDictionary_addProfile", [](tp::ProfileDictionary& dict,
                                              const std::string& ns,
                                              const std::string& profile_name,
                                              tp::Profile::ConstPtr profile) {
        dict.addProfile(ns, profile_name, profile);
    }, "dict"_a, "ns"_a, "profile_name"_a, "profile"_a,
    "Add a profile to the dictionary (cross-module helper)");
}
