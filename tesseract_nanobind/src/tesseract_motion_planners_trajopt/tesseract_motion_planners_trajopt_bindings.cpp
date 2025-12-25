/**
 * @file tesseract_motion_planners_trajopt_bindings.cpp
 * @brief nanobind bindings for tesseract_motion_planners TrajOpt
 */

#include "tesseract_nb.h"

// tesseract_motion_planners core (for PlannerRequest/Response)
#include <tesseract_motion_planners/core/types.h>

// tesseract_command_language (for Profile base class)
#include <tesseract_command_language/profile.h>
#include <tesseract_command_language/profile_dictionary.h>

// tesseract_motion_planners TrajOpt
#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/trajopt_collision_config.h>
#include <tesseract_motion_planners/trajopt/trajopt_waypoint_config.h>

// trajopt for CollisionEvaluatorType
#include <trajopt/problem_description.hpp>

// tesseract_collision for ContactTestType
#include <tesseract_collision/core/types.h>

namespace tp = tesseract_planning;

NB_MODULE(_tesseract_motion_planners_trajopt, m) {
    m.doc() = "tesseract_motion_planners_trajopt Python bindings";

    // Import Profile type from tesseract_command_language for cross-module inheritance
    auto cl_module = nb::module_::import_("tesseract_robotics.tesseract_command_language._tesseract_command_language");

    // Import tesseract_collision for ContactTestType (used by TrajOptDefaultCompositeProfile)
    nb::module_::import_("tesseract_robotics.tesseract_collision._tesseract_collision");

    // ========== trajopt::CollisionEvaluatorType enum ==========
    nb::enum_<trajopt::CollisionEvaluatorType>(m, "CollisionEvaluatorType")
        .value("SINGLE_TIMESTEP", trajopt::CollisionEvaluatorType::SINGLE_TIMESTEP)
        .value("DISCRETE_CONTINUOUS", trajopt::CollisionEvaluatorType::DISCRETE_CONTINUOUS)
        .value("CAST_CONTINUOUS", trajopt::CollisionEvaluatorType::CAST_CONTINUOUS);

    // ========== TrajOptCartesianWaypointConfig ==========
    nb::class_<tp::TrajOptCartesianWaypointConfig>(m, "TrajOptCartesianWaypointConfig")
        .def(nb::init<>())
        .def_rw("enabled", &tp::TrajOptCartesianWaypointConfig::enabled)
        .def_rw("use_tolerance_override", &tp::TrajOptCartesianWaypointConfig::use_tolerance_override)
        .def_rw("lower_tolerance", &tp::TrajOptCartesianWaypointConfig::lower_tolerance)
        .def_rw("upper_tolerance", &tp::TrajOptCartesianWaypointConfig::upper_tolerance)
        .def_rw("coeff", &tp::TrajOptCartesianWaypointConfig::coeff);

    // ========== TrajOptJointWaypointConfig ==========
    nb::class_<tp::TrajOptJointWaypointConfig>(m, "TrajOptJointWaypointConfig")
        .def(nb::init<>())
        .def_rw("enabled", &tp::TrajOptJointWaypointConfig::enabled)
        .def_rw("use_tolerance_override", &tp::TrajOptJointWaypointConfig::use_tolerance_override)
        .def_rw("lower_tolerance", &tp::TrajOptJointWaypointConfig::lower_tolerance)
        .def_rw("upper_tolerance", &tp::TrajOptJointWaypointConfig::upper_tolerance)
        .def_rw("coeff", &tp::TrajOptJointWaypointConfig::coeff);

    // ========== CollisionCostConfig ==========
    nb::class_<tp::CollisionCostConfig>(m, "CollisionCostConfig")
        .def(nb::init<>())
        .def_rw("enabled", &tp::CollisionCostConfig::enabled)
        .def_rw("use_weighted_sum", &tp::CollisionCostConfig::use_weighted_sum)
        .def_rw("type", &tp::CollisionCostConfig::type)
        .def_rw("safety_margin", &tp::CollisionCostConfig::safety_margin)
        .def_rw("safety_margin_buffer", &tp::CollisionCostConfig::safety_margin_buffer)
        .def_rw("coeff", &tp::CollisionCostConfig::coeff);

    // ========== CollisionConstraintConfig ==========
    nb::class_<tp::CollisionConstraintConfig>(m, "CollisionConstraintConfig")
        .def(nb::init<>())
        .def_rw("enabled", &tp::CollisionConstraintConfig::enabled)
        .def_rw("use_weighted_sum", &tp::CollisionConstraintConfig::use_weighted_sum)
        .def_rw("type", &tp::CollisionConstraintConfig::type)
        .def_rw("safety_margin", &tp::CollisionConstraintConfig::safety_margin)
        .def_rw("safety_margin_buffer", &tp::CollisionConstraintConfig::safety_margin_buffer)
        .def_rw("coeff", &tp::CollisionConstraintConfig::coeff);

    // ========== TrajOptPlanProfile (base) ==========
    nb::class_<tp::TrajOptPlanProfile, tp::Profile>(m, "TrajOptPlanProfile")
        .def("getKey", &tp::TrajOptPlanProfile::getKey)
        .def_static("getStaticKey", &tp::TrajOptPlanProfile::getStaticKey);

    // ========== TrajOptCompositeProfile (base) ==========
    nb::class_<tp::TrajOptCompositeProfile, tp::Profile>(m, "TrajOptCompositeProfile")
        .def("getKey", &tp::TrajOptCompositeProfile::getKey)
        .def_static("getStaticKey", &tp::TrajOptCompositeProfile::getStaticKey);

    // ========== TrajOptDefaultPlanProfile ==========
    nb::class_<tp::TrajOptDefaultPlanProfile, tp::TrajOptPlanProfile>(m, "TrajOptDefaultPlanProfile")
        .def(nb::init<>())
        .def_rw("cartesian_cost_config", &tp::TrajOptDefaultPlanProfile::cartesian_cost_config)
        .def_rw("cartesian_constraint_config", &tp::TrajOptDefaultPlanProfile::cartesian_constraint_config)
        .def_rw("joint_cost_config", &tp::TrajOptDefaultPlanProfile::joint_cost_config)
        .def_rw("joint_constraint_config", &tp::TrajOptDefaultPlanProfile::joint_constraint_config);

    // ========== TrajOptDefaultCompositeProfile ==========
    nb::class_<tp::TrajOptDefaultCompositeProfile, tp::TrajOptCompositeProfile>(m, "TrajOptDefaultCompositeProfile")
        .def(nb::init<>())
        .def_rw("contact_test_type", &tp::TrajOptDefaultCompositeProfile::contact_test_type)
        .def_rw("collision_cost_config", &tp::TrajOptDefaultCompositeProfile::collision_cost_config)
        .def_rw("collision_constraint_config", &tp::TrajOptDefaultCompositeProfile::collision_constraint_config)
        .def_rw("smooth_velocities", &tp::TrajOptDefaultCompositeProfile::smooth_velocities)
        .def_rw("smooth_accelerations", &tp::TrajOptDefaultCompositeProfile::smooth_accelerations)
        .def_rw("smooth_jerks", &tp::TrajOptDefaultCompositeProfile::smooth_jerks)
        .def_rw("avoid_singularity", &tp::TrajOptDefaultCompositeProfile::avoid_singularity)
        .def_rw("avoid_singularity_coeff", &tp::TrajOptDefaultCompositeProfile::avoid_singularity_coeff)
        .def_rw("longest_valid_segment_fraction", &tp::TrajOptDefaultCompositeProfile::longest_valid_segment_fraction)
        .def_rw("longest_valid_segment_length", &tp::TrajOptDefaultCompositeProfile::longest_valid_segment_length);

    // Helper to add TrajOpt plan profile to ProfileDictionary directly
    m.def("ProfileDictionary_addTrajOptPlanProfile", [](tp::ProfileDictionary& dict,
                                                         const std::string& ns,
                                                         const std::string& profile_name,
                                                         std::shared_ptr<tp::TrajOptPlanProfile> profile) {
        dict.addProfile(ns, profile_name, profile);
    }, "dict"_a, "ns"_a, "profile_name"_a, "profile"_a,
    "Add TrajOpt plan profile to ProfileDictionary");

    // Helper to add TrajOpt composite profile to ProfileDictionary directly
    m.def("ProfileDictionary_addTrajOptCompositeProfile", [](tp::ProfileDictionary& dict,
                                                              const std::string& ns,
                                                              const std::string& profile_name,
                                                              std::shared_ptr<tp::TrajOptCompositeProfile> profile) {
        dict.addProfile(ns, profile_name, profile);
    }, "dict"_a, "ns"_a, "profile_name"_a, "profile"_a,
    "Add TrajOpt composite profile to ProfileDictionary");

    // ========== TrajOptMotionPlanner ==========
    nb::class_<tp::TrajOptMotionPlanner>(m, "TrajOptMotionPlanner")
        .def(nb::init<std::string>(), "name"_a)
        .def("getName", &tp::TrajOptMotionPlanner::getName)
        .def("solve", &tp::TrajOptMotionPlanner::solve, "request"_a, nb::call_guard<nb::gil_scoped_release>())
        .def("terminate", &tp::TrajOptMotionPlanner::terminate)
        .def("clear", &tp::TrajOptMotionPlanner::clear);
}
