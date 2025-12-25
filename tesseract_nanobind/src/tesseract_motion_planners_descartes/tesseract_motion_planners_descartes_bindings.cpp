/**
 * @file tesseract_motion_planners_descartes_bindings.cpp
 * @brief nanobind bindings for tesseract_motion_planners Descartes
 */

#include "tesseract_nb.h"

// tesseract_motion_planners core (for PlannerRequest/Response)
#include <tesseract_motion_planners/core/types.h>

// tesseract_command_language (for Profile base class)
#include <tesseract_command_language/profile.h>
#include <tesseract_command_language/profile_dictionary.h>

// tesseract_motion_planners Descartes
#include <tesseract_motion_planners/descartes/descartes_motion_planner.h>
#include <tesseract_motion_planners/descartes/profile/descartes_profile.h>
#include <tesseract_motion_planners/descartes/profile/descartes_default_plan_profile.h>
#include <tesseract_motion_planners/descartes/profile/descartes_ladder_graph_solver_profile.h>

// tesseract_collision for CollisionCheckConfig
#include <tesseract_collision/core/types.h>

namespace tp = tesseract_planning;

NB_MODULE(_tesseract_motion_planners_descartes, m) {
    m.doc() = "tesseract_motion_planners_descartes Python bindings";

    // Import Profile type from tesseract_command_language for cross-module inheritance
    nb::module_::import_("tesseract_robotics.tesseract_command_language._tesseract_command_language");

    // Import tesseract_collision for CollisionCheckConfig
    nb::module_::import_("tesseract_robotics.tesseract_collision._tesseract_collision");

    // ========== DescartesSolverProfile<double> (base for solver profiles) ==========
    nb::class_<tp::DescartesSolverProfile<double>, tp::Profile>(m, "DescartesSolverProfileD")
        .def("getKey", &tp::DescartesSolverProfile<double>::getKey)
        .def_static("getStaticKey", &tp::DescartesSolverProfile<double>::getStaticKey);

    // ========== DescartesLadderGraphSolverProfile<double> ==========
    nb::class_<tp::DescartesLadderGraphSolverProfile<double>, tp::DescartesSolverProfile<double>>(m, "DescartesLadderGraphSolverProfileD")
        .def(nb::init<>())
        .def_rw("num_threads", &tp::DescartesLadderGraphSolverProfile<double>::num_threads,
            "Number of threads to use during planning (default: 1)");

    // Helper to cast DescartesLadderGraphSolverProfileD to Profile
    m.def("cast_DescartesSolverProfileD", [](std::shared_ptr<tp::DescartesLadderGraphSolverProfile<double>> profile) {
        return std::static_pointer_cast<tp::Profile>(profile);
    }, "profile"_a,
    "Cast DescartesLadderGraphSolverProfileD to Profile for use with ProfileDictionary");

    // ========== DescartesPlanProfile<double> (base) ==========
    nb::class_<tp::DescartesPlanProfile<double>, tp::Profile>(m, "DescartesPlanProfileD")
        .def("getKey", &tp::DescartesPlanProfile<double>::getKey)
        .def_static("getStaticKey", &tp::DescartesPlanProfile<double>::getStaticKey);

    // ========== DescartesDefaultPlanProfile<double> ==========
    nb::class_<tp::DescartesDefaultPlanProfile<double>, tp::DescartesPlanProfile<double>>(m, "DescartesDefaultPlanProfileD")
        .def(nb::init<>())
        .def_rw("target_pose_fixed", &tp::DescartesDefaultPlanProfile<double>::target_pose_fixed)
        .def_rw("target_pose_sample_axis", &tp::DescartesDefaultPlanProfile<double>::target_pose_sample_axis)
        .def_rw("target_pose_sample_resolution", &tp::DescartesDefaultPlanProfile<double>::target_pose_sample_resolution)
        .def_rw("target_pose_sample_min", &tp::DescartesDefaultPlanProfile<double>::target_pose_sample_min)
        .def_rw("target_pose_sample_max", &tp::DescartesDefaultPlanProfile<double>::target_pose_sample_max)
        .def_rw("manipulator_ik_solver", &tp::DescartesDefaultPlanProfile<double>::manipulator_ik_solver)
        .def_rw("allow_collision", &tp::DescartesDefaultPlanProfile<double>::allow_collision)
        .def_rw("enable_collision", &tp::DescartesDefaultPlanProfile<double>::enable_collision)
        .def_rw("vertex_collision_check_config", &tp::DescartesDefaultPlanProfile<double>::vertex_collision_check_config)
        .def_rw("enable_edge_collision", &tp::DescartesDefaultPlanProfile<double>::enable_edge_collision)
        .def_rw("edge_collision_check_config", &tp::DescartesDefaultPlanProfile<double>::edge_collision_check_config)
        .def_rw("use_redundant_joint_solutions", &tp::DescartesDefaultPlanProfile<double>::use_redundant_joint_solutions)
        .def_rw("debug", &tp::DescartesDefaultPlanProfile<double>::debug);

    // Helper to add Descartes plan profile to ProfileDictionary
    // This casts DescartesDefaultPlanProfileD to the base Profile type
    m.def("cast_DescartesPlanProfileD", [](std::shared_ptr<tp::DescartesDefaultPlanProfile<double>> profile) {
        return std::static_pointer_cast<tp::Profile>(profile);
    }, "profile"_a,
    "Cast DescartesDefaultPlanProfileD to Profile for use with ProfileDictionary");

    // ========== DescartesMotionPlanner<double> ==========
    nb::class_<tp::DescartesMotionPlanner<double>>(m, "DescartesMotionPlannerD")
        .def(nb::init<std::string>(), "name"_a)
        .def("getName", &tp::DescartesMotionPlanner<double>::getName)
        .def("solve", &tp::DescartesMotionPlanner<double>::solve, "request"_a, nb::call_guard<nb::gil_scoped_release>())
        .def("terminate", &tp::DescartesMotionPlanner<double>::terminate)
        .def("clear", &tp::DescartesMotionPlanner<double>::clear);
}
