/**
 * @file tesseract_motion_planners_bindings.cpp
 * @brief nanobind bindings for tesseract_motion_planners core
 */

#include "tesseract_nb.h"
#include <nanobind/stl/unordered_map.h>

// tesseract_motion_planners core
#include <tesseract_motion_planners/core/planner.h>
#include <tesseract_motion_planners/core/types.h>
#include <tesseract_motion_planners/core/utils.h>

// tesseract_environment
#include <tesseract_environment/environment.h>

// tesseract_command_language
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/profile_dictionary.h>

namespace tp = tesseract_planning;
namespace te = tesseract_environment;

NB_MODULE(_tesseract_motion_planners, m) {
    m.doc() = "tesseract_motion_planners Python bindings";

    // ========== PlannerRequest ==========
    nb::class_<tp::PlannerRequest>(m, "PlannerRequest")
        .def(nb::init<>())
        .def_rw("name", &tp::PlannerRequest::name)
        .def_prop_rw("env",
            [](const tp::PlannerRequest& self) { return self.env; },
            [](tp::PlannerRequest& self, std::shared_ptr<const te::Environment> env) { self.env = env; })
        .def_prop_rw("profiles",
            [](const tp::PlannerRequest& self) { return self.profiles; },
            [](tp::PlannerRequest& self, std::shared_ptr<const tp::ProfileDictionary> profiles) { self.profiles = profiles; })
        .def_rw("instructions", &tp::PlannerRequest::instructions)
        .def_rw("verbose", &tp::PlannerRequest::verbose)
        .def_rw("format_result_as_input", &tp::PlannerRequest::format_result_as_input);

    // ========== PlannerResponse ==========
    nb::class_<tp::PlannerResponse>(m, "PlannerResponse")
        .def(nb::init<>())
        .def_rw("results", &tp::PlannerResponse::results)
        .def_rw("successful", &tp::PlannerResponse::successful)
        .def_rw("message", &tp::PlannerResponse::message)
        .def("__bool__", [](const tp::PlannerResponse& self) { return static_cast<bool>(self); });

    // ========== MotionPlanner (abstract base) ==========
    nb::class_<tp::MotionPlanner>(m, "MotionPlanner")
        .def("getName", &tp::MotionPlanner::getName)
        .def("solve", &tp::MotionPlanner::solve, "request"_a, nb::call_guard<nb::gil_scoped_release>())
        .def("terminate", &tp::MotionPlanner::terminate)
        .def("clear", &tp::MotionPlanner::clear)
        .def_static("checkRequest", nb::overload_cast<const tp::PlannerRequest&>(&tp::MotionPlanner::checkRequest),
                    "request"_a);

    // ========== Utility functions ==========
    m.def("assignCurrentStateAsSeed", &tp::assignCurrentStateAsSeed,
          "composite_instructions"_a, "env"_a,
          "Assign the current environment state as seed to all CartesianWaypoints in the program");
}
