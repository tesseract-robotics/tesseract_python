/**
 * @file tesseract_motion_planners_simple_bindings.cpp
 * @brief nanobind bindings for tesseract_motion_planners simple
 */

#include "tesseract_nb.h"

// tesseract_motion_planners core (for PlannerRequest/Response)
#include <tesseract_motion_planners/core/types.h>

// tesseract_motion_planners simple
#include <tesseract_motion_planners/simple/interpolation.h>
#include <tesseract_motion_planners/simple/simple_motion_planner.h>

// tesseract_environment
#include <tesseract_environment/environment.h>

// tesseract_command_language
#include <tesseract_command_language/composite_instruction.h>

namespace tp = tesseract_planning;
namespace te = tesseract_environment;

NB_MODULE(_tesseract_motion_planners_simple, m) {
    m.doc() = "tesseract_motion_planners_simple Python bindings";

    // ========== SimpleMotionPlanner ==========
    // Note: Not binding inheritance from MotionPlanner to avoid cross-module issues
    nb::class_<tp::SimpleMotionPlanner>(m, "SimpleMotionPlanner")
        .def(nb::init<std::string>(), "name"_a = "SIMPLE")
        .def("getName", &tp::SimpleMotionPlanner::getName)
        .def("solve", &tp::SimpleMotionPlanner::solve, "request"_a, nb::call_guard<nb::gil_scoped_release>())
        .def("terminate", &tp::SimpleMotionPlanner::terminate)
        .def("clear", &tp::SimpleMotionPlanner::clear);

    // ========== generateInterpolatedProgram ==========
    m.def("generateInterpolatedProgram",
          [](const tp::CompositeInstruction& instructions,
             const std::shared_ptr<const te::Environment>& env,
             double state_lvs,
             double translation_lvs,
             double rotation_lvs,
             int min_steps) {
              return tp::generateInterpolatedProgram(instructions, env, state_lvs, translation_lvs, rotation_lvs, min_steps);
          },
          "instructions"_a,
          "env"_a,
          "state_longest_valid_segment_length"_a = 5 * M_PI / 180,
          "translation_longest_valid_segment_length"_a = 0.15,
          "rotation_longest_valid_segment_length"_a = 5 * M_PI / 180,
          "min_steps"_a = 1,
          "Generate an interpolated program from a composite instruction");
}
