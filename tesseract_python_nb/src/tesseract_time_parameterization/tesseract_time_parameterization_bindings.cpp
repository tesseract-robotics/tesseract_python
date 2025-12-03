/**
 * @file tesseract_time_parameterization_bindings.cpp
 * @brief nanobind bindings for tesseract_time_parameterization
 */

#include "tesseract_nb.h"

// tesseract_time_parameterization
#include <tesseract_time_parameterization/core/time_parameterization.h>
#include <tesseract_time_parameterization/core/instructions_trajectory.h>
#include <tesseract_time_parameterization/totg/time_optimal_trajectory_generation.h>

// tesseract_command_language
#include <tesseract_command_language/composite_instruction.h>

namespace tp = tesseract_planning;

NB_MODULE(_tesseract_time_parameterization, m) {
    m.doc() = "tesseract_time_parameterization Python bindings";

    // ========== TrajectoryContainer (base) ==========
    nb::class_<tp::TrajectoryContainer>(m, "TrajectoryContainer")
        .def("size", &tp::TrajectoryContainer::size)
        .def("dof", &tp::TrajectoryContainer::dof)
        .def("empty", &tp::TrajectoryContainer::empty)
        .def("getTimeFromStart", &tp::TrajectoryContainer::getTimeFromStart, "i"_a);

    // ========== InstructionsTrajectory ==========
    nb::class_<tp::InstructionsTrajectory, tp::TrajectoryContainer>(m, "InstructionsTrajectory")
        .def(nb::init<tp::CompositeInstruction&>(), "program"_a);

    // ========== TimeParameterization (base) ==========
    nb::class_<tp::TimeParameterization>(m, "TimeParameterization")
        .def("compute", &tp::TimeParameterization::compute,
             "trajectory"_a,
             "velocity_limits"_a,
             "acceleration_limits"_a,
             "jerk_limits"_a,
             "velocity_scaling_factors"_a = Eigen::VectorXd::Ones(1),
             "acceleration_scaling_factors"_a = Eigen::VectorXd::Ones(1),
             "jerk_scaling_factors"_a = Eigen::VectorXd::Ones(1));

    // ========== TimeOptimalTrajectoryGeneration ==========
    nb::class_<tp::TimeOptimalTrajectoryGeneration, tp::TimeParameterization>(m, "TimeOptimalTrajectoryGeneration")
        .def(nb::init<double, double>(),
             "path_tolerance"_a = 0.1,
             "min_angle_change"_a = 0.001);
}
