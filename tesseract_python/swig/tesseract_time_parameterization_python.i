/**
 * @file tesseract_time_parameterization_python.i
 * @brief The tesseract_time_parameterization_python SWIG master file.
 *
 * @author John Wason
 * @date December 8, 2020
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

%module(directors="1", package="tesseract_robotics.tesseract_time_parameterization") tesseract_time_parameterization_python

#pragma SWIG nowarn=473

%include "tesseract_swig_include.i"

%import "tesseract_command_language_python.i"
%import "tesseract_environment_python.i"

%{


#include <tesseract/common/resource_locator.h>

// tesseract_time_parameterization
#include <tesseract/time_parameterization/isp/iterative_spline_parameterization.h>
#include <tesseract/time_parameterization/isp/iterative_spline_parameterization_profiles.h>
#include <tesseract/time_parameterization/totg/time_optimal_trajectory_generation.h>
#include <tesseract/time_parameterization/totg/time_optimal_trajectory_generation_profiles.h>
#include <tesseract/time_parameterization/ruckig/ruckig_trajectory_smoothing.h>
#include <tesseract/time_parameterization/ruckig/ruckig_trajectory_smoothing_profiles.h>
#include <tesseract/time_parameterization/instructions_trajectory.h>
#include <tesseract/time_parameterization/kdl/constant_tcp_speed_parameterization.h>
#include <tesseract/time_parameterization/kdl/constant_tcp_speed_parameterization_profiles.h>

// tesseract_state_solver
#include <tesseract/state_solver/mutable_state_solver.h>
#include <tesseract/state_solver/state_solver.h>
#include <tesseract/state_solver/kdl/kdl_state_solver.h>
#include <tesseract/state_solver/ofkt/ofkt_state_solver.h>

// tesseract_kinematics
#include <tesseract/kinematics/joint_group.h>
#include <tesseract/kinematics/kinematic_group.h>

// tesseract_environment
#include <tesseract/environment/commands.h>
#include <tesseract/environment/events.h>
#include <tesseract/environment/environment.h>

#include <tesseract/geometry/geometries.h>
#include <tesseract/common/resource_locator.h>
#include <tesseract/srdf/kinematics_information.h>
%}

%ignore tesseract::time_parameterization::totg;

// tesseract_time_parameterization
#define TESSERACT_TIME_PARAMETERIZATION_PUBLIC

%shared_ptr(tesseract::time_parameterization::InstructionsTrajectory)
%include "tesseract/time_parameterization/instructions_trajectory.h"

// tesseract_time_parameterization_isp
%shared_ptr(tesseract::time_parameterization::IterativeSplineParameterization)
%pythondynamic tesseract::time_parameterization::IterativeSplineParameterizationCompositeProfile;
%pythondynamic tesseract::time_parameterization::IterativeSplineParameterizationMoveProfile;
%shared_ptr(tesseract::time_parameterization::IterativeSplineParameterizationCompositeProfile)
%shared_ptr(tesseract::time_parameterization::IterativeSplineParameterizationMoveProfile)
%include "tesseract/time_parameterization/isp/iterative_spline_parameterization.h"
%include "tesseract/time_parameterization/isp/iterative_spline_parameterization_profiles.h"

// tesseract_time_parameterization_totg
%shared_ptr(tesseract::time_parameterization::TimeOptimalTrajectoryGeneration)
%pythondynamic tesseract::time_parameterization::TimeOptimalTrajectoryGenerationCompositeProfile;
%shared_ptr(tesseract::time_parameterization::TimeOptimalTrajectoryGenerationCompositeProfile)
%include "tesseract/time_parameterization/totg/time_optimal_trajectory_generation.h"
%include "tesseract/time_parameterization/totg/time_optimal_trajectory_generation_profiles.h"

// tesseract_time_parameterization_ruckig
%shared_ptr(tesseract::time_parameterization::RuckigTrajectorySmoothing)
%pythondynamic tesseract::time_parameterization::RuckigTrajectorySmoothingCompositeProfile;
%shared_ptr(tesseract::time_parameterization::RuckigTrajectorySmoothingCompositeProfile)
%include "tesseract/time_parameterization/ruckig/ruckig_trajectory_smoothing.h"
%include "tesseract/time_parameterization/ruckig/ruckig_trajectory_smoothing_profiles.h" 

// tesseract_time_parameterization_kdl
// %shared_ptr(tesseract::time_parameterization::ConstantTCPSpeedParameterization)
// %shared_ptr(tesseract::time_parameterization::ConstantTCPSpeedParameterizationCompositeProfile)
// %include "tesseract/time_parameterization/kdl/constant_tcp_speed_parameterization.h"
// %include "tesseract/time_parameterization/kdl/constant_tcp_speed_parameterization_profiles.h"

