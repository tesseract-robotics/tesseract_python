/**
 * @file tesseract_motion_planners_python.i
 * @brief The tesseract_motion_planners_python SWIG master file.
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

%module(directors="1", package="tesseract_robotics.tesseract_motion_planners") tesseract_motion_planners_python

#pragma SWIG nowarn=473

%include "tesseract_swig_include.i"

%import "tesseract_environment_python.i"
%import "tesseract_command_language_python.i"

%{
// tesseract_motion_planners
#include <tesseract_motion_planners/core/planner.h>
#include <tesseract_motion_planners/robot_config.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_motion_planners/core/types.h>

#include <tesseract_geometry/geometries.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_srdf/kinematics_information.h>

// tesseract_state_solver
#include <tesseract_state_solver/mutable_state_solver.h>
#include <tesseract_state_solver/state_solver.h>
#include <tesseract_state_solver/kdl/kdl_state_solver.h>
#include <tesseract_state_solver/ofkt/ofkt_state_solver.h>

// tesseract_kinematics
#include <tesseract_kinematics/core/joint_group.h>
#include <tesseract_kinematics/core/kinematic_group.h>

// tesseract_environment
#include <tesseract_environment/commands.h>
#include <tesseract_environment/events.h>
#include <tesseract_environment/environment.h>

// tesseract_command_language
#include <tesseract_command_language/fwd.h>
#include <tesseract_command_language/move_instruction.h>

#include "tesseract_environment_python_std_functions.h"

%}

%unique_ptr_value_wrapper(tesseract_kinematics::KinematicGroup);
%unique_ptr_value_wrapper(tesseract_kinematics::JointGroup);

// tesseract_motion_planners
#define TESSERACT_MOTION_PLANNERS_CORE_PUBLIC

%include "tesseract_motion_planners/core/types.h"

%shared_ptr(tesseract_planning::MotionPlanner)
%wrap_unique_ptr(MotionPlannerUPtr,tesseract_planning::MotionPlanner)
%include "tesseract_motion_planners/core/planner.h"

%include "tesseract_motion_planners/core/utils.h"

%include "tesseract_motion_planners/robot_config.h"
%template(getRobotConfig) tesseract_planning::getRobotConfig<double>;
%template(getJointTurns) tesseract_planning::getJointTurns<double>;
