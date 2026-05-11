/**
 * @file tesseract_motion_planners_simple_python.i
 * @brief The tesseract_motion_planners_simple_python SWIG master file.
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

%module(directors="1", package="tesseract_robotics.tesseract_motion_planners_simple") tesseract_motion_planners_simple_python

#pragma SWIG nowarn=473

%include "tesseract_swig_include.i"
%include "tesseract_std_function.i"

%import "tesseract_motion_planners_python.i"

%{
// tesseract_motion_planners_simple
#include <tesseract/motion_planners/simple/profile/simple_planner_profile.h>
#include <tesseract/motion_planners/simple/profile/simple_planner_lvs_move_profile.h>
#include <tesseract/motion_planners/simple/profile/simple_planner_lvs_no_ik_move_profile.h>
#include <tesseract/motion_planners/simple/profile/simple_planner_fixed_size_move_profile.h>
#include <tesseract/motion_planners/simple/profile/simple_planner_fixed_size_assign_move_profile.h>
#include <tesseract/motion_planners/simple/simple_motion_planner.h>
#include <tesseract/motion_planners/simple/interpolation.h>


#include <tesseract/geometry/geometries.h>
#include <tesseract/common/resource_locator.h>
#include <tesseract/srdf/kinematics_information.h>

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

// tesseract_command_language
#include <tesseract/command_language/fwd.h>
#include <tesseract/command_language/move_instruction.h>
#include <tesseract/command_language/composite_instruction.h>
#include <tesseract/common/profile_dictionary.h>

// tesseract_motion_planners
#include <tesseract/motion_planners/planner.h>
#include <tesseract/motion_planners/types.h>

#include "tesseract_environment_python_std_functions.h"

%}

// tesseract_motion_planners_simple
#define TESSERACT_MOTION_PLANNERS_SIMPLE_PUBLIC

%ignore tesseract::motion_planners::JointGroupInstructionInfo::getWorkingFrame;
%include "tesseract/motion_planners/simple/interpolation.h"

%pythondynamic tesseract::motion_planners::SimplePlannerMoveProfile;
%pythondynamic tesseract::motion_planners::SimplePlannerCompositeProfile;
%shared_ptr(tesseract::motion_planners::SimplePlannerMoveProfile)
%shared_ptr(tesseract::motion_planners::SimplePlannerCompositeProfile)
%include "tesseract/motion_planners/simple/profile/simple_planner_profile.h"
%tesseract_command_language_add_profile_type(SimplePlannerMoveProfile);
%tesseract_command_language_add_profile_type(SimplePlannerCompositeProfile);

%pythondynamic tesseract::motion_planners::SimplePlannerLVSMoveProfile;
%shared_ptr(tesseract::motion_planners::SimplePlannerLVSMoveProfile)
%include "tesseract/motion_planners/simple/profile/simple_planner_lvs_move_profile.h"

%pythondynamic tesseract::motion_planners::SimplePlannerLVSNoIKMoveProfile;
%shared_ptr(tesseract::motion_planners::SimplePlannerLVSNoIKMoveProfile)
%include "tesseract/motion_planners/simple/profile/simple_planner_lvs_no_ik_move_profile.h"
%pythondynamic tesseract::motion_planners::SimplePlannerFixedSizeMoveProfile;
%shared_ptr(tesseract::motion_planners::SimplePlannerFixedSizeMoveProfile)
%include "tesseract/motion_planners/simple/profile/simple_planner_fixed_size_move_profile.h"

%pythondynamic tesseract::motion_planners::SimplePlannerFixedSizeAssignMoveProfile;
%shared_ptr(tesseract::motion_planners::SimplePlannerFixedSizeAssignMoveProfile)
%include "tesseract/motion_planners/simple/profile/simple_planner_fixed_size_assign_move_profile.h"
%pythondynamic tesseract::motion_planners::SimpleMotionPlanner;
%shared_ptr(tesseract::motion_planners::SimpleMotionPlanner)
%ignore tesseract::motion_planners::SimpleMotionPlanner::clone;
%include "tesseract/motion_planners/simple/simple_motion_planner.h"
