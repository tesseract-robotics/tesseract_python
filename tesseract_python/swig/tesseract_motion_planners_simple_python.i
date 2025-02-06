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
#include <tesseract_motion_planners/simple/profile/simple_planner_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_lvs_plan_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_lvs_no_ik_plan_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_fixed_size_plan_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_fixed_size_assign_plan_profile.h>
#include <tesseract_motion_planners/simple/simple_motion_planner.h>
#include <tesseract_motion_planners/simple/interpolation.h>


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
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/profile_dictionary.h>

// tesseract_motion_planners
#include <tesseract_motion_planners/core/planner.h>
#include <tesseract_motion_planners/core/types.h>

#include "tesseract_environment_python_std_functions.h"

%}

// tesseract_motion_planners_simple
#define TESSERACT_MOTION_PLANNERS_SIMPLE_PUBLIC

%ignore tesseract_planning::JointGroupInstructionInfo::getWorkingFrame;
%include "tesseract_motion_planners/simple/interpolation.h"

%pythondynamic tesseract_planning::SimplePlannerPlanProfile;
%pythondynamic tesseract_planning::SimplePlannerCompositeProfile;
%shared_ptr(tesseract_planning::SimplePlannerPlanProfile)
%shared_ptr(tesseract_planning::SimplePlannerCompositeProfile)
%include "tesseract_motion_planners/simple/profile/simple_planner_profile.h"
%template(SimplePlannerPlanProfileMap) std::unordered_map<std::string, tesseract_planning::SimplePlannerPlanProfile::ConstPtr>;
%template(SimplePlannerCompositeProfileMap) std::unordered_map<std::string, tesseract_planning::SimplePlannerCompositeProfile::ConstPtr>;
%tesseract_command_language_add_profile_type(SimplePlannerPlanProfile);
%tesseract_command_language_add_profile_type(SimplePlannerCompositeProfile);

%pythondynamic tesseract_planning::SimplePlannerLVSPlanProfile;
%shared_ptr(tesseract_planning::SimplePlannerLVSPlanProfile)
%include "tesseract_motion_planners/simple/profile/simple_planner_lvs_plan_profile.h"

%pythondynamic tesseract_planning::SimplePlannerLVSNoIKPlanProfile;
%shared_ptr(tesseract_planning::SimplePlannerLVSNoIKPlanProfile)
%include "tesseract_motion_planners/simple/profile/simple_planner_lvs_no_ik_plan_profile.h"

%pythondynamic tesseract_planning::SimplePlannerFixedSizePlanProfile;
%shared_ptr(tesseract_planning::SimplePlannerFixedSizePlanProfile)
%include "tesseract_motion_planners/simple/profile/simple_planner_fixed_size_plan_profile.h"

%pythondynamic tesseract_planning::SimplePlannerFixedSizeAssignPlanProfile;
%shared_ptr(tesseract_planning::SimplePlannerFixedSizeAssignPlanProfile)
%include "tesseract_motion_planners/simple/profile/simple_planner_fixed_size_assign_plan_profile.h"

%pythondynamic tesseract_planning::SimpleMotionPlanner;
%shared_ptr(tesseract_planning::SimpleMotionPlanner)
%ignore tesseract_planning::SimpleMotionPlanner::clone;
%include "tesseract_motion_planners/simple/simple_motion_planner.h"
