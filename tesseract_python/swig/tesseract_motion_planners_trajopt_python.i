/**
 * @file tesseract_motion_planners_trajopt_python.i
 * @brief The tesseract_motion_planners_trajopt_python SWIG master file.
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

%module(directors="1", package="tesseract_robotics.tesseract_motion_planners_trajopt") tesseract_motion_planners_trajopt_python

#pragma SWIG nowarn=473

%include "tesseract_swig_include.i"
%include "tesseract_std_function.i"

%import "tesseract_motion_planners_python.i"

%{
// trajopt
#include <trajopt/problem_description.hpp>

// tesseract_motion_planners_trajopt
#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>
#include <tesseract_motion_planners/trajopt/trajopt_collision_config.h>
#include <tesseract_motion_planners/trajopt/trajopt_utils.h>
#include <tesseract_motion_planners/trajopt/trajopt_waypoint_config.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h>
#include <tesseract_motion_planners/trajopt/trajopt_collision_config.h>


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

// tesseract_visualization
#include <tesseract_visualization/visualization.h>

#include "tesseract_command_language_python_std_functions.h"

#include "tesseract_environment_python_std_functions.h"

%}

// trajopt

%ignore createSolverConfig;
%ignore createOptimizationParameters;
%ignore createOptimizationCallbacks;

// Including trajopt headers is too noisy, use *.i file instead
%include "trajopt/problem_description.i"

// %tesseract_std_function_base(TrajOptProblemGeneratorFn,tesseract_planning,std::shared_ptr<trajopt::ProblemConstructionInfo>,const std::string&,a,const tesseract_planning::PlannerRequest&,b,const tesseract_planning::TrajOptPlanProfileMap&,c,const tesseract_planning::TrajOptCompositeProfileMap&,d,const tesseract_planning::TrajOptSolverProfileMap&,e);
// %tesseract_std_function(TrajOptProblemGeneratorFn,tesseract_planning,std::shared_ptr<trajopt::ProblemConstructionInfo>,const std::string&,a,const tesseract_planning::PlannerRequest&,b,const tesseract_planning::TrajOptPlanProfileMap&,c,const tesseract_planning::TrajOptCompositeProfileMap&,d,const tesseract_planning::TrajOptSolverProfileMap&,e);

// tesseract_motion_planners_trajopt
#define TESSERACT_MOTION_PLANNERS_TRAJOPT_PUBLIC

%include "tesseract_motion_planners/trajopt/trajopt_collision_config.h"
%include "tesseract_motion_planners/trajopt/trajopt_waypoint_config.h"

%pythondynamic tesseract_planning::TrajOptPlanProfile;
%pythondynamic tesseract_planning::TrajOptCompositeProfile;
%pythondynamic tesseract_planning::TrajOptSolverProfile;
%shared_ptr(tesseract_planning::TrajOptPlanProfile)
%shared_ptr(tesseract_planning::TrajOptSolverProfile)
%shared_ptr(tesseract_planning::TrajOptCompositeProfile)
%include "tesseract_motion_planners/trajopt/profile/trajopt_profile.h"
// %template(TrajOptSolverProfileMap) std::unordered_map<std::string, std::shared_ptr<const tesseract_planning::TrajOptSolverProfile>>;
// %template(TrajOptCompositeProfileMap) std::unordered_map<std::string, std::shared_ptr<const tesseract_planning::TrajOptCompositeProfile>>;
// %template(TrajOptPlanProfileMap) std::unordered_map<std::string, std::shared_ptr<const tesseract_planning::TrajOptPlanProfile>>;
%tesseract_command_language_add_profile_type(TrajOptSolverProfile);
%tesseract_command_language_add_profile_type(TrajOptPlanProfile);
%tesseract_command_language_add_profile_type(TrajOptCompositeProfile);

%pythondynamic tesseract_planning::TrajOptDefaultPlanProfile;
%shared_ptr(tesseract_planning::TrajOptDefaultPlanProfile)
%include "tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h"

%pythondynamic tesseract_planning::TrajOptDefaultCompositeProfile;
%shared_ptr(tesseract_planning::TrajOptDefaultCompositeProfile)
%include "tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h"

%include "tesseract_motion_planners/trajopt/trajopt_utils.h"

%include "tesseract_motion_planners/trajopt/trajopt_collision_config.h"

%pythondynamic tesseract_planning::TrajOptMotionPlanner;
%shared_ptr(tesseract_planning::TrajOptMotionPlanner)
%ignore tesseract_planning::TrajOptMotionPlanner::clone;
%include "tesseract_motion_planners/trajopt/trajopt_motion_planner.h"

// TODO
// %template(TrajOptIfoptCompositeProfileMap) std::unordered_map<std::string, std::shared_ptr<const tesseract_planning::TrajOptIfoptCompositeProfile>>;
// %template(TrajOptIfoptPlanProfileMap) std::unordered_map<std::string, std::shared_ptr<const tesseract_planning::TrajOptIfoptPlanProfile>>;

