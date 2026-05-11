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
#include <tesseract/motion_planners/trajopt/trajopt_motion_planner.h>
#include <tesseract/motion_planners/trajopt/trajopt_utils.h>
#include <tesseract/motion_planners/trajopt/trajopt_waypoint_config.h>
#include <tesseract/motion_planners/trajopt/profile/trajopt_profile.h>
#include <tesseract/motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract/motion_planners/trajopt/profile/trajopt_default_move_profile.h>


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

// tesseract_visualization
#include <tesseract/visualization/visualization.h>

#include "tesseract_command_language_python_std_functions.h"

#include "tesseract_environment_python_std_functions.h"

%}

// trajopt

%ignore createSolverConfig;
%ignore createOptimizationParameters;
%ignore createOptimizationCallbacks;

// Including trajopt headers is too noisy, use *.i file instead
%include "trajopt/problem_description.i"

// %tesseract_std_function_base(TrajOptProblemGeneratorFn,tesseract_planning,std::shared_ptr<trajopt::ProblemConstructionInfo>,const std::string&,a,const tesseract::motion_planners::PlannerRequest&,b,const tesseract::motion_planners::TrajOptMoveProfileMap&,c,const tesseract::motion_planners::TrajOptCompositeProfileMap&,d,const tesseract::motion_planners::TrajOptSolverProfileMap&,e);
// %tesseract_std_function(TrajOptProblemGeneratorFn,tesseract_planning,std::shared_ptr<trajopt::ProblemConstructionInfo>,const std::string&,a,const tesseract::motion_planners::PlannerRequest&,b,const tesseract::motion_planners::TrajOptMoveProfileMap&,c,const tesseract::motion_planners::TrajOptCompositeProfileMap&,d,const tesseract::motion_planners::TrajOptSolverProfileMap&,e);

// tesseract_motion_planners_trajopt
#define TESSERACT_MOTION_PLANNERS_TRAJOPT_PUBLIC

%include "tesseract/motion_planners/trajopt/trajopt_waypoint_config.h"

%pythondynamic tesseract::motion_planners::TrajOptMoveProfile;
%pythondynamic tesseract::motion_planners::TrajOptCompositeProfile;
%pythondynamic tesseract::motion_planners::TrajOptSolverProfile;
%shared_ptr(tesseract::motion_planners::TrajOptMoveProfile)
%shared_ptr(tesseract::motion_planners::TrajOptSolverProfile)
%shared_ptr(tesseract::motion_planners::TrajOptCompositeProfile)
%include "tesseract/motion_planners/trajopt/profile/trajopt_profile.h"
// %template(TrajOptSolverProfileMap) std::unordered_map<std::string, std::shared_ptr<const tesseract::motion_planners::TrajOptSolverProfile>>;
// %template(TrajOptCompositeProfileMap) std::unordered_map<std::string, std::shared_ptr<const tesseract::motion_planners::TrajOptCompositeProfile>>;
// %template(TrajOptMoveProfileMap) std::unordered_map<std::string, std::shared_ptr<const tesseract::motion_planners::TrajOptMoveProfile>>;
%tesseract_command_language_add_profile_type(TrajOptSolverProfile);
%tesseract_command_language_add_profile_type(TrajOptMoveProfile);
%tesseract_command_language_add_profile_type(TrajOptCompositeProfile);

%pythondynamic tesseract::motion_planners::TrajOptDefaultMoveProfile;
%shared_ptr(tesseract::motion_planners::TrajOptDefaultMoveProfile)
%include "tesseract/motion_planners/trajopt/profile/trajopt_default_move_profile.h"

%pythondynamic tesseract::motion_planners::TrajOptDefaultCompositeProfile;
%shared_ptr(tesseract::motion_planners::TrajOptDefaultCompositeProfile)
%include "tesseract/motion_planners/trajopt/profile/trajopt_default_composite_profile.h"

%include "tesseract/motion_planners/trajopt/trajopt_utils.h"

%pythondynamic tesseract::motion_planners::TrajOptMotionPlanner;
%shared_ptr(tesseract::motion_planners::TrajOptMotionPlanner)
%ignore tesseract::motion_planners::TrajOptMotionPlanner::clone;
%include "tesseract/motion_planners/trajopt/trajopt_motion_planner.h"

// TODO
// %template(TrajOptIfoptCompositeProfileMap) std::unordered_map<std::string, std::shared_ptr<const tesseract::motion_planners::TrajOptIfoptCompositeProfile>>;
// %template(TrajOptIfoptMoveProfileMap) std::unordered_map<std::string, std::shared_ptr<const tesseract::motion_planners::TrajOptIfoptMoveProfile>>;

