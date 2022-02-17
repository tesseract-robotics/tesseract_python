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

%module(directors="1", package="tesseract_robotics.tesseract_process_managers") tesseract_process_managers_python

#pragma SWIG nowarn=473

%include "tesseract_swig_include.i"

%import "tesseract_motion_planners_simple_python.i"
%import "tesseract_motion_planners_trajopt_python.i"
%import "tesseract_motion_planners_ompl_python.i"
%import "tesseract_motion_planners_descartes_python.i"
%import "tesseract_time_parameterization_python.i"

%{

// tesseract_motion_planners_simple
#include <tesseract_motion_planners/simple/profile/simple_planner_utils.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_lvs_plan_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_lvs_no_ik_plan_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_fixed_size_plan_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_fixed_size_assign_plan_profile.h>
#include <tesseract_motion_planners/simple/simple_motion_planner.h>

// tesseract_motion_planners_trajopt
#include <tesseract_motion_planners/trajopt/trajopt_motion_planner.h>
#include <tesseract_motion_planners/trajopt/trajopt_collision_config.h>
#include <tesseract_motion_planners/trajopt/trajopt_utils.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_plan_profile.h>
#include <tesseract_motion_planners/trajopt/profile/trajopt_default_solver_profile.h>
#include <tesseract_motion_planners/trajopt/serialize.h>
#include <tesseract_motion_planners/trajopt/deserialize.h>

// tesseract_motion_planners_ompl
#include <tesseract_motion_planners/ompl/ompl_planner_configurator.h>
#include <tesseract_motion_planners/ompl/ompl_problem.h>
#include <tesseract_motion_planners/ompl/ompl_motion_planner_status_category.h>
#include <tesseract_motion_planners/ompl/profile/ompl_profile.h>
#include <tesseract_motion_planners/ompl/profile/ompl_default_plan_profile.h>
#include <tesseract_motion_planners/ompl/ompl_motion_planner.h>
#include <tesseract_motion_planners/ompl/serialize.h>
#include <tesseract_motion_planners/ompl/deserialize.h>

// tesseract_motion_planner_descartes
#include <tesseract_motion_planners/descartes/descartes_problem.h>
#include <tesseract_motion_planners/descartes/descartes_motion_planner_status_category.h>
#include <tesseract_motion_planners/descartes/profile/descartes_profile.h>
#include <tesseract_motion_planners/descartes/profile/descartes_default_plan_profile.h>
#include <tesseract_motion_planners/descartes/descartes_motion_planner.h>
#include <tesseract_motion_planners/descartes/serialize.h>
#include <tesseract_motion_planners/descartes/deserialize.h>

// tesseract_process_managers

#include <tesseract_process_managers/core/task_info.h>
#include <tesseract_process_managers/core/taskflow_interface.h>
#include <tesseract_process_managers/core/process_planning_request.h>
#include <tesseract_process_managers/core/process_planning_future.h>
#include <tesseract_process_managers/core/process_planning_server.h>

#include <tesseract_process_managers/task_generators/profile_switch_task_generator.h>
#include <tesseract_process_managers/task_generators/iterative_spline_parameterization_task_generator.h>
#include "tesseract_process_managers/task_generators/time_optimal_parameterization_task_generator.h"

#include <tesseract_common/status_code.h>
#include <tesseract_common/resource_locator.h>

// tesseract_state_solver
#include <tesseract_state_solver/mutable_state_solver.h>
#include <tesseract_state_solver/state_solver.h>
#include <tesseract_state_solver/kdl/kdl_state_solver.h>
#include <tesseract_state_solver/ofkt/ofkt_state_solver.h>

#include <tesseract_time_parameterization/instructions_trajectory.h>

#include "tesseract_command_language_python_std_functions.h"
#include "tesseract_command_language_python_profile_dictionary_functions.h"

#include "tesseract_environment_python_std_functions.h"

%}

%shared_ptr(tesseract_planning::IterativeSplineParameterizationProfile)
%shared_ptr(tesseract_planning::ProfileSwitchProfile);


// TODO: Fix unique_ptr conversion problems
%ignore tesseract_planning::TaskInput::TaskInput;
%ignore tesseract_planning::TaskInfoContainer::addTaskInfo;
%ignore tesseract_planning::TaskInfoContainer::getTaskInfoMap;
%ignore tesseract_planning::TaskflowInterface::getTaskInfoMap;
%ignore tesseract_planning::TaskInput::getTaskInfoMap;
%ignore tesseract_planning::TaskInput::addTaskInfo;



%include "tesseract_process_managers/core/task_info.h"
%include "tesseract_process_managers/core/taskflow_interface.h"
%include "tesseract_process_managers/core/task_input.h"
%include "tesseract_process_managers/core/process_planning_request.h"
%include "tesseract_process_managers/core/process_planning_future.h"
%include "tesseract_process_managers/core/process_planning_server.h"

// %include "tesseract_process_managers/task_generators/profile_switch_task_generator.h"
// %include "tesseract_process_managers/task_generators/iterative_spline_parameterization_task_generator.h"
// %include "tesseract_process_managers/task_generators/time_optimal_parameterization_task_generator.h"


