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
#include <tesseract_motion_planners/simple/profile/simple_planner_utils.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_lvs_plan_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_lvs_no_ik_plan_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_fixed_size_plan_profile.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_fixed_size_assign_plan_profile.h>
#include <tesseract_motion_planners/simple/simple_motion_planner.h>

#include <tesseract_common/status_code.h>
#include <tesseract_common/resource_locator.h>

// tesseract_state_solver
#include <tesseract_state_solver/mutable_state_solver.h>
#include <tesseract_state_solver/state_solver.h>
#include <tesseract_state_solver/kdl/kdl_state_solver.h>
#include <tesseract_state_solver/ofkt/ofkt_state_solver.h>

#include "tesseract_environment_python_std_functions.h"

#include "tesseract_command_language_python_profile_dictionary_functions.h"
%}

// tesseract_motion_planners_simple
#define TESSERACT_MOTION_PLANNERS_SIMPLE_PUBLIC
%include "tesseract_motion_planners/simple/profile/simple_planner_utils.h"
%include "tesseract_motion_planners/simple/profile/simple_planner_profile.h"
%include "tesseract_motion_planners/simple/profile/simple_planner_lvs_plan_profile.h"
%include "tesseract_motion_planners/simple/profile/simple_planner_lvs_no_ik_plan_profile.h"
%include "tesseract_motion_planners/simple/profile/simple_planner_fixed_size_plan_profile.h"
%include "tesseract_motion_planners/simple/profile/simple_planner_fixed_size_assign_plan_profile.h"
%include "tesseract_motion_planners/simple/simple_motion_planner.h"
