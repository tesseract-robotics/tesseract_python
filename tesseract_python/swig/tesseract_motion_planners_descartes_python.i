/**
 * @file tesseract_motion_planners_descartes_python.i
 * @brief The tesseract_motion_planners_descartes_python SWIG master file.
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

%module(directors="1", package="tesseract_robotics.tesseract_motion_planners_descartes") tesseract_motion_planners_descartes_python

#pragma SWIG nowarn=473

%include "tesseract_swig_include.i"
%include "tesseract_std_function.i"

%import "tesseract_motion_planners_python.i"

%ignore edge_evaluator;
%ignore state_evaluator;
%ignore vertex_evaluator;

%{
// tesseract_motion_planner_descartes
#include <tesseract_motion_planners/descartes/profile/descartes_profile.h>
#include <tesseract_motion_planners/descartes/profile/descartes_default_move_profile.h>
#include <tesseract_motion_planners/descartes/descartes_motion_planner.h>


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
#include <tesseract_command_language/fwd.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract/common/profile_dictionary.h>

// tesseract_motion_planners
#include <tesseract_motion_planners/core/planner.h>
#include <tesseract_motion_planners/core/types.h>

// tesseract_visualization
#include <tesseract/visualization/visualization.h>

#include "tesseract_command_language_python_std_functions.h"

#include "tesseract_environment_python_std_functions.h"
%}

// tesseract_motion_planner_descartes
#define TESSERACT_MOTION_PLANNERS_DESCARTES_PUBLIC

%ignore createWaypointSampler;
%ignore createEdgeEvaluator;
%ignore createStateEvaluator;

%include "tesseract_motion_planners/descartes/descartes_utils.h"
%tesseract_std_function_base(PoseSamplerFn,tesseract_planning,tesseract::common::VectorIsometry3d,const Eigen::Isometry3d&,a);
%tesseract_std_function(PoseSamplerFn,tesseract_planning,tesseract::common::VectorIsometry3d,const Eigen::Isometry3d&,a);

%pythondynamic tesseract_planning::DescartesMoveProfileD;
%shared_ptr(tesseract_planning::DescartesMoveProfile<double>)
%include "tesseract_motion_planners/descartes/profile/descartes_profile.h"
%template(DescartesMoveProfileD) std::shared_ptr<tesseract_planning::DescartesMoveProfile<double> >;
%template(DescartesMoveProfileMapD) std::unordered_map<std::string, std::shared_ptr<const tesseract_planning::DescartesMoveProfile<double>>>;
namespace tesseract_planning {using DescartesMoveProfileMapD = std::unordered_map<std::string, std::shared_ptr<const DescartesMoveProfile<double>>>;}
%tesseract_command_language_add_profile_type2(DescartesMoveProfileD,DescartesMoveProfile<double>);

%pythondynamic tesseract_planning::DescartesDefaultMoveProfileD;
%shared_ptr(tesseract_planning::DescartesDefaultMoveProfile<double>)
%ignore tesseract_planning::DescartesDefaultMoveProfile::edge_evaluator;
%ignore tesseract_planning::DescartesDefaultMoveProfile::is_valid;
%include "tesseract_motion_planners/descartes/profile/descartes_default_move_profile.h"
%template(DescartesDefaultMoveProfileD) tesseract_planning::DescartesDefaultMoveProfile<double>;

%pythondynamic tesseract_planning::DescartesMotionPlannerD;
%pythondynamic tesseract_planning::DescartesMotionPlanner<double>;
%shared_ptr(tesseract_planning::DescartesMotionPlannerD);
%shared_ptr(tesseract_planning::DescartesMotionPlanner<double>);
%ignore tesseract_planning::DescartesMotionPlanner::clone;
%include "tesseract_motion_planners/descartes/descartes_motion_planner.h"
%template(DescartesMotionPlannerD) tesseract_planning::DescartesMotionPlanner<double>;

%inline {
    std::shared_ptr<tesseract::common::Profile> cast_DescartesMoveProfileD(
        const std::shared_ptr<tesseract_planning::DescartesDefaultMoveProfile<double>>& a
    )
    {
        return a;
    }
}
