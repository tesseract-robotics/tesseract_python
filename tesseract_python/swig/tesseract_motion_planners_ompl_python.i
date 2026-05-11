/**
 * @file tesseract_motion_planners_ompl_python.i
 * @brief The tesseract_motion_planners_ompl_python SWIG master file.
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

%module(directors="1", package="tesseract_robotics.tesseract_motion_planners_ompl") tesseract_motion_planners_ompl_python

#pragma SWIG nowarn=473

%include "tesseract_swig_include.i"
%include "tesseract_std_function.i"

%import "tesseract_motion_planners_python.i"

%{
// tesseract_motion_planners_ompl
#include <tesseract/motion_planners/ompl/ompl_planner_configurator.h>
#include <tesseract/motion_planners/ompl/profile/ompl_profile.h>
#include <tesseract/motion_planners/ompl/profile/ompl_real_vector_move_profile.h>
#include <tesseract/motion_planners/ompl/ompl_motion_planner.h>

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

#include <tesseract/geometry/geometries.h>
#include <tesseract/common/resource_locator.h>
#include <tesseract/srdf/kinematics_information.h>

// tesseract_state_solver
#include <tesseract/state_solver/mutable_state_solver.h>
#include <tesseract/state_solver/state_solver.h>
#include <tesseract/state_solver/kdl/kdl_state_solver.h>
#include <tesseract/state_solver/ofkt/ofkt_state_solver.h>

#include "tesseract_command_language_python_std_functions.h"

#include "tesseract_environment_python_std_functions.h"
%}

// tesseract_motion_planners_ompl
#define TESSERACT_MOTION_PLANNERS_OMPL_PUBLIC

%ignore createSolverConfig;
%ignore createSimpleSetup;
%ignore omplPlanFromXMLString;

%include "tesseract/motion_planners/ompl/types.h"

%shared_ptr(tesseract::motion_planners::OMPLPlannerConfigurator)
%shared_ptr(tesseract::motion_planners::SBLConfigurator)
%shared_ptr(tesseract::motion_planners::ESTConfigurator)
%shared_ptr(tesseract::motion_planners::LBKPIECE1Configurator)
%shared_ptr(tesseract::motion_planners::BKPIECE1Configurator)
%shared_ptr(tesseract::motion_planners::KPIECE1Configurator)
%shared_ptr(tesseract::motion_planners::BiTRRTConfigurator)
%shared_ptr(tesseract::motion_planners::RRTConfigurator)
%shared_ptr(tesseract::motion_planners::RRTConnectConfigurator)
%shared_ptr(tesseract::motion_planners::RRTstarConfigurator)
%shared_ptr(tesseract::motion_planners::TRRTConfigurator)
%shared_ptr(tesseract::motion_planners::PRMConfigurator)
%shared_ptr(tesseract::motion_planners::PRMstarConfigurator)
%shared_ptr(tesseract::motion_planners::LazyPRMstarConfigurator)
%shared_ptr(tesseract::motion_planners::SPARSConfigurator)
%ignore create(ompl::base::SpaceInformationPtr si) const;
%include "tesseract/motion_planners/ompl/ompl_planner_configurator.h"
%template(OMPLPlanners) std::vector<tesseract::motion_planners::OMPLPlannerConfigurator::ConstPtr>;
%shared_factory(
  tesseract::motion_planners::OMPLPlannerConfigurator,
  tesseract::motion_planners::SBLConfigurator,
  tesseract::motion_planners::ESTConfigurator,
  tesseract::motion_planners::LBKPIECE1Configurator,
  tesseract::motion_planners::BKPIECE1Configurator,
  tesseract::motion_planners::KPIECE1Configurator,
  tesseract::motion_planners::BiTRRTConfigurator,
  tesseract::motion_planners::RRTConfigurator,
  tesseract::motion_planners::RRTConnectConfigurator,
  tesseract::motion_planners::RRTstarConfigurator,
  tesseract::motion_planners::TRRTConfigurator,
  tesseract::motion_planners::PRMConfigurator,
  tesseract::motion_planners::PRMstarConfigurator,
  tesseract::motion_planners::LazyPRMstarConfigurator,
  tesseract::motion_planners::SPARSConfigurator
)

%pythondynamic tesseract::motion_planners::OMPLMoveProfile;
%shared_ptr(tesseract::motion_planners::OMPLMoveProfile)
%include "tesseract/motion_planners/ompl/profile/ompl_profile.h"
// %template(OMPLMoveProfileMap) std::unordered_map<std::string, std::shared_ptr<const tesseract::motion_planners::OMPLMoveProfile>>;
%tesseract_command_language_add_profile_type(OMPLMoveProfile);

%pythondynamic tesseract::motion_planners::OMPLRealVectorMoveProfile;
%shared_ptr(tesseract::motion_planners::OMPLRealVectorMoveProfile)
%include "tesseract/motion_planners/ompl/profile/ompl_real_vector_move_profile.h"

// %tesseract_std_function_base(OMPLProblemGeneratorFn,tesseract_planning,std::vector<std::shared_ptr<tesseract::motion_planners::OMPLProblem>>,const std::string&,a,const tesseract::motion_planners::PlannerRequest&,b,const tesseract::motion_planners::OMPLMoveProfileMap&,c);
// %tesseract_std_function(OMPLProblemGeneratorFn,tesseract_planning,std::vector<std::shared_ptr<tesseract::motion_planners::OMPLProblem>>,const std::string&,a,const tesseract::motion_planners::PlannerRequest&,b,const tesseract::motion_planners::OMPLMoveProfileMap&,c);

%pythondynamic tesseract::motion_planners::OMPLMotionPlanner;
%shared_ptr(tesseract::motion_planners::OMPLMotionPlanner)
%ignore tesseract::motion_planners::OMPLMotionPlanner::clone;
%include "tesseract/motion_planners/ompl/ompl_motion_planner.h"
