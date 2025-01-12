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
#include <tesseract_motion_planners/ompl/ompl_planner_configurator.h>
#include <tesseract_motion_planners/ompl/profile/ompl_profile.h>
#include <tesseract_motion_planners/ompl/profile/ompl_real_vector_plan_profile.h>
#include <tesseract_motion_planners/ompl/ompl_motion_planner.h>

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

#include <tesseract_geometry/geometries.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_srdf/kinematics_information.h>

// tesseract_state_solver
#include <tesseract_state_solver/mutable_state_solver.h>
#include <tesseract_state_solver/state_solver.h>
#include <tesseract_state_solver/kdl/kdl_state_solver.h>
#include <tesseract_state_solver/ofkt/ofkt_state_solver.h>

#include "tesseract_command_language_python_std_functions.h"

#include "tesseract_environment_python_std_functions.h"
%}

// tesseract_motion_planners_ompl
#define TESSERACT_MOTION_PLANNERS_OMPL_PUBLIC

%ignore createSolverConfig;
%ignore createSimpleSetup;
%ignore omplPlanFromXMLString;

%include "tesseract_motion_planners/ompl/types.h"

%shared_ptr(tesseract_planning::OMPLPlannerConfigurator)
%shared_ptr(tesseract_planning::SBLConfigurator)
%shared_ptr(tesseract_planning::ESTConfigurator)
%shared_ptr(tesseract_planning::LBKPIECE1Configurator)
%shared_ptr(tesseract_planning::BKPIECE1Configurator)
%shared_ptr(tesseract_planning::KPIECE1Configurator)
%shared_ptr(tesseract_planning::BiTRRTConfigurator)
%shared_ptr(tesseract_planning::RRTConfigurator)
%shared_ptr(tesseract_planning::RRTConnectConfigurator)
%shared_ptr(tesseract_planning::RRTstarConfigurator)
%shared_ptr(tesseract_planning::TRRTConfigurator)
%shared_ptr(tesseract_planning::PRMConfigurator)
%shared_ptr(tesseract_planning::PRMstarConfigurator)
%shared_ptr(tesseract_planning::LazyPRMstarConfigurator)
%shared_ptr(tesseract_planning::SPARSConfigurator)
%ignore create(ompl::base::SpaceInformationPtr si) const;
%include "tesseract_motion_planners/ompl/ompl_planner_configurator.h"
%template(OMPLPlanners) std::vector<tesseract_planning::OMPLPlannerConfigurator::ConstPtr>;
%shared_factory(
  tesseract_planning::OMPLPlannerConfigurator,
  tesseract_planning::SBLConfigurator,
  tesseract_planning::ESTConfigurator,
  tesseract_planning::LBKPIECE1Configurator,
  tesseract_planning::BKPIECE1Configurator,
  tesseract_planning::KPIECE1Configurator,
  tesseract_planning::BiTRRTConfigurator,
  tesseract_planning::RRTConfigurator,
  tesseract_planning::RRTConnectConfigurator,
  tesseract_planning::RRTstarConfigurator,
  tesseract_planning::TRRTConfigurator,
  tesseract_planning::PRMConfigurator,
  tesseract_planning::PRMstarConfigurator,
  tesseract_planning::LazyPRMstarConfigurator,
  tesseract_planning::SPARSConfigurator
)

%pythondynamic tesseract_planning::OMPLPlanProfile;
%shared_ptr(tesseract_planning::OMPLPlanProfile)
%include "tesseract_motion_planners/ompl/profile/ompl_profile.h"
// %template(OMPLPlanProfileMap) std::unordered_map<std::string, std::shared_ptr<const tesseract_planning::OMPLPlanProfile>>;
%tesseract_command_language_add_profile_type(OMPLPlanProfile);

%pythondynamic tesseract_planning::OMPLRealVectorPlanProfile;
%shared_ptr(tesseract_planning::OMPLRealVectorPlanProfile)
%include "tesseract_motion_planners/ompl/profile/ompl_real_vector_plan_profile.h"

// %tesseract_std_function_base(OMPLProblemGeneratorFn,tesseract_planning,std::vector<std::shared_ptr<tesseract_planning::OMPLProblem>>,const std::string&,a,const tesseract_planning::PlannerRequest&,b,const tesseract_planning::OMPLPlanProfileMap&,c);
// %tesseract_std_function(OMPLProblemGeneratorFn,tesseract_planning,std::vector<std::shared_ptr<tesseract_planning::OMPLProblem>>,const std::string&,a,const tesseract_planning::PlannerRequest&,b,const tesseract_planning::OMPLPlanProfileMap&,c);

%pythondynamic tesseract_planning::OMPLMotionPlanner;
%shared_ptr(tesseract_planning::OMPLMotionPlanner)
%ignore tesseract_planning::OMPLMotionPlanner::clone;
%include "tesseract_motion_planners/ompl/ompl_motion_planner.h"
