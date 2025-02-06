/**
 * @file tesseract_task_composer_python.i
 * @brief The tesseract_task_composer_python SWIG master file.
 *
 * @author John Wason
 * @date June 2, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2023, Wason Technology, LLC
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

%module(directors="1", package="tesseract_robotics.tesseract_task_composer") tesseract_task_composer_python

#pragma SWIG nowarn=473

%include "tesseract_swig_include.i"

%import "tesseract_motion_planners_simple_python.i"
%import "tesseract_motion_planners_trajopt_python.i"
%import "tesseract_motion_planners_ompl_python.i"
%import "tesseract_motion_planners_descartes_python.i"
%import "tesseract_time_parameterization_python.i"

%{

// tesseract_common
#include <tesseract_common/plugin_loader.h>

// tesseract_kinematics
#include <tesseract_kinematics/core/joint_group.h>
#include <tesseract_kinematics/core/kinematic_group.h>

// tesseract_environment
#include <tesseract_environment/commands.h>
#include <tesseract_environment/events.h>
#include <tesseract_environment/environment.h>

// tesseract_collision
#include <tesseract_collision/core/types.h>
#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_collision/core/continuous_contact_manager.h>

// tesseract_command_language
#include <tesseract_command_language/fwd.h>
#include <tesseract_command_language/move_instruction.h>
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/profile_dictionary.h>

// tesseract_motion_planners_simple
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
#include <tesseract_motion_planners/trajopt/trajopt_collision_config.h>

// tesseract_motion_planners_ompl
#include <tesseract_motion_planners/ompl/ompl_planner_configurator.h>
#include <tesseract_motion_planners/ompl/profile/ompl_profile.h>
#include <tesseract_motion_planners/ompl/profile/ompl_real_vector_plan_profile.h>
#include <tesseract_motion_planners/ompl/ompl_motion_planner.h>

// tesseract_motion_planner_descartes
#include <tesseract_motion_planners/descartes/profile/descartes_profile.h>
#include <tesseract_motion_planners/descartes/profile/descartes_default_plan_profile.h>
#include <tesseract_motion_planners/descartes/descartes_motion_planner.h>

// tesseract_time_parameterization
#include <tesseract_time_parameterization/isp/iterative_spline_parameterization.h>
#include <tesseract_time_parameterization/totg/time_optimal_trajectory_generation.h>
#include <tesseract_time_parameterization/ruckig/ruckig_trajectory_smoothing.h>
#include <tesseract_time_parameterization/core/instructions_trajectory.h>

// tesseract_task_composer
#include <tesseract_task_composer/core/task_composer_node_info.h>
#include <tesseract_task_composer/core/task_composer_data_storage.h>
//#include <tesseract_task_composer/core/task_composer_problem.h>
#include <tesseract_task_composer/core/task_composer_node.h>
#include <tesseract_task_composer/core/task_composer_graph.h>
#include <tesseract_task_composer/core/task_composer_future.h>
#include <tesseract_task_composer/core/task_composer_task.h>
#include <tesseract_task_composer/core/task_composer_executor.h>
#include <tesseract_task_composer/core/task_composer_plugin_factory.h>
#include <tesseract_task_composer/core/task_composer_server.h>
#include <tesseract_task_composer/core/task_composer_pipeline.h>
#include <tesseract_task_composer/core/task_composer_context.h>

// #include <tesseract_task_composer/planning/planning_task_composer_problem.h>

#include <tesseract_task_composer/taskflow/taskflow_task_composer_plugin_factories.h>
#include <tesseract_task_composer/planning/planning_task_composer_plugin_factories.h>
#include <tesseract_task_composer/core/task_composer_task_plugin_factory.h>

// TODO: task_composer.h doesn't compile??
// #include <tesseract_task_composer/task_composer.h>

// tesseract_task_composer profiles

// #include <tesseract_task_composer/planning/profiles/check_input_profile.h>
#include <tesseract_task_composer/planning/profiles/contact_check_profile.h>
#include <tesseract_task_composer/planning/profiles/fix_state_bounds_profile.h>
#include <tesseract_task_composer/planning/profiles/fix_state_collision_profile.h>
#include <tesseract_task_composer/planning/profiles/iterative_spline_parameterization_profile.h>
#include <tesseract_task_composer/planning/profiles/min_length_profile.h>
#include <tesseract_task_composer/planning/profiles/profile_switch_profile.h>
#include <tesseract_task_composer/planning/profiles/ruckig_trajectory_smoothing_profile.h>
#include <tesseract_task_composer/planning/profiles/time_optimal_parameterization_profile.h>
#include <tesseract_task_composer/planning/profiles/upsample_trajectory_profile.h>


#include <tesseract_geometry/geometries.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_srdf/kinematics_information.h>

// tesseract_state_solver
#include <tesseract_state_solver/mutable_state_solver.h>
#include <tesseract_state_solver/state_solver.h>
#include <tesseract_state_solver/kdl/kdl_state_solver.h>
#include <tesseract_state_solver/ofkt/ofkt_state_solver.h>

#include "tesseract_environment_python_std_functions.h"


%}

%unique_ptr_value_wrapper(tesseract_collision::DiscreteContactManager);

%define %s_u_ptr(name)
%shared_ptr(tesseract_planning::name)
%wrap_unique_ptr(name ## UPtr, tesseract_planning::name)
%enddef

%define %unique_ptr_as_planning(source_class_type, dest_class_type)
%unique_ptr_as(source_class_type, tesseract_planning, dest_class_type, tesseract_planning)
%enddef

%include "tesseract_task_composer/core/fwd.h"

// task_composer_keys
%include "tesseract_task_composer/core/task_composer_keys.h"
%template(get) tesseract_planning::TaskComposerKeys::get<std::string>;

// task_composer_node_info

%s_u_ptr(TaskComposerNodeInfo)
%s_u_ptr(TaskComposerNodeInfoContainer)
// %s_u_ptr(TaskComposerProblem)
// TODO: Handle maps containing unique_ptr
// %template(MapUuidTaskComposerNodeInfoUPtr) std::map<boost::uuids::uuid, std::unique_ptr<tesseract_planning::TaskComposerNodeInfo> >;
%ignore tesseract_planning::TaskComposerNodeInfoContainer::getInfoMap;
%ignore tesseract_planning::TaskComposerNodeInfo::find;
%ignore tesseract_planning::TaskComposerNodeInfoContainer::find;
%include "tesseract_task_composer/core/task_composer_node_info.h"

// task_composer_data_storage

%s_u_ptr(TaskComposerDataStorage)
// TODO: Handle tesseract_common::AnyPoly
%include "tesseract_task_composer/core/task_composer_data_storage.h"

%unique_ptr_constructor(tesseract_planning::TaskComposerDataStorage, %arg(), %arg());

// task_composer_problem

// %include "tesseract_task_composer/core/task_composer_problem.h"

// task_composer_context
%s_u_ptr(TaskComposerContext)
// missing function definition
%ignore tesseract_planning::TaskComposerContext::abort;
%include "tesseract_task_composer/core/task_composer_context.h"

// task_composer_node
%s_u_ptr(TaskComposerNode)
%include "tesseract_task_composer/core/task_composer_node.h"

// task_composer_graph
%s_u_ptr(TaskComposerGraph)
%template(MapUuidTaskComposerNodeConstPtr) std::map<boost::uuids::uuid, tesseract_planning::TaskComposerNode::ConstPtr>;
%include "tesseract_task_composer/core/task_composer_graph.h"

%s_u_ptr(TaskComposerPipeline)
%include "tesseract_task_composer/core/task_composer_pipeline.h"

namespace std
{
enum class future_status {
    ready,
    timeout,
    deferred
};
}

// task_composer_future
%nodefaultctor tesseract_planning::TaskComposerFuture;
%s_u_ptr(TaskComposerFuture)

%typemap(in) const std::chrono::duration<double>& (std::chrono::duration<double> temp) {
    if (!PyFloat_Check($input)) {
        PyErr_SetString(PyExc_TypeError, "Expected a float value.");
        SWIG_fail;
    }
    temp = std::chrono::duration<double>(PyFloat_AsDouble($input));
    $1 = &temp;
}

%ignore tesseract_planning::TaskComposerFuture::wait_until;

// %template(MapUuidTaskComposerFuture) std::map<boost::uuids::uuid, tesseract_planning::TaskComposerFuture>;
%include "tesseract_task_composer/core/task_composer_future.h"

// task_composer_task
%s_u_ptr(TaskComposerTask)
%ignore tesseract_planning::TaskComposerTask::run;
%include "tesseract_task_composer/core/task_composer_task.h"

// task_composer_executor
%s_u_ptr(TaskComposerExecutor)
%include "tesseract_task_composer/core/task_composer_executor.h"

// task_composer_plugin_factory
%shared_ptr(tesseract_planning::TaskComposerNodeFactory)
%shared_ptr(tesseract_planning::TaskComposerExecutorFactory)

%shared_ptr(tesseract_planning::TaskComposerPluginFactory)

%include "tesseract_task_composer/core/task_composer_plugin_factory.h"

// task_composer_server
%shared_ptr(tesseract_planning::TaskComposerServer)
%include "tesseract_task_composer/core/task_composer_server.h"

// planning_task_composer_problem
/*%s_u_ptr(PlanningTaskComposerProblem)
%include "tesseract_task_composer/planning/planning_task_composer_problem.h"
%unique_ptr_as_planning(PlanningTaskComposerProblem, TaskComposerProblem);

%unique_ptr_constructor(tesseract_planning::PlanningTaskComposerProblem, %arg(std::string name), %arg(name));
%unique_ptr_constructor(tesseract_planning::PlanningTaskComposerProblem,%arg(
                              tesseract_planning::ProfileDictionary::ConstPtr profiles = nullptr,
                              std::string name = "unset"),
                              %arg(profiles, name));
%unique_ptr_constructor(tesseract_planning::PlanningTaskComposerProblem, %arg(tesseract_environment::Environment::ConstPtr env,
                              tesseract_common::ManipulatorInfo manip_info,
                              tesseract_planning::ProfileDictionary::ConstPtr profiles = nullptr,
                              std::string name = "unset"),
                              %arg(env, manip_info, profiles, name));
%unique_ptr_constructor(tesseract_planning::PlanningTaskComposerProblem, %arg(tesseract_environment::Environment::ConstPtr env,
                              tesseract_common::ManipulatorInfo manip_info,
                              tesseract_planning::ProfileRemapping move_profile_remapping,
                              tesseract_planning::ProfileRemapping composite_profile_remapping,
                              tesseract_planning::ProfileDictionary::ConstPtr profiles = nullptr,
                              std::string name = "unset"),
                              %arg(env, manip_info, move_profile_remapping, composite_profile_remapping, profiles, name));

%unique_ptr_constructor(tesseract_planning::PlanningTaskComposerProblem, %arg(tesseract_environment::Environment::ConstPtr env,
                              tesseract_planning::ProfileRemapping move_profile_remapping,
                              tesseract_planning::ProfileRemapping composite_profile_remapping,
                              tesseract_planning::ProfileDictionary::ConstPtr profiles = nullptr,
                              std::string name = "unset"),
                              %arg(env, move_profile_remapping, composite_profile_remapping, profiles, name));
%unique_ptr_constructor(tesseract_planning::PlanningTaskComposerProblem, %arg(tesseract_environment::Environment::ConstPtr env,
                              tesseract_planning::ProfileDictionary::ConstPtr profiles = nullptr,
                              std::string name = "unset"),
                              %arg(env, profiles, name));*/

// contact_check_profile
%pythondynamic tesseract_planning::ContactCheckProfile;
%shared_ptr(tesseract_planning::ContactCheckProfile)
%include "tesseract_task_composer/planning/profiles/contact_check_profile.h"
%tesseract_command_language_add_profile_type(ContactCheckProfile);

// fix_state_bounds_profile
%pythondynamic tesseract_planning::FixStateBoundsProfile;
%shared_ptr(tesseract_planning::FixStateBoundsProfile)
%include "tesseract_task_composer/planning/profiles/fix_state_bounds_profile.h"
%tesseract_command_language_add_profile_type(FixStateBoundsProfile);

// fix_state_collision_profile
%pythondynamic tesseract_planning::FixStateCollisionProfile;
%shared_ptr(tesseract_planning::FixStateCollisionProfile)
%include "tesseract_task_composer/planning/profiles/fix_state_collision_profile.h"
%tesseract_command_language_add_profile_type(FixStateCollisionProfile);

// iterative_spline_parameterization_profile
%pythondynamic tesseract_planning::IterativeSplineParameterizationProfile;
%shared_ptr(tesseract_planning::IterativeSplineParameterizationProfile)
%include "tesseract_task_composer/planning/profiles/iterative_spline_parameterization_profile.h"
%tesseract_command_language_add_profile_type(IterativeSplineParameterizationProfile);

// min_length_profile
%pythondynamic tesseract_planning::MinLengthProfile;
%shared_ptr(tesseract_planning::MinLengthProfile)
%include "tesseract_task_composer/planning/profiles/min_length_profile.h"
%tesseract_command_language_add_profile_type(MinLengthProfile);

// profile_switch_profile
%pythondynamic tesseract_planning::ProfileSwitchProfile;
%shared_ptr(tesseract_planning::ProfileSwitchProfile)
%include "tesseract_task_composer/planning/profiles/profile_switch_profile.h"
%tesseract_command_language_add_profile_type(ProfileSwitchProfile);

// ruckig_trajectory_smoothing_profile
%pythondynamic tesseract_planning::RuckigTrajectorySmoothingCompositeProfile;
%pythondynamic tesseract_planning::RuckigTrajectorySmoothingMoveProfile;
%shared_ptr(tesseract_planning::RuckigTrajectorySmoothingCompositeProfile)
%shared_ptr(tesseract_planning::RuckigTrajectorySmoothingMoveProfile)
%include "tesseract_task_composer/planning/profiles/ruckig_trajectory_smoothing_profile.h"
%tesseract_command_language_add_profile_type(RuckigTrajectorySmoothingCompositeProfile);
%tesseract_command_language_add_profile_type(RuckigTrajectorySmoothingMoveProfile);

//time_optimal_parameterization_profile
%pythondynamic tesseract_planning::TimeOptimalParameterizationProfile;
%shared_ptr(tesseract_planning::TimeOptimalParameterizationProfile)
%include "tesseract_task_composer/planning/profiles/time_optimal_parameterization_profile.h"
%tesseract_command_language_add_profile_type(TimeOptimalParameterizationProfile);

// upsample_trajectory_profile
%pythondynamic tesseract_planning::UpsampleTrajectoryProfile;
%shared_ptr(tesseract_planning::UpsampleTrajectoryProfile)
%include "tesseract_task_composer/planning/profiles/upsample_trajectory_profile.h"
%tesseract_command_language_add_profile_type(UpsampleTrajectoryProfile);

%init %{
// TODO: fix anchors
tesseract_common::PluginLoader::addSymbolLibraryToSearchLibrariesEnv(tesseract_planning::TaskComposerPlanningFactoriesAnchor(), "TESSERACT_TASK_COMPOSER_PLUGINS");
tesseract_common::PluginLoader::addSymbolLibraryToSearchLibrariesEnv(tesseract_planning::TaskComposerTaskflowFactoriesAnchor(), "TESSERACT_TASK_COMPOSER_PLUGINS");
tesseract_common::PluginLoader::addSymbolLibraryToSearchLibrariesEnv(tesseract_planning::TaskComposerTaskFactoryAnchor(), "TESSERACT_TASK_COMPOSER_PLUGINS");

%}

// %tesseract_command_language_add_profile_type(TrajOptCompositeProfile);
