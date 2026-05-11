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

// tesseract_kinematics
#include <tesseract/kinematics/joint_group.h>
#include <tesseract/kinematics/kinematic_group.h>

// tesseract_environment
#include <tesseract/environment/commands.h>
#include <tesseract/environment/events.h>
#include <tesseract/environment/environment.h>

// tesseract_collision
#include <tesseract/collision/types.h>
#include <tesseract/collision/discrete_contact_manager.h>
#include <tesseract/collision/continuous_contact_manager.h>

// tesseract_command_language
#include <tesseract/command_language/fwd.h>
#include <tesseract/command_language/move_instruction.h>
#include <tesseract/command_language/composite_instruction.h>
#include <tesseract/common/profile_dictionary.h>

// tesseract_motion_planners_simple
#include <tesseract/motion_planners/simple/profile/simple_planner_profile.h>
#include <tesseract/motion_planners/simple/profile/simple_planner_lvs_move_profile.h>
#include <tesseract/motion_planners/simple/profile/simple_planner_lvs_no_ik_move_profile.h>
#include <tesseract/motion_planners/simple/profile/simple_planner_fixed_size_move_profile.h>
#include <tesseract/motion_planners/simple/profile/simple_planner_fixed_size_assign_move_profile.h>
#include <tesseract/motion_planners/simple/simple_motion_planner.h>

// tesseract_motion_planners_trajopt
#include <tesseract/motion_planners/trajopt/trajopt_motion_planner.h>
#include <tesseract/motion_planners/trajopt/trajopt_waypoint_config.h>
#include <tesseract/motion_planners/trajopt/trajopt_utils.h>
#include <tesseract/motion_planners/trajopt/profile/trajopt_profile.h>
#include <tesseract/motion_planners/trajopt/profile/trajopt_default_composite_profile.h>
#include <tesseract/motion_planners/trajopt/profile/trajopt_default_move_profile.h>

// tesseract_motion_planners_ompl
#include <tesseract/motion_planners/ompl/ompl_planner_configurator.h>
#include <tesseract/motion_planners/ompl/profile/ompl_profile.h>
#include <tesseract/motion_planners/ompl/profile/ompl_real_vector_move_profile.h>
#include <tesseract/motion_planners/ompl/ompl_motion_planner.h>

// tesseract_motion_planner_descartes
#include <tesseract/motion_planners/descartes/profile/descartes_profile.h>
#include <tesseract/motion_planners/descartes/profile/descartes_default_move_profile.h>
#include <tesseract/motion_planners/descartes/descartes_motion_planner.h>

// tesseract_time_parameterization
#include <tesseract/time_parameterization/isp/iterative_spline_parameterization.h>
#include <tesseract/time_parameterization/totg/time_optimal_trajectory_generation.h>
#include <tesseract/time_parameterization/ruckig/ruckig_trajectory_smoothing.h>
#include <tesseract/time_parameterization/instructions_trajectory.h>

// tesseract_task_composer
#include <tesseract/task_composer/task_composer_node_info.h>
#include <tesseract/task_composer/task_composer_data_storage.h>
//#include <tesseract/task_composer/task_composer_problem.h>
#include <tesseract/task_composer/task_composer_node.h>
#include <tesseract/task_composer/task_composer_graph.h>
#include <tesseract/task_composer/task_composer_future.h>
#include <tesseract/task_composer/task_composer_task.h>
#include <tesseract/task_composer/task_composer_executor.h>
#include <tesseract/task_composer/task_composer_plugin_factory.h>
#include <tesseract/task_composer/task_composer_server.h>
#include <tesseract/task_composer/task_composer_pipeline.h>
#include <tesseract/task_composer/task_composer_context.h>
#include <tesseract/task_composer/task_composer_log.h>

// #include <tesseract/task_composer/planning/planning_task_composer_problem.h>

#include <tesseract/task_composer/taskflow/taskflow_task_composer_plugin_factories.h>
#include <tesseract/task_composer/planning/planning_task_composer_plugin_factories.h>
#include <tesseract/task_composer/task_composer_task_plugin_factory.h>

// TODO: task_composer.h doesn't compile??
// #include <tesseract/task_composer/task_composer.h>

// tesseract_task_composer profiles

// #include <tesseract/task_composer/planning/profiles/check_input_profile.h>
#include <tesseract/task_composer/planning/profiles/contact_check_profile.h>
#include <tesseract/task_composer/planning/profiles/fix_state_bounds_profile.h>
#include <tesseract/task_composer/planning/profiles/fix_state_collision_profile.h>
#include <tesseract/time_parameterization/isp/iterative_spline_parameterization_profiles.h>
#include <tesseract/task_composer/planning/profiles/min_length_profile.h>
#include <tesseract/task_composer/planning/profiles/profile_switch_profile.h>
#include <tesseract/time_parameterization/ruckig/ruckig_trajectory_smoothing_profiles.h>
#include <tesseract/time_parameterization/totg/time_optimal_trajectory_generation_profiles.h>
#include <tesseract/task_composer/planning/profiles/upsample_trajectory_profile.h>


#include <tesseract/geometry/geometries.h>
#include <tesseract/common/resource_locator.h>
#include <tesseract/srdf/kinematics_information.h>

// tesseract_state_solver
#include <tesseract/state_solver/mutable_state_solver.h>
#include <tesseract/state_solver/state_solver.h>
#include <tesseract/state_solver/kdl/kdl_state_solver.h>
#include <tesseract/state_solver/ofkt/ofkt_state_solver.h>

#include <boost_plugin_loader/plugin_loader.h>
#include <boost_plugin_loader/utils.h>

#include "tesseract_environment_python_std_functions.h"


%}

%unique_ptr_value_wrapper(tesseract::collision::DiscreteContactManager);

%define %s_u_ptr(name)
%shared_ptr(tesseract::task_composer::name)
%wrap_unique_ptr(name ## UPtr, tesseract::task_composer::name)
%enddef

%define %unique_ptr_as_planning(source_class_type, dest_class_type)
%unique_ptr_as(source_class_type, tesseract_planning, dest_class_type, tesseract::planning)
%enddef

%include "tesseract/task_composer/fwd.h"

// task_composer_keys
%include "tesseract/task_composer/task_composer_keys.h"
%template(get) tesseract::task_composer::TaskComposerKeys::get<std::string>;

// task_composer_node_info

%s_u_ptr(TaskComposerNodeInfo)
%s_u_ptr(TaskComposerNodeInfoContainer)
// %s_u_ptr(TaskComposerProblem)
// TODO: Handle maps containing unique_ptr
// %template(MapUuidTaskComposerNodeInfoUPtr) std::map<boost::uuids::uuid, std::unique_ptr<tesseract::task_composer::TaskComposerNodeInfo> >;
%ignore tesseract::task_composer::TaskComposerNodeInfoContainer::getInfoMap;
%ignore tesseract::task_composer::TaskComposerNodeInfo::find;
%ignore tesseract::task_composer::TaskComposerNodeInfoContainer::find;
%include "tesseract/task_composer/task_composer_node_info.h"

// task_composer_data_storage

%s_u_ptr(TaskComposerDataStorage)
// TODO: Handle tesseract::common::AnyPoly
%include "tesseract/task_composer/task_composer_data_storage.h"

%unique_ptr_constructor(tesseract::task_composer::TaskComposerDataStorage, %arg(), %arg());

#define TESSERACT_CLASS_EXTENSION(a,b,c,d)
%include "tesseract/task_composer/task_composer_log.h"

// task_composer_problem

// %include "tesseract/task_composer/task_composer_problem.h"

// task_composer_context
%s_u_ptr(TaskComposerContext)
// missing function definition
%ignore tesseract::task_composer::TaskComposerContext::abort;
%include "tesseract/task_composer/task_composer_context.h"

// task_composer_node
%s_u_ptr(TaskComposerNode)
%include "tesseract/task_composer/task_composer_node.h"

// task_composer_graph
%s_u_ptr(TaskComposerGraph)
%template(MapUuidTaskComposerNodeConstPtr) std::map<boost::uuids::uuid, tesseract::task_composer::TaskComposerNode::ConstPtr>;
%include "tesseract/task_composer/task_composer_graph.h"

%s_u_ptr(TaskComposerPipeline)
%include "tesseract/task_composer/task_composer_pipeline.h"

namespace std
{
enum class future_status {
    ready,
    timeout,
    deferred
};
}

// task_composer_future
%nodefaultctor tesseract::task_composer::TaskComposerFuture;
%s_u_ptr(TaskComposerFuture)

%typemap(in) const std::chrono::duration<double>& (std::chrono::duration<double> temp) {
    if (!PyFloat_Check($input)) {
        PyErr_SetString(PyExc_TypeError, "Expected a float value.");
        SWIG_fail;
    }
    temp = std::chrono::duration<double>(PyFloat_AsDouble($input));
    $1 = &temp;
}

%ignore tesseract::task_composer::TaskComposerFuture::wait_until;

// %template(MapUuidTaskComposerFuture) std::map<boost::uuids::uuid, tesseract::task_composer::TaskComposerFuture>;
%include "tesseract/task_composer/task_composer_future.h"

// task_composer_task
%s_u_ptr(TaskComposerTask)
%ignore tesseract::task_composer::TaskComposerTask::run;
%include "tesseract/task_composer/task_composer_task.h"

// task_composer_executor
%s_u_ptr(TaskComposerExecutor)
%include "tesseract/task_composer/task_composer_executor.h"

// task_composer_plugin_factory
%shared_ptr(tesseract::task_composer::TaskComposerNodeFactory)
%shared_ptr(tesseract::task_composer::TaskComposerExecutorFactory)

%shared_ptr(tesseract::task_composer::TaskComposerPluginFactory)

%include "tesseract/task_composer/task_composer_plugin_factory.h"

// task_composer_server
%shared_ptr(tesseract::task_composer::TaskComposerServer)
%include "tesseract/task_composer/task_composer_server.h"

// planning_task_composer_problem
/*%s_u_ptr(PlanningTaskComposerProblem)
%include "tesseract/task_composer/planning/planning_task_composer_problem.h"
%unique_ptr_as_planning(PlanningTaskComposerProblem, TaskComposerProblem);

%unique_ptr_constructor(tesseract::task_composer::PlanningTaskComposerProblem, %arg(std::string name), %arg(name));
%unique_ptr_constructor(tesseract::task_composer::PlanningTaskComposerProblem,%arg(
                              tesseract::task_composer::ProfileDictionary::ConstPtr profiles = nullptr,
                              std::string name = "unset"),
                              %arg(profiles, name));
%unique_ptr_constructor(tesseract::task_composer::PlanningTaskComposerProblem, %arg(tesseract_environment::Environment::ConstPtr env,
                              tesseract::common::ManipulatorInfo manip_info,
                              tesseract::task_composer::ProfileDictionary::ConstPtr profiles = nullptr,
                              std::string name = "unset"),
                              %arg(env, manip_info, profiles, name));
%unique_ptr_constructor(tesseract::task_composer::PlanningTaskComposerProblem, %arg(tesseract_environment::Environment::ConstPtr env,
                              tesseract::common::ManipulatorInfo manip_info,
                              tesseract::task_composer::ProfileRemapping move_profile_remapping,
                              tesseract::task_composer::ProfileRemapping composite_profile_remapping,
                              tesseract::task_composer::ProfileDictionary::ConstPtr profiles = nullptr,
                              std::string name = "unset"),
                              %arg(env, manip_info, move_profile_remapping, composite_profile_remapping, profiles, name));

%unique_ptr_constructor(tesseract::task_composer::PlanningTaskComposerProblem, %arg(tesseract_environment::Environment::ConstPtr env,
                              tesseract::task_composer::ProfileRemapping move_profile_remapping,
                              tesseract::task_composer::ProfileRemapping composite_profile_remapping,
                              tesseract::task_composer::ProfileDictionary::ConstPtr profiles = nullptr,
                              std::string name = "unset"),
                              %arg(env, move_profile_remapping, composite_profile_remapping, profiles, name));
%unique_ptr_constructor(tesseract::task_composer::PlanningTaskComposerProblem, %arg(tesseract_environment::Environment::ConstPtr env,
                              tesseract::task_composer::ProfileDictionary::ConstPtr profiles = nullptr,
                              std::string name = "unset"),
                              %arg(env, profiles, name));*/

#define TESSERACT_TASK_COMPOSER_PLANNING_NODES_EXPORT

// contact_check_profile
%pythondynamic tesseract::task_composer::ContactCheckProfile;
%shared_ptr(tesseract::task_composer::ContactCheckProfile)
%include "tesseract/task_composer/planning/profiles/contact_check_profile.h"
%tesseract_command_language_add_profile_type(ContactCheckProfile);

// fix_state_bounds_profile
%pythondynamic tesseract::task_composer::FixStateBoundsProfile;
%shared_ptr(tesseract::task_composer::FixStateBoundsProfile)
%include "tesseract/task_composer/planning/profiles/fix_state_bounds_profile.h"
%tesseract_command_language_add_profile_type(FixStateBoundsProfile);

// fix_state_collision_profile
%pythondynamic tesseract::task_composer::FixStateCollisionProfile;
%shared_ptr(tesseract::task_composer::FixStateCollisionProfile)
%include "tesseract/task_composer/planning/profiles/fix_state_collision_profile.h"
%tesseract_command_language_add_profile_type(FixStateCollisionProfile);

// iterative_spline_parameterization_profile
%pythondynamic tesseract::task_composer::IterativeSplineParameterizationMoveProfile;
%shared_ptr(tesseract::task_composer::IterativeSplineParameterizationMoveProfile)
%pythondynamic tesseract::task_composer::IterativeSplineParameterizationCompositeProfile;
%shared_ptr(tesseract::task_composer::IterativeSplineParameterizationCompositeProfile)
%include "tesseract/time_parameterization/isp/iterative_spline_parameterization_profiles.h"
%tesseract_command_language_add_profile_type(IterativeSplineParameterizationMoveProfile);
%tesseract_command_language_add_profile_type(IterativeSplineParameterizationCompositeProfile);

// min_length_profile
%pythondynamic tesseract::task_composer::MinLengthProfile;
%shared_ptr(tesseract::task_composer::MinLengthProfile)
%include "tesseract/task_composer/planning/profiles/min_length_profile.h"
%tesseract_command_language_add_profile_type(MinLengthProfile);

// profile_switch_profile
%pythondynamic tesseract::task_composer::ProfileSwitchProfile;
%shared_ptr(tesseract::task_composer::ProfileSwitchProfile)
%include "tesseract/task_composer/planning/profiles/profile_switch_profile.h"
%tesseract_command_language_add_profile_type(ProfileSwitchProfile);

// ruckig_trajectory_smoothing_profile
%pythondynamic tesseract::task_composer::RuckigTrajectorySmoothingCompositeProfile;
%pythondynamic tesseract::task_composer::RuckigTrajectorySmoothingMoveProfile;
%shared_ptr(tesseract::task_composer::RuckigTrajectorySmoothingCompositeProfile)
%shared_ptr(tesseract::task_composer::RuckigTrajectorySmoothingMoveProfile)
%include "tesseract/time_parameterization/ruckig/ruckig_trajectory_smoothing_profiles.h"
%tesseract_command_language_add_profile_type(RuckigTrajectorySmoothingCompositeProfile);
%tesseract_command_language_add_profile_type(RuckigTrajectorySmoothingMoveProfile);

//time_optimal_parameterization_profile
%pythondynamic tesseract::task_composer::TimeOptimalTrajectoryGenerationCompositeProfile ;
%shared_ptr(tesseract::task_composer::TimeOptimalTrajectoryGenerationCompositeProfile )
%include "tesseract/time_parameterization/totg/time_optimal_trajectory_generation_profiles.h"
%tesseract_command_language_add_profile_type(TimeOptimalTrajectoryGenerationCompositeProfile );

// upsample_trajectory_profile
%pythondynamic tesseract::task_composer::UpsampleTrajectoryProfile;
%shared_ptr(tesseract::task_composer::UpsampleTrajectoryProfile)
%include "tesseract/task_composer/planning/profiles/upsample_trajectory_profile.h"
%tesseract_command_language_add_profile_type(UpsampleTrajectoryProfile);

%init %{
// TODO: fix anchors
boost_plugin_loader::addSymbolLibraryToSearchLibrariesEnv(tesseract::task_composer::TaskComposerPlanningFactoriesAnchor(), "TESSERACT_TASK_COMPOSER_PLUGINS");
boost_plugin_loader::addSymbolLibraryToSearchLibrariesEnv(tesseract::task_composer::TaskComposerTaskflowFactoriesAnchor(), "TESSERACT_TASK_COMPOSER_PLUGINS");
boost_plugin_loader::addSymbolLibraryToSearchLibrariesEnv(tesseract::task_composer::TaskComposerTaskFactoryAnchor(), "TESSERACT_TASK_COMPOSER_PLUGINS");

%}

// %tesseract_command_language_add_profile_type(TrajOptCompositeProfile);
