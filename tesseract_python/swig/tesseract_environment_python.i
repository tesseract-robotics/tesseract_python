/**
 * @file tesseract_environment_python.i
 * @brief The tesseract_environment_python SWIG master file.
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

%module(directors="1", package="tesseract_robotics.tesseract_environment") tesseract_environment_python

#pragma SWIG nowarn=473

%include "tesseract_swig_include.i"

%import "tesseract_kinematics_python.i"
%import "tesseract_collision_python.i"
%import "tesseract_srdf_python.i"
%import "tesseract_state_solver_python.i"

%{

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/directed_graph.hpp>
#include <boost/graph/properties.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/breadth_first_search.hpp>


#include <tesseract_geometry/geometries.h>

#include <tesseract_common/resource_locator.h>

// tesseract_state_solver
#include <tesseract_state_solver/mutable_state_solver.h>
#include <tesseract_state_solver/state_solver.h>
#include <tesseract_state_solver/kdl/kdl_state_solver.h>
#include <tesseract_state_solver/ofkt/ofkt_state_solver.h>

// tesseract_kinematics
#include <tesseract_kinematics/core/joint_group.h>
#include <tesseract_kinematics/core/kinematic_group.h>

// tesseract_collision
#include <tesseract_collision/core/types.h>
#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_collision/core/continuous_contact_manager.h>

// tesseract_environment
#include <tesseract_environment/commands.h>
#include <tesseract_environment/events.h>
#include <tesseract_environment/environment.h>

#include "tesseract_common_python_std_functions.h"
#include "tesseract_collisions_python_std_functions.h"

#include "tesseract_environment_python_std_functions.h"
%}

%unique_ptr_value_wrapper(tesseract_collision::ContinuousContactManager);
%unique_ptr_value_wrapper(tesseract_collision::DiscreteContactManager);
%unique_ptr_value_wrapper(tesseract_scene_graph::StateSolver);
// %unique_ptr_value_wrapper(tesseract_kinematics::KinematicGroup)
// %unique_ptr_value_wrapper(tesseract_kinematics::JointGroup)

%tesseract_std_function(FindTCPOffsetCallbackFn,tesseract,Eigen::Isometry3d,const tesseract_common::ManipulatorInfo&,a);
%tesseract_std_function(EventCallbackFn,tesseract_environment,void,const tesseract_environment::Event&,a);

%ignore getEventCallbacks;
%ignore lockRead;

// tesseract_environment
#define TESSERACT_ENVIRONMENT_CORE_PUBLIC
%include "tesseract_environment/commands.h"

%shared_ptr(tesseract_environment::Command)
%shared_ptr(tesseract_environment::AddContactManagersPluginInfoCommand)
%shared_ptr(tesseract_environment::AddKinematicsInformationCommand)
%shared_ptr(tesseract_environment::AddLinkCommand)
%shared_ptr(tesseract_environment::AddSceneGraphCommand)
%shared_ptr(tesseract_environment::AddTrajectoryLinkCommand)
%shared_ptr(tesseract_environment::ChangeCollisionMarginsCommand)
%shared_ptr(tesseract_environment::ChangeJointAccelerationLimitsCommand)
%shared_ptr(tesseract_environment::ChangeJointOriginCommand)
%shared_ptr(tesseract_environment::ChangeJointPositionLimitsCommand)
%shared_ptr(tesseract_environment::ChangeJointVelocityLimitsCommand)
%shared_ptr(tesseract_environment::ChangeLinkCollisionEnabledCommand)
%shared_ptr(tesseract_environment::ChangeLinkOriginCommand)
%shared_ptr(tesseract_environment::ChangeLinkVisibilityCommand)
%shared_ptr(tesseract_environment::ModifyAllowedCollisionsCommand)
%shared_ptr(tesseract_environment::MoveLinkCommand)
%shared_ptr(tesseract_environment::MoveJointCommand)
%shared_ptr(tesseract_environment::RemoveLinkCommand)
%shared_ptr(tesseract_environment::RemoveJointCommand)
%shared_ptr(tesseract_environment::ReplaceJointCommand)
%shared_ptr(tesseract_environment::RemoveAllowedCollisionLinkCommand)
%shared_ptr(tesseract_environment::SetActiveContinuousContactManagerCommand)
%shared_ptr(tesseract_environment::SetActiveDiscreteContactManagerCommand)

%shared_factory(
    tesseract_environment::AddContactManagersPluginInfoCommand,
    tesseract_environment::AddKinematicsInformationCommand,
    tesseract_environment::AddLinkCommand,
    tesseract_environment::AddSceneGraphCommand,
    tesseract_environment::AddTrajectoryLinkCommand,
    tesseract_environment::ChangeCollisionMarginsCommand,
    tesseract_environment::ChangeJointAccelerationLimitsCommand,
    tesseract_environment::ChangeJointOriginCommand,
    tesseract_environment::ChangeJointPositionLimitsCommand,
    tesseract_environment::ChangeJointVelocityLimitsCommand,
    tesseract_environment::ChangeLinkCollisionEnabledCommand,
    tesseract_environment::ChangeLinkOriginCommand,
    tesseract_environment::ChangeLinkVisibilityCommand,
    tesseract_environment::ModifyAllowedCollisionsCommand,
    tesseract_environment::MoveLinkCommand,
    tesseract_environment::MoveJointCommand,
    tesseract_environment::RemoveLinkCommand,
    tesseract_environment::RemoveJointCommand,
    tesseract_environment::ReplaceJointCommand,
    tesseract_environment::RemoveAllowedCollisionLinkCommand,
    tesseract_environment::SetActiveContinuousContactManagerCommand,
    tesseract_environment::SetActiveDiscreteContactManagerCommand
)

%include "tesseract_environment/command.h"

%template(Commands) std::vector<tesseract_environment::Command::ConstPtr>;

%include "tesseract_environment/commands/add_contact_managers_plugin_info_command.h"
%include "tesseract_environment/commands/add_kinematics_information_command.h"
%include "tesseract_environment/commands/add_link_command.h"
%include "tesseract_environment/commands/add_scene_graph_command.h"
%include "tesseract_environment/commands/add_trajectory_link_command.h"
%include "tesseract_environment/commands/change_collision_margins_command.h"
%include "tesseract_environment/commands/change_joint_acceleration_limits_command.h"
%include "tesseract_environment/commands/change_joint_origin_command.h"
%include "tesseract_environment/commands/change_joint_position_limits_command.h"
%include "tesseract_environment/commands/change_joint_velocity_limits_command.h"
%include "tesseract_environment/commands/change_link_collision_enabled_command.h"
%include "tesseract_environment/commands/change_link_origin_command.h"
%include "tesseract_environment/commands/change_link_visibility_command.h"
%include "tesseract_environment/commands/modify_allowed_collisions_command.h"
%include "tesseract_environment/commands/move_joint_command.h"
%include "tesseract_environment/commands/move_link_command.h"
%include "tesseract_environment/commands/remove_allowed_collision_link_command.h"
%include "tesseract_environment/commands/remove_joint_command.h"
%include "tesseract_environment/commands/remove_link_command.h"
%include "tesseract_environment/commands/replace_joint_command.h"
%include "tesseract_environment/commands/set_active_continuous_contact_manager_command.h"
%include "tesseract_environment/commands/set_active_discrete_contact_manager_command.h"

%include "tesseract_environment/events.h"

%shared_ptr(tesseract_environment::Environment)
%wrap_unique_ptr(EnvironmentUPtr,tesseract_environment::Environment)

%ignore tesseract_environment::Environment::Environment(std::unique_ptr<Implementation> impl);

%include "tesseract_environment/environment.h"

%inline {
    const tesseract_environment::SceneStateChangedEvent& cast_SceneStateChangedEvent(
        const tesseract_environment::Event& a
        )
    {
        return dynamic_cast<const tesseract_environment::SceneStateChangedEvent&>(a);
    }
}

%inline {
    const tesseract_environment::CommandAppliedEvent& cast_CommandAppliedEvent(
        const tesseract_environment::Event& a
        )
    {
        return dynamic_cast<const tesseract_environment::CommandAppliedEvent&>(a);
    }
}

%tesseract_any_poly_type_shared_ptr(Environment, tesseract_environment)
