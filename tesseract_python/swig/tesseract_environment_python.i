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


#include <tesseract/geometry/geometries.h>

#include <tesseract/common/resource_locator.h>

// tesseract_state_solver
#include <tesseract/state_solver/mutable_state_solver.h>
#include <tesseract/state_solver/state_solver.h>
#include <tesseract/state_solver/kdl/kdl_state_solver.h>
#include <tesseract/state_solver/ofkt/ofkt_state_solver.h>

// tesseract_kinematics
#include <tesseract/kinematics/joint_group.h>
#include <tesseract/kinematics/kinematic_group.h>

// tesseract_collision
#include <tesseract/collision/types.h>
#include <tesseract/collision/discrete_contact_manager.h>
#include <tesseract/collision/continuous_contact_manager.h>

// tesseract_environment
#include <tesseract/environment/commands.h>
#include <tesseract/environment/events.h>
#include <tesseract/environment/environment.h>

#include "tesseract_common_python_std_functions.h"
#include "tesseract_collisions_python_std_functions.h"

#include "tesseract_environment_python_std_functions.h"
%}

%unique_ptr_value_wrapper(tesseract::collision::ContinuousContactManager);
%unique_ptr_value_wrapper(tesseract::collision::DiscreteContactManager);
%unique_ptr_value_wrapper(tesseract::scene_graph::StateSolver);
// %unique_ptr_value_wrapper(tesseract::kinematics::KinematicGroup)
// %unique_ptr_value_wrapper(tesseract::kinematics::JointGroup)

%tesseract_std_function(FindTCPOffsetCallbackFn,tesseract,Eigen::Isometry3d,const tesseract::common::ManipulatorInfo&,a);
%tesseract_std_function(EventCallbackFn,tesseract::environment,void,const tesseract::environment::Event&,a);

%ignore getEventCallbacks;
%ignore lockRead;

// tesseract_environment
#define TESSERACT_ENVIRONMENT_CORE_PUBLIC
%include "tesseract/environment/commands.h"

%shared_ptr(tesseract::environment::Command)
%shared_ptr(tesseract::environment::AddContactManagersPluginInfoCommand)
%shared_ptr(tesseract::environment::AddKinematicsInformationCommand)
%shared_ptr(tesseract::environment::AddLinkCommand)
%shared_ptr(tesseract::environment::AddSceneGraphCommand)
%shared_ptr(tesseract::environment::AddTrajectoryLinkCommand)
%shared_ptr(tesseract::environment::ChangeCollisionMarginsCommand)
%shared_ptr(tesseract::environment::ChangeJointAccelerationLimitsCommand)
%shared_ptr(tesseract::environment::ChangeJointOriginCommand)
%shared_ptr(tesseract::environment::ChangeJointPositionLimitsCommand)
%shared_ptr(tesseract::environment::ChangeJointVelocityLimitsCommand)
%shared_ptr(tesseract::environment::ChangeLinkCollisionEnabledCommand)
%shared_ptr(tesseract::environment::ChangeLinkOriginCommand)
%shared_ptr(tesseract::environment::ChangeLinkVisibilityCommand)
%shared_ptr(tesseract::environment::ModifyAllowedCollisionsCommand)
%shared_ptr(tesseract::environment::MoveLinkCommand)
%shared_ptr(tesseract::environment::MoveJointCommand)
%shared_ptr(tesseract::environment::RemoveLinkCommand)
%shared_ptr(tesseract::environment::RemoveJointCommand)
%shared_ptr(tesseract::environment::ReplaceJointCommand)
%shared_ptr(tesseract::environment::RemoveAllowedCollisionLinkCommand)
%shared_ptr(tesseract::environment::SetActiveContinuousContactManagerCommand)
%shared_ptr(tesseract::environment::SetActiveDiscreteContactManagerCommand)

%shared_factory(
    tesseract::environment::AddContactManagersPluginInfoCommand,
    tesseract::environment::AddKinematicsInformationCommand,
    tesseract::environment::AddLinkCommand,
    tesseract::environment::AddSceneGraphCommand,
    tesseract::environment::AddTrajectoryLinkCommand,
    tesseract::environment::ChangeCollisionMarginsCommand,
    tesseract::environment::ChangeJointAccelerationLimitsCommand,
    tesseract::environment::ChangeJointOriginCommand,
    tesseract::environment::ChangeJointPositionLimitsCommand,
    tesseract::environment::ChangeJointVelocityLimitsCommand,
    tesseract::environment::ChangeLinkCollisionEnabledCommand,
    tesseract::environment::ChangeLinkOriginCommand,
    tesseract::environment::ChangeLinkVisibilityCommand,
    tesseract::environment::ModifyAllowedCollisionsCommand,
    tesseract::environment::MoveLinkCommand,
    tesseract::environment::MoveJointCommand,
    tesseract::environment::RemoveLinkCommand,
    tesseract::environment::RemoveJointCommand,
    tesseract::environment::ReplaceJointCommand,
    tesseract::environment::RemoveAllowedCollisionLinkCommand,
    tesseract::environment::SetActiveContinuousContactManagerCommand,
    tesseract::environment::SetActiveDiscreteContactManagerCommand
)

%include "tesseract/environment/command.h"

%template(Commands) std::vector<tesseract::environment::Command::ConstPtr>;

%include "tesseract/environment/commands/add_contact_managers_plugin_info_command.h"
%include "tesseract/environment/commands/add_kinematics_information_command.h"
%include "tesseract/environment/commands/add_link_command.h"
%include "tesseract/environment/commands/add_scene_graph_command.h"
%include "tesseract/environment/commands/add_trajectory_link_command.h"
%include "tesseract/environment/commands/change_collision_margins_command.h"
%include "tesseract/environment/commands/change_joint_acceleration_limits_command.h"
%include "tesseract/environment/commands/change_joint_origin_command.h"
%include "tesseract/environment/commands/change_joint_position_limits_command.h"
%include "tesseract/environment/commands/change_joint_velocity_limits_command.h"
%include "tesseract/environment/commands/change_link_collision_enabled_command.h"
%include "tesseract/environment/commands/change_link_origin_command.h"
%include "tesseract/environment/commands/change_link_visibility_command.h"
%include "tesseract/environment/commands/modify_allowed_collisions_command.h"
%include "tesseract/environment/commands/move_joint_command.h"
%include "tesseract/environment/commands/move_link_command.h"
%include "tesseract/environment/commands/remove_allowed_collision_link_command.h"
%include "tesseract/environment/commands/remove_joint_command.h"
%include "tesseract/environment/commands/remove_link_command.h"
%include "tesseract/environment/commands/replace_joint_command.h"
%include "tesseract/environment/commands/set_active_continuous_contact_manager_command.h"
%include "tesseract/environment/commands/set_active_discrete_contact_manager_command.h"

%include "tesseract/environment/events.h"

%shared_ptr(tesseract::environment::Environment)
%wrap_unique_ptr(EnvironmentUPtr,tesseract::environment::Environment)

%ignore tesseract::environment::Environment::Environment(std::unique_ptr<Implementation> impl);

%include "tesseract/environment/environment.h"

%inline {
    const tesseract::environment::SceneStateChangedEvent& cast_SceneStateChangedEvent(
        const tesseract::environment::Event& a
        )
    {
        return dynamic_cast<const tesseract::environment::SceneStateChangedEvent&>(a);
    }
}

%inline {
    const tesseract::environment::CommandAppliedEvent& cast_CommandAppliedEvent(
        const tesseract::environment::Event& a
        )
    {
        return dynamic_cast<const tesseract::environment::CommandAppliedEvent&>(a);
    }
}

%tesseract_any_poly_type_shared_ptr(Environment, tesseract::environment)
