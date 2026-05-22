import os
from pathlib import Path

import numpy as np
import tesseract_robotics.tesseract_command_language as tesseract_command_language
import tesseract_robotics.tesseract_common as tesseract_common
import tesseract_robotics.tesseract_environment as tesseract_environment
import tesseract_robotics.tesseract_geometry as tesseract_geometry
import tesseract_robotics.tesseract_scene_graph as tesseract_scene_graph
import tesseract_robotics.tesseract_task_composer as tesseract_task_composer
from tesseract_robotics.tesseract_command_language import (
    AnyPoly_as_CompositeInstruction,
    AnyPoly_wrap_CompositeInstruction,
    CartesianWaypoint,
)
from tesseract_robotics.tesseract_command_language import (
    CartesianWaypointPoly_wrap_CartesianWaypoint as CartesianWaypointPoly,
)
from tesseract_robotics.tesseract_command_language import CompositeInstruction, MoveInstruction
from tesseract_robotics.tesseract_command_language import (
    MoveInstructionPoly_wrap_MoveInstruction as MoveInstructionPoly,
)
from tesseract_robotics.tesseract_command_language import StateWaypoint
from tesseract_robotics.tesseract_command_language import StateWaypointPoly_wrap_StateWaypoint as StateWaypointPoly
from tesseract_robotics.tesseract_command_language import toJointTrajectory
from tesseract_robotics.tesseract_common import (
    FilesystemPath,
    GeneralResourceLocator,
    Isometry3d,
    ManipulatorInfo,
    Quaterniond,
    Translation3d,
)
from tesseract_robotics.tesseract_environment import AddLinkCommand, Environment
from tesseract_robotics.tesseract_geometry import Box
from tesseract_robotics.tesseract_motion_planners import toToolpath
from tesseract_robotics.tesseract_motion_planners_trajopt import (
    ProfileDictionary_addProfile_TrajOptCompositeProfile,
    ProfileDictionary_addProfile_TrajOptPlanProfile,
    ProfileDictionary_addProfile_TrajOptSolverProfile,
    TrajOptCompositeProfile,
    TrajOptDefaultCompositeProfile,
    TrajOptDefaultPlanProfile,
    TrajOptDefaultSolverProfile,
    TrajOptPlanProfile,
    TrajOptSolverProfile,
)
from tesseract_robotics.tesseract_scene_graph import Collision, Joint, Link, Visual
from tesseract_robotics.tesseract_task_composer import PlanningTaskComposerProblem, PlanningTaskComposerProblemUPtr
from tesseract_robotics.tesseract_task_composer import (
    PlanningTaskComposerProblemUPtr_as_TaskComposerProblemUPtr as TaskComposerProblemUPtr,
)
from tesseract_robotics.tesseract_task_composer import (
    TaskComposerDataStorage,
    TaskComposerFuture,
    TaskComposerFutureUPtr,
    TaskComposerInput,
    TaskComposerPluginFactory,
)

# Run the following commands in the folder containing tesseract, tesseract_planning, tesseract_python
# Linux:
#   export TESSERACT_RESOURCE_PATH=`pwd`/tesseract
#   export TESSERACT_TASK_COMPOSER_DIR=`pwd`/tesseract_planning/tesseract_task_composer
# Windows:
#   set TESSERACT_RESOURCE_PATH=%CD%/tesseract
#   set TESSERACT_TASK_COMPOSER_DIR=%CD%/tesseract_planning/tesseract_task_composer
TESSERACT_SUPPORT_DIR = Path(os.environ["TESSERACT_RESOURCE_PATH"])
TESSERACT_TASK_COMPOSER_DIR = Path(os.environ["TESSERACT_TASK_COMPOSER_DIR"])
TASK_COMPOSER_PLUGIN_YAML = Path(r"config/task_composer_plugins.yaml")
TRAJOPT_DEFAULT_NAMESPACE = "TrajOptMotionPlannerTask"


def get_environment(url) -> Environment:
    """
    given a `url` load a URDF & SRDF and return an Enviornment and Manipulator instance and a
    list of joint names
    """
    locator = GeneralResourceLocator()
    env = Environment()
    # tesseract_support = os.environ["TESSERACT_SUPPORT_DIR"]
    urdf_path = locator.locateResource(f"{url}.urdf").getFilePath()
    srdf_path = locator.locateResource(f"{url}.srdf").getFilePath()

    urdf_path_str = FilesystemPath(urdf_path)
    srdf_path_str = FilesystemPath(srdf_path)

    assert env.init(urdf_path_str, srdf_path_str, locator)

    return env


class BasicCartesionExample:
    def __init__(self, env: Environment, visualize_results: bool = True, ifopt: bool = False, debug: bool = False):
        self.env = env
        self.visualize_results = visualize_results
        self.ifopt = ifopt
        self.debug = debug

        # self.update_env()

    def add_point_cloud(self) -> AddLinkCommand:
        """Create octomap and add it to the local environment"""

        visual = Visual()
        visual.origin = Isometry3d.Identity()
        # As a default set dtype to np.float64 for numpy arrays
        # https://github.com/tesseract-robotics/tesseract_python/issues/55
        visual.origin.setTranslation(np.array([1, 0, 0], dtype=np.float64))

        visual.geometry = Box(1, 1, 1)  # Octree does not work: missing constructors/templates

        collision = Collision()
        collision.origin = visual.origin
        collision.geometry = visual.geometry

        link_box = Link("octomap_attached")
        link_box.visual.push_back(visual)
        link_box.collision.push_back(collision)

        joint = Joint("joint_octomap_attached")
        joint.parent_link_name = "base_link"
        joint.child_link_name = link_box.getName()
        joint.type = tesseract_scene_graph.JointType_FIXED

        cmd = AddLinkCommand(link_box, joint)
        self.env.applyCommand(cmd)

        return cmd

    def run(self):
        # Create octomap and add it to the local environment
        cmd = self.add_point_cloud()

        # Set the robot initial state
        joint_names = ["joint_a1", "joint_a2", "joint_a3", "joint_a4", "joint_a5", "joint_a6", "joint_a7"]
        joint_position = np.array([-0.4, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0])

        if not self.env.isInitialized():
            # Prevent segmentation fault
            # https://github.com/tesseract-robotics/tesseract_python/issues/54
            raise RuntimeError("Environment not initialized")
        self.env.setState(joint_names, joint_position)

        if self.debug:
            tesseract_common.setLogLevel(tesseract_common.CONSOLE_BRIDGE_LOG_DEBUG)

        # Create Task Composer Plugin Factory
        config_path = str(TESSERACT_TASK_COMPOSER_DIR / TASK_COMPOSER_PLUGIN_YAML)
        # Incorect config version could result in errors:
        # https://github.com/tesseract-robotics/tesseract_python/issues/56
        config_path = tesseract_common.FilesystemPath(config_path)
        factory = TaskComposerPluginFactory(config_path)

        # Create program
        manip_info = ManipulatorInfo("manipulator", "tool0", "base_link")
        program = CompositeInstruction(
            "cartesian_program", tesseract_command_language.CompositeInstructionOrder_ORDERED, manip_info
        )

        # Start Joint Position for the program
        wp0 = StateWaypointPoly(StateWaypoint(joint_names, joint_position))
        start_instruction = MoveInstruction(
            wp0, tesseract_command_language.MoveInstructionType_FREESPACE, "freespace_profile"
        )
        start_instruction.setDescription("Start Instruction")

        # Create cartesian waypoint
        wp1 = CartesianWaypointPoly(
            CartesianWaypoint(Isometry3d.Identity() * Translation3d(0.5, -0.2, 0.62) * Quaterniond(0, 0, 1.0, 0))
        )
        wp2 = CartesianWaypointPoly(
            CartesianWaypoint(Isometry3d.Identity() * Translation3d(0.5, 0.3, 0.62) * Quaterniond(0, 0, 1.0, 0))
        )

        # Plan freespace from start
        plan_f0 = MoveInstruction(wp1, tesseract_command_language.MoveInstructionType_FREESPACE, "freespace_profile")
        plan_f0.setDescription("from_start_plan")

        # Plan linear move
        plan_c0 = MoveInstruction(wp2, tesseract_command_language.MoveInstructionType_LINEAR, "RASTER")

        # Plan freespace to end
        plan_f1 = MoveInstruction(wp0, tesseract_command_language.MoveInstructionType_FREESPACE, "freespace_profile")
        plan_f1.setDescription("to_end_plan")

        # Add instructions to program
        program.appendMoveInstruction(MoveInstructionPoly(start_instruction))
        program.appendMoveInstruction(MoveInstructionPoly(plan_f0))
        program.appendMoveInstruction(MoveInstructionPoly(plan_c0))
        program.appendMoveInstruction(MoveInstructionPoly(plan_f1))

        # Print diagnosics
        program._print("Program: ")
        # CONSOLE_BRIDGE_logInform("basic cartesian plan example")

        # Create executor
        executor = factory.createTaskComposerExecutor("TaskflowExecutor")

        # Create profile dictionary
        profiles = tesseract_command_language.ProfileDictionary()

        if self.ifopt:
            # I am only able to find the base, descartes, ompl, simpl, and trajobt motion planners under
            # tesseract_robotics.tesseract_motion_planners_*. It seems trajopt_ifopt has not been ported yet.
            raise NotImplementedError("TrajOptIfopt has not been ported to Python yet.")

        else:
            composite_profile = TrajOptDefaultCompositeProfile()
            composite_profile.collision_cost_config.enabled = True
            composite_profile.collision_constraint_config.enabled = True
            composite_profile.smooth_velocities = True
            composite_profile.smooth_accelerations = False
            composite_profile.smooth_jerks = False
            composite_profile.velocity_coeff = np.array([1], dtype=np.float64)
            ProfileDictionary_addProfile_TrajOptCompositeProfile(
                profile_dictionary=profiles,
                ns=TRAJOPT_DEFAULT_NAMESPACE,
                profile_name="cartesian_program",
                profile=composite_profile,
            )

            plan_profile = TrajOptDefaultPlanProfile()
            plan_profile.cartesian_coeff = np.array([1, 1, 1, 1, 1, 1], dtype=np.float64)
            plan_profile.joint_coeff = np.array([1, 1, 1, 1, 1, 1, 1], dtype=np.float64)
            ProfileDictionary_addProfile_TrajOptPlanProfile(
                profile_dictionary=profiles,
                ns=TRAJOPT_DEFAULT_NAMESPACE,
                profile_name="RASTER",
                profile=plan_profile,
            )
            ProfileDictionary_addProfile_TrajOptPlanProfile(
                profile_dictionary=profiles,
                ns=TRAJOPT_DEFAULT_NAMESPACE,
                profile_name="freespace_profile",
                profile=plan_profile,
            )

        # Create task
        task_name = "TrajOptIfoptPipeline" if self.ifopt else "TrajOptPipeline"
        # The next line will create and error if the loaded config is incorrect: https://github.com/tesseract-robotics/tesseract_python/issues/52
        task = factory.createTaskComposerNode(task_name)
        input_key = task.getInputKeys()[0]
        output_key = task.getOutputKeys()[0]

        # Create Task Composer problem
        task_data = TaskComposerDataStorage()
        task_data.setData(input_key, AnyPoly_wrap_CompositeInstruction(program))
        problem = TaskComposerProblemUPtr(PlanningTaskComposerProblemUPtr.make_unique(self.env, task_data, profiles))
        task_composer_input = TaskComposerInput(problem)

        if self.visualize_results:
            input("Hit Enter to solve for trajectory")

        # Solve task
        stopwatch = tesseract_common.Timer()
        stopwatch.start()
        future = executor.run(task.get(), task_composer_input)
        future.wait()

        stopwatch.stop()
        print(f"Planning took {stopwatch.elapsedSeconds()} seconds.")

        # Plot Process Trajectory
        if self.visualize_results:
            # TaskComposerFuture does not expose the `context` so have to work around this
            composite_instuction = AnyPoly_as_CompositeInstruction(task_data.getData(output_key))
            toolpath = toToolpath(composite_instuction, self.env)
            trajectory = toJointTrajectory(composite_instuction)
            # state_solver = self.env.getStateSolver()
            # plotMarker(ToolpathMarger(toolpath))
            # plotTrajectory(trajectory, state_solver)

        print("Final trjectory is collision free")
        # return future.context.isSuccessful() # Context is not shared


def main():
    env = get_environment("package://tesseract_support/urdf/lbr_iiwa_14_r820")
    x = BasicCartesionExample(env, debug=True)
    x.run()

    input("Press enter to exit")


if __name__ == "__main__":
    main()
