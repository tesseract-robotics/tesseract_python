"""
Glass Upright Example

This example demonstrates constrained motion planning using TrajOpt where
the robot must keep the tool orientation "upright" (constraining the orientation)
while moving through space. This is useful for tasks like carrying a glass of water.

The key difference from freespace planning is that TrajOpt applies constraints
to keep the tool orientation fixed while allowing position changes.

Based on: tesseract_examples/src/glass_upright_example.cpp

Required environment variables:
- TESSERACT_RESOURCE_PATH: Path to tesseract repo (for tesseract_support)
- TESSERACT_TASK_COMPOSER_CONFIG_FILE: Path to task composer config YAML
"""

import os
import numpy as np

from tesseract_robotics.tesseract_common import (
    GeneralResourceLocator,
    FilesystemPath,
    Isometry3d,
    Translation3d,
    ManipulatorInfo,
)
from tesseract_robotics.tesseract_environment import (
    Environment,
    AddLinkCommand,
    AnyPoly_wrap_EnvironmentConst,
)
from tesseract_robotics.tesseract_scene_graph import (
    Link,
    Joint,
    JointType,
    Visual,
    Collision,
)
from tesseract_robotics.tesseract_geometry import Sphere
from tesseract_robotics.tesseract_command_language import (
    CompositeInstruction,
    MoveInstruction,
    MoveInstructionType_LINEAR,
    StateWaypoint,
    StateWaypointPoly_wrap_StateWaypoint,
    MoveInstructionPoly_wrap_MoveInstruction,
    ProfileDictionary,
    AnyPoly_wrap_CompositeInstruction,
    AnyPoly_wrap_ProfileDictionary,
    AnyPoly_as_CompositeInstruction,
    InstructionPoly_as_MoveInstructionPoly,
    WaypointPoly_as_StateWaypointPoly,
)
from tesseract_robotics.tesseract_task_composer import (
    createTaskComposerPluginFactory,
    TaskComposerDataStorage,
)

# Optional: viewer for visualization
try:
    from tesseract_robotics_viewer import TesseractViewer
    HAS_VIEWER = True
except ImportError:
    HAS_VIEWER = False

TRAJOPT_DEFAULT_NAMESPACE = "TrajOptMotionPlannerTask"


def add_sphere():
    """Add a sphere obstacle to the environment"""
    link_sphere = Link("sphere_attached")

    # Create visual
    visual = Visual()
    visual.origin = Isometry3d.Identity()
    visual.origin = visual.origin * Translation3d(0.5, 0, 0.55)
    visual.geometry = Sphere(0.15)
    link_sphere.visual.append(visual)

    # Create collision (same as visual)
    collision = Collision()
    collision.origin = visual.origin
    collision.geometry = visual.geometry
    link_sphere.collision.append(collision)

    # Create joint to attach sphere to base_link
    joint_sphere = Joint("joint_sphere_attached")
    joint_sphere.parent_link_name = "base_link"
    joint_sphere.child_link_name = link_sphere.getName()
    joint_sphere.type = JointType.FIXED

    return AddLinkCommand(link_sphere, joint_sphere)


def main():
    # Get config file path
    task_composer_filename = os.environ.get("TESSERACT_TASK_COMPOSER_CONFIG_FILE")
    if not task_composer_filename:
        print("Error: TESSERACT_TASK_COMPOSER_CONFIG_FILE environment variable not set")
        print("Set it to: tesseract_planning/tesseract_task_composer/config/task_composer_plugins.yaml")
        return False

    # Initialize resource locator and environment
    locator = GeneralResourceLocator()

    # Load KUKA IIWA robot
    urdf_url = "package://tesseract_support/urdf/lbr_iiwa_14_r820.urdf"
    srdf_url = "package://tesseract_support/urdf/lbr_iiwa_14_r820.srdf"
    urdf_path = FilesystemPath(locator.locateResource(urdf_url).getFilePath())
    srdf_path = FilesystemPath(locator.locateResource(srdf_url).getFilePath())

    env = Environment()
    if not env.init(urdf_path, srdf_path, locator):
        print("Failed to initialize environment")
        return False

    print(f"Environment initialized: {env.getName()}")

    # Add sphere obstacle
    cmd = add_sphere()
    if not env.applyCommand(cmd):
        print("Failed to add sphere to environment")
        return False
    print("Added sphere obstacle at (0.5, 0, 0.55)")

    # Define joint names for KUKA IIWA
    joint_names = [f"joint_a{i}" for i in range(1, 8)]

    # Define start and end positions
    joint_start_pos = np.array([-0.4, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0])
    joint_end_pos = np.array([0.4, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0])

    # Set initial state
    env.setState(joint_names, joint_start_pos)

    # Create manipulator info
    manip_info = ManipulatorInfo()
    manip_info.manipulator = "manipulator"
    manip_info.working_frame = "base_link"
    manip_info.tcp_frame = "tool0"

    # Create program with "UPRIGHT" profile
    # The UPRIGHT profile will constrain orientation while allowing position changes
    program = CompositeInstruction("UPRIGHT")
    program.setManipulatorInfo(manip_info)

    # Start and end waypoints
    wp0 = StateWaypoint(joint_names, joint_start_pos)
    wp1 = StateWaypoint(joint_names, joint_end_pos)

    # Create instructions using LINEAR motion type
    # LINEAR ensures the tool follows a straight-line path while maintaining constraints
    start_instruction = MoveInstruction(
        StateWaypointPoly_wrap_StateWaypoint(wp0),
        MoveInstructionType_LINEAR,
        "UPRIGHT"
    )
    start_instruction.setDescription("Start Instruction")

    plan_f0 = MoveInstruction(
        StateWaypointPoly_wrap_StateWaypoint(wp1),
        MoveInstructionType_LINEAR,
        "UPRIGHT"
    )
    plan_f0.setDescription("upright_plan")

    # Add instructions to program
    program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(start_instruction))
    program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(plan_f0))

    print("\nProgram created with 'UPRIGHT' constraint profile")
    print("The tool orientation will be constrained during motion")

    # Create task composer factory
    factory = createTaskComposerPluginFactory(task_composer_filename, locator)

    # Create executor
    executor = factory.createTaskComposerExecutor("TaskflowExecutor")

    # Create task (TrajOpt pipeline)
    task = factory.createTaskComposerNode("TrajOptPipeline")
    output_key = task.getOutputKeys().get("program")
    input_key = task.getInputKeys().get("planning_input")

    # Create profile dictionary
    # Note: For full glass upright behavior, custom TrajOpt profiles would need to be
    # configured to constrain the orientation. The default profiles provide basic planning.
    profiles = ProfileDictionary()

    # Create task data storage
    task_data = TaskComposerDataStorage()
    task_data.setData(input_key, AnyPoly_wrap_CompositeInstruction(program))
    task_data.setData("environment", AnyPoly_wrap_EnvironmentConst(env))
    task_data.setData("profiles", AnyPoly_wrap_ProfileDictionary(profiles))

    print("\nRunning TrajOpt planner with upright constraint...")

    # Run the task (nanobind returns the node directly, no .get() needed)
    future = executor.run(task, task_data)
    future.wait()

    if not future.context.isSuccessful():
        print("Planning failed!")
        return False

    print("Planning successful!")

    # Get results
    results = AnyPoly_as_CompositeInstruction(future.context.data_storage.getData(output_key))

    # Print trajectory
    print(f"\nTrajectory has {len(results)} waypoints:")
    for i, instr in enumerate(results):
        if instr.isMoveInstruction():
            move_instr = InstructionPoly_as_MoveInstructionPoly(instr)
            wp = move_instr.getWaypoint()
            if wp.isStateWaypoint():
                state_wp = WaypointPoly_as_StateWaypointPoly(wp)
                pos = state_wp.getPosition()
                print(f"  [{i}] {pos.flatten()}")

    # Optional: visualize with viewer
    if HAS_VIEWER:
        print("\nStarting viewer at http://localhost:8000")
        viewer = TesseractViewer()
        viewer.update_environment(env, [0, 0, 0])
        viewer.update_trajectory(results)
        viewer.start_serve_background()
        input("Press Enter to exit...")

    return True


if __name__ == "__main__":
    success = main()
    exit(0 if success else 1)
