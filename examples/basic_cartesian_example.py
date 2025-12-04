"""
Basic Cartesian Example

This example demonstrates Cartesian motion planning using TrajOpt with the KUKA IIWA robot.
It creates a point cloud obstacle and plans a trajectory that includes both freespace and
linear Cartesian moves.

Based on: tesseract_examples/src/basic_cartesian_example.cpp

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
    Quaterniond,
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
from tesseract_robotics.tesseract_geometry import Box
from tesseract_robotics.tesseract_command_language import (
    CompositeInstruction,
    MoveInstruction,
    MoveInstructionType_FREESPACE,
    MoveInstructionType_LINEAR,
    StateWaypoint,
    CartesianWaypoint,
    StateWaypointPoly_wrap_StateWaypoint,
    CartesianWaypointPoly_wrap_CartesianWaypoint,
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


def add_box_obstacle():
    """Add a box obstacle to represent a simplified point cloud"""
    link_box = Link("box_obstacle")

    # Create visual - a box to represent an obstacle
    visual = Visual()
    visual.origin = Isometry3d.Identity()
    visual.origin = visual.origin * Translation3d(1.0, 0, 0)  # Position at (1, 0, 0)
    visual.geometry = Box(0.5, 0.5, 0.5)  # 0.5m cube
    link_box.visual.append(visual)

    # Create collision (same as visual)
    collision = Collision()
    collision.origin = visual.origin
    collision.geometry = visual.geometry
    link_box.collision.append(collision)

    # Create joint to attach box to base_link
    joint_box = Joint("joint_box_obstacle")
    joint_box.parent_link_name = "base_link"
    joint_box.child_link_name = link_box.getName()
    joint_box.type = JointType.FIXED

    return AddLinkCommand(link_box, joint_box)


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

    print(f"Environment initialized with robot: {env.getName()}")
    print(f"Root link: {env.getRootLinkName()}")

    # Add box obstacle (simplified version of point cloud)
    cmd = add_box_obstacle()
    if not env.applyCommand(cmd):
        print("Failed to add obstacle to environment")
        return False
    print("Added box obstacle at (1.0, 0, 0)")

    # Define joint names for KUKA IIWA
    joint_names = [f"joint_a{i}" for i in range(1, 8)]

    # Define initial joint position
    joint_pos = np.array([-0.4, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0])

    # Set initial state
    env.setState(joint_names, joint_pos)

    # Create manipulator info
    manip_info = ManipulatorInfo()
    manip_info.manipulator = "manipulator"
    manip_info.working_frame = "base_link"
    manip_info.tcp_frame = "tool0"

    # Create program
    program = CompositeInstruction("cartesian_program")
    program.setManipulatorInfo(manip_info)

    # Start waypoint (joint position)
    wp0 = StateWaypoint(joint_names, joint_pos)

    # Create Cartesian waypoints (tool poses)
    # wp1: First Cartesian target
    wp1 = CartesianWaypoint(
        Isometry3d.Identity() * Translation3d(0.5, -0.2, 0.62) * Quaterniond(0, 0, 1.0, 0)
    )

    # wp2: Second Cartesian target (linear move)
    wp2 = CartesianWaypoint(
        Isometry3d.Identity() * Translation3d(0.5, 0.3, 0.62) * Quaterniond(0, 0, 1.0, 0)
    )

    # Create instructions
    # Start instruction
    start_instruction = MoveInstruction(
        StateWaypointPoly_wrap_StateWaypoint(wp0),
        MoveInstructionType_FREESPACE,
        "freespace_profile"
    )
    start_instruction.setDescription("Start Instruction")

    # Freespace move to first Cartesian target
    plan_f0 = MoveInstruction(
        CartesianWaypointPoly_wrap_CartesianWaypoint(wp1),
        MoveInstructionType_FREESPACE,
        "freespace_profile"
    )
    plan_f0.setDescription("from_start_plan")

    # Linear move to second Cartesian target
    plan_c0 = MoveInstruction(
        CartesianWaypointPoly_wrap_CartesianWaypoint(wp2),
        MoveInstructionType_LINEAR,
        "RASTER"
    )
    plan_c0.setDescription("linear_move")

    # Freespace move back to start
    plan_f1 = MoveInstruction(
        StateWaypointPoly_wrap_StateWaypoint(wp0),
        MoveInstructionType_FREESPACE,
        "freespace_profile"
    )
    plan_f1.setDescription("to_end_plan")

    # Add instructions to program
    program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(start_instruction))
    program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(plan_f0))
    program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(plan_c0))
    program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(plan_f1))

    print("\nProgram created with TrajOpt Cartesian planning")
    print("  - Freespace to Cartesian wp1")
    print("  - Linear to Cartesian wp2")
    print("  - Freespace back to start")

    # Create task composer factory
    factory = createTaskComposerPluginFactory(task_composer_filename, locator)

    # Create executor
    executor = factory.createTaskComposerExecutor("TaskflowExecutor")

    # Create task (TrajOpt pipeline)
    task = factory.createTaskComposerNode("TrajOptPipeline")
    output_key = task.getOutputKeys().get("program")
    input_key = task.getInputKeys().get("planning_input")

    # Create profile dictionary (using defaults)
    profiles = ProfileDictionary()

    # Create task data storage
    task_data = TaskComposerDataStorage()
    task_data.setData(input_key, AnyPoly_wrap_CompositeInstruction(program))
    task_data.setData("environment", AnyPoly_wrap_EnvironmentConst(env))
    task_data.setData("profiles", AnyPoly_wrap_ProfileDictionary(profiles))

    print("\nRunning TrajOpt planner...")

    # Run the task
    future = executor.run(task.get(), task_data)
    future.wait()

    if not future.context.isSuccessful():
        print("Planning failed!")
        return False

    print("Planning successful!")

    # Get results
    results = AnyPoly_as_CompositeInstruction(future.context.data_storage.getData(output_key))

    # Print trajectory summary
    print(f"\nTrajectory has {len(results)} waypoints")

    # Count waypoint types
    n_state = 0
    for instr in results:
        if instr.isMoveInstruction():
            move_instr = InstructionPoly_as_MoveInstructionPoly(instr)
            wp = move_instr.getWaypoint()
            if wp.isStateWaypoint():
                n_state += 1
    print(f"  State waypoints: {n_state}")

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
