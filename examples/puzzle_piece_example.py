"""
Puzzle Piece Example

This example demonstrates Cartesian path planning using TrajOpt for following a
toolpath defined in a CSV file. The robot follows a series of poses along a
puzzle piece edge.

Based on: tesseract_examples/src/puzzle_piece_example.cpp

Required environment variables:
- TESSERACT_RESOURCE_PATH: Path to tesseract repo (for tesseract_support)
- TESSERACT_TASK_COMPOSER_CONFIG_FILE: Path to task composer config YAML

Required files:
- tesseract_support/urdf/puzzle_bent.csv: CSV file with toolpath poses
"""

import os
import sys
import gc
import csv
import numpy as np

from tesseract_robotics.tesseract_common import (
    GeneralResourceLocator,
    FilesystemPath,
    Isometry3d,
    ManipulatorInfo,
)
from tesseract_robotics.tesseract_environment import (
    Environment,
    AnyPoly_wrap_EnvironmentConst,
)
from tesseract_robotics.tesseract_command_language import (
    CompositeInstruction,
    MoveInstruction,
    MoveInstructionType_LINEAR,
    CartesianWaypoint,
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

# Optional: viewer for visualization (disabled in pytest/headless mode)
try:
    from tesseract_robotics_viewer import TesseractViewer
    HAS_VIEWER = os.environ.get("TESSERACT_HEADLESS", "0") != "1" and "pytest" not in sys.modules
except ImportError:
    HAS_VIEWER = False

TRAJOPT_DEFAULT_NAMESPACE = "TrajOptMotionPlannerTask"


def make_puzzle_tool_poses(locator):
    """
    Load toolpath poses from the puzzle_bent.csv file.

    The CSV contains position (x, y, z) and normal direction (i, j, k) for each pose.
    We construct a full orientation from the normal using the position for the x-axis.
    """
    # Locate the CSV file
    resource = locator.locateResource("package://tesseract_support/urdf/puzzle_bent.csv")
    csv_path = resource.getFilePath()

    poses = []

    with open(csv_path, 'r') as f:
        reader = csv.reader(f)
        for lnum, row in enumerate(reader):
            # Skip header rows (first 2 lines)
            if lnum < 2:
                continue

            if len(row) < 7:
                continue

            # Parse values: skip first column (point number), then x, y, z, i, j, k
            try:
                x, y, z = float(row[1]), float(row[2]), float(row[3])
                i, j, k = float(row[4]), float(row[5]), float(row[6])
            except (ValueError, IndexError):
                continue

            # Convert from mm to meters
            pos = np.array([x, y, z]) / 1000.0

            # Normalize the normal vector
            norm = np.array([i, j, k])
            norm = norm / np.linalg.norm(norm)

            # Compute frame from normal (z-axis)
            temp_x = -pos / np.linalg.norm(pos) if np.linalg.norm(pos) > 1e-6 else np.array([1, 0, 0])
            y_axis = np.cross(norm, temp_x)
            y_axis = y_axis / np.linalg.norm(y_axis)
            x_axis = np.cross(y_axis, norm)
            x_axis = x_axis / np.linalg.norm(x_axis)

            # Build rotation matrix
            rot = np.eye(4)
            rot[:3, 0] = x_axis
            rot[:3, 1] = y_axis
            rot[:3, 2] = norm
            rot[:3, 3] = pos

            # Convert to Isometry3d (default constructor creates identity)
            pose = Isometry3d(rot)

            poses.append(pose)

    return poses


def main():
    # Get config file path
    task_composer_filename = os.environ.get("TESSERACT_TASK_COMPOSER_CONFIG_FILE")
    if not task_composer_filename:
        print("Error: TESSERACT_TASK_COMPOSER_CONFIG_FILE environment variable not set")
        print("Set it to: tesseract_planning/tesseract_task_composer/config/task_composer_plugins.yaml")
        return False

    # Initialize resource locator and environment
    locator = GeneralResourceLocator()

    # Load KUKA IIWA robot (puzzle piece example uses same robot)
    urdf_url = "package://tesseract_support/urdf/lbr_iiwa_14_r820.urdf"
    srdf_url = "package://tesseract_support/urdf/lbr_iiwa_14_r820.srdf"
    urdf_path = FilesystemPath(locator.locateResource(urdf_url).getFilePath())
    srdf_path = FilesystemPath(locator.locateResource(srdf_url).getFilePath())

    env = Environment()
    if not env.init(urdf_path, srdf_path, locator):
        print("Failed to initialize environment")
        return False

    print(f"Environment initialized with robot: {env.getName()}")

    # Define joint names for KUKA IIWA
    joint_names = [f"joint_a{i}" for i in range(1, 8)]

    # Define initial joint position
    joint_pos = np.array([-0.785398, 0.4, 0.0, -1.9, 0.0, 1.0, 0.0])

    # Set initial state
    env.setState(joint_names, joint_pos)

    # Load tool poses from CSV
    print("\nLoading toolpath from puzzle_bent.csv...")
    try:
        tool_poses = make_puzzle_tool_poses(locator)
    except Exception as e:
        print(f"Failed to load toolpath: {e}")
        return False

    print(f"Loaded {len(tool_poses)} tool poses")

    if len(tool_poses) == 0:
        print("No poses loaded from CSV!")
        return False

    # Create manipulator info
    # Note: The puzzle piece example uses a custom frame "grinder_frame" as TCP
    # and "part" as working frame. For simplicity we use standard frames.
    manip_info = ManipulatorInfo()
    manip_info.manipulator = "manipulator"
    manip_info.working_frame = "base_link"
    manip_info.tcp_frame = "tool0"

    # Create program
    program = CompositeInstruction("DEFAULT")
    program.setManipulatorInfo(manip_info)

    # Add Cartesian waypoints from toolpath
    for i, pose in enumerate(tool_poses):
        wp = CartesianWaypoint(pose)
        plan_instruction = MoveInstruction(
            CartesianWaypointPoly_wrap_CartesianWaypoint(wp),
            MoveInstructionType_LINEAR,
            "CARTESIAN"
        )
        plan_instruction.setDescription(f"waypoint_{i}")
        program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(plan_instruction))

    print(f"\nProgram created with {len(tool_poses)} Cartesian waypoints")

    # Create task composer factory
    factory = createTaskComposerPluginFactory(task_composer_filename, locator)

    # Create executor
    executor = factory.createTaskComposerExecutor("TaskflowExecutor")

    # Create task (TrajOpt pipeline for Cartesian path)
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

    print("\nRunning TrajOpt planner for puzzle piece path...")
    print("(This may take a while for many waypoints)")

    # Run the task (nanobind returns the node directly, no .get() needed)
    future = executor.run(task, task_data)
    future.wait()

    if not future.context.isSuccessful():
        print("Planning failed!")
        return False

    print("Planning successful!")

    # Get results
    results = AnyPoly_as_CompositeInstruction(future.context.data_storage.getData(output_key))

    # Print trajectory summary
    print(f"\nTrajectory has {len(results)} waypoints")

    # Optional: visualize with viewer
    if HAS_VIEWER:
        print("\nStarting viewer at http://localhost:8000")
        viewer = TesseractViewer()
        viewer.update_environment(env, [0, 0, 0])
        viewer.update_trajectory(results)
        viewer.start_serve_background()
        input("Press Enter to exit...")

    # Explicit cleanup to prevent segfault at interpreter shutdown
    # TaskComposer objects must be destroyed in proper order
    del future
    del task_data
    del task
    del executor
    del factory
    gc.collect()

    return True


if __name__ == "__main__":
    success = main()
    exit(0 if success else 1)
