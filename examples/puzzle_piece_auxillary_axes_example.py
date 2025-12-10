"""
Puzzle Piece Auxiliary Axes Example

This example demonstrates Cartesian path planning using TrajOpt with auxiliary axes.
The robot (KUKA IIWA 7-DOF) is combined with a 2-DOF positioner (aux1, aux2) for
a total of 9 degrees of freedom. The planner can move both the robot and the
auxiliary axes to follow the toolpath.

Based on: tesseract_examples/src/puzzle_piece_auxillary_axes_example.cpp

Required environment variables:
- TESSERACT_RESOURCE_PATH: Path to tesseract repo (for tesseract_support)
- TESSERACT_TASK_COMPOSER_CONFIG_FILE: Path to task composer config YAML
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
)
from tesseract_robotics.tesseract_task_composer import (
    createTaskComposerPluginFactory,
    TaskComposerDataStorage,
)
from tesseract_robotics.tesseract_motion_planners import assignCurrentStateAsSeed
from tesseract_robotics.tesseract_motion_planners_trajopt import (
    TrajOptDefaultPlanProfile,
    TrajOptDefaultCompositeProfile,
    CollisionEvaluatorType,
    ProfileDictionary_addTrajOptPlanProfile,
    ProfileDictionary_addTrajOptCompositeProfile,
)

# Viewer (skip in pytest)
TesseractViewer = None
if "pytest" not in sys.modules:
    try:
        from tesseract_robotics_viewer import TesseractViewer
    except ImportError:
        pass

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

            # Convert to Isometry3d
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

    # Load puzzle piece workcell (KUKA + grinder + puzzle piece part + auxiliary axes)
    urdf_url = "package://tesseract_support/urdf/puzzle_piece_workcell.urdf"
    srdf_url = "package://tesseract_support/urdf/puzzle_piece_workcell.srdf"
    urdf_path = FilesystemPath(locator.locateResource(urdf_url).getFilePath())
    srdf_path = FilesystemPath(locator.locateResource(srdf_url).getFilePath())

    env = Environment()
    if not env.init(urdf_path, srdf_path, locator):
        print("Failed to initialize environment")
        return False

    print(f"Environment initialized: {env.getName()}")

    # Define joint names for KUKA IIWA (7) + auxiliary axes (2) = 9 DOF
    joint_names = [
        "joint_a1", "joint_a2", "joint_a3", "joint_a4",
        "joint_a5", "joint_a6", "joint_a7",
        "joint_aux1", "joint_aux2"
    ]

    # Define initial joint position (9 values)
    joint_pos = np.array([
        -0.785398,  # joint_a1
        0.4,        # joint_a2
        0.0,        # joint_a3
        -1.9,       # joint_a4
        0.0,        # joint_a5
        1.0,        # joint_a6
        0.0,        # joint_a7
        0.0,        # joint_aux1 (positioner rotation around Z)
        0.0         # joint_aux2 (positioner tilt)
    ])

    # Set initial state
    env.setState(joint_names, joint_pos)

    # Load tool poses from CSV
    try:
        tool_poses = make_puzzle_tool_poses(locator)
    except Exception as e:
        print(f"Failed to load toolpath: {e}")
        return False

    print(f"Loaded {len(tool_poses)} tool poses from CSV")

    if len(tool_poses) == 0:
        print("No poses loaded from CSV!")
        return False

    # Create manipulator info - use manipulator_aux group (9 DOF)
    manip_info = ManipulatorInfo()
    manip_info.manipulator = "manipulator_aux"  # 9 DOF group including auxiliary axes
    manip_info.working_frame = "part"
    manip_info.tcp_frame = "grinder_frame"

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

    print(f"Program has {len(tool_poses)} Cartesian waypoints")

    # Assign current state as seed for CartesianWaypoints
    assignCurrentStateAsSeed(program, env)

    # Create task composer factory
    factory = createTaskComposerPluginFactory(task_composer_filename, locator)

    # Create executor
    executor = factory.createTaskComposerExecutor("TaskflowExecutor")

    # Create task (TrajOpt pipeline for Cartesian path)
    task = factory.createTaskComposerNode("TrajOptPipeline")
    output_key = task.getOutputKeys().get("program")
    input_key = task.getInputKeys().get("planning_input")

    # Create profile dictionary with custom TrajOpt profiles
    profiles = ProfileDictionary()

    # Configure TrajOpt plan profile
    trajopt_plan_profile = TrajOptDefaultPlanProfile()
    trajopt_plan_profile.joint_cost_config.enabled = False
    trajopt_plan_profile.cartesian_cost_config.enabled = False
    trajopt_plan_profile.cartesian_constraint_config.enabled = True
    # Coefficients: [x, y, z, rx, ry, rz] - lower weight on rotation around z (tool axis)
    trajopt_plan_profile.cartesian_constraint_config.coeff = np.array([5.0, 5.0, 5.0, 2.0, 2.0, 0.0])

    # Configure TrajOpt composite profile
    trajopt_composite_profile = TrajOptDefaultCompositeProfile()
    trajopt_composite_profile.collision_constraint_config.enabled = False
    trajopt_composite_profile.collision_cost_config.enabled = True
    trajopt_composite_profile.collision_cost_config.safety_margin = 0.025
    trajopt_composite_profile.collision_cost_config.type = CollisionEvaluatorType.SINGLE_TIMESTEP
    trajopt_composite_profile.collision_cost_config.coeff = 1.0

    # Add profiles to dictionary
    ProfileDictionary_addTrajOptPlanProfile(profiles, TRAJOPT_DEFAULT_NAMESPACE, "CARTESIAN", trajopt_plan_profile)
    ProfileDictionary_addTrajOptCompositeProfile(profiles, TRAJOPT_DEFAULT_NAMESPACE, "DEFAULT", trajopt_composite_profile)

    # Create task data storage
    task_data = TaskComposerDataStorage()
    task_data.setData(input_key, AnyPoly_wrap_CompositeInstruction(program))
    task_data.setData("environment", AnyPoly_wrap_EnvironmentConst(env))
    task_data.setData("profiles", AnyPoly_wrap_ProfileDictionary(profiles))

    print("Running TrajOpt planner with 9 DOF (7 arm + 2 auxiliary axes)...")

    # Run the task
    future = executor.run(task, task_data)
    future.wait()

    if not future.context.isSuccessful():
        print("Planning failed!")
        return False

    print("Planning successful!")

    # Get results
    results = AnyPoly_as_CompositeInstruction(future.context.data_storage.getData(output_key))

    # Print trajectory summary
    print(f"Trajectory has {len(results)} waypoints")

    # Optional: visualize with viewer
    if TesseractViewer is not None:
        print("\nStarting viewer at http://localhost:8000")
        viewer = TesseractViewer()
        viewer.update_environment(env, [0, 0, 0])
        viewer.update_trajectory(results)
        viewer.start_serve_background()
        input("Press Enter to exit...")

    # Explicit cleanup to prevent segfault at interpreter shutdown
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
