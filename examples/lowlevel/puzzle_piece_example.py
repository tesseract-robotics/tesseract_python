"""
Puzzle Piece Example

Cartesian path planning using TrajOpt to follow a toolpath from a CSV file.
"""

import sys
import csv
import numpy as np

from tesseract_robotics.planning import (
    Robot,
    MotionProgram,
    CartesianTarget,
    Pose,
    TaskComposer,
)
from tesseract_robotics.tesseract_command_language import ProfileDictionary
from tesseract_robotics.tesseract_motion_planners_trajopt import (
    TrajOptDefaultPlanProfile,
    TrajOptDefaultCompositeProfile,
    CollisionEvaluatorType,
    ProfileDictionary_addTrajOptPlanProfile,
    ProfileDictionary_addTrajOptCompositeProfile,
)

TesseractViewer = None
if "pytest" not in sys.modules:
    try:
        from tesseract_robotics_viewer import TesseractViewer
    except ImportError:
        pass

TRAJOPT_DEFAULT_NAMESPACE = "TrajOptMotionPlannerTask"


def make_puzzle_tool_poses(robot):
    """
    Load toolpath poses from the puzzle_bent.csv file.

    The CSV contains position (x, y, z) and normal direction (i, j, k) for each pose.
    We construct a full orientation from the normal using the position for the x-axis.
    """
    # Locate the CSV file
    resource = robot.locator.locateResource("package://tesseract_support/urdf/puzzle_bent.csv")
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

            # Build pose from rotation and position
            rot = np.column_stack([x_axis, y_axis, norm])
            poses.append(Pose.from_matrix_position(rot, pos))

    return poses


def main():
    # Load puzzle piece workcell
    robot = Robot.from_urdf(
        "package://tesseract_support/urdf/puzzle_piece_workcell.urdf",
        "package://tesseract_support/urdf/puzzle_piece_workcell.srdf"
    )
    print(f"Loaded robot with {len(robot.get_link_names())} links")

    # Define joint names for KUKA IIWA
    joint_names = [f"joint_a{i}" for i in range(1, 8)]

    # Define initial joint position
    joint_pos = np.array([-0.785398, 0.4, 0.0, -1.9, 0.0, 1.0, 0.0])

    # Set initial state
    robot.set_joints(joint_pos, joint_names=joint_names)

    # Load tool poses from CSV
    try:
        tool_poses = make_puzzle_tool_poses(robot)
    except Exception as e:
        print(f"Failed to load toolpath: {e}")
        return False

    print(f"Loaded {len(tool_poses)} tool poses from CSV")

    if len(tool_poses) == 0:
        print("No poses loaded from CSV!")
        return False

    # Build motion program with all waypoints
    program = MotionProgram("manipulator", tcp_frame="grinder_frame", working_frame="part")
    program.set_joint_names(joint_names)

    for pose in tool_poses:
        program.linear_to(CartesianTarget(pose, profile="CARTESIAN"))

    print(f"Program has {len(program)} Cartesian waypoints")

    # Create custom TrajOpt profiles for Cartesian path following
    profiles = ProfileDictionary()

    # Configure TrajOpt plan profile
    trajopt_plan_profile = TrajOptDefaultPlanProfile()
    trajopt_plan_profile.joint_cost_config.enabled = False
    trajopt_plan_profile.cartesian_cost_config.enabled = False
    trajopt_plan_profile.cartesian_constraint_config.enabled = True
    trajopt_plan_profile.cartesian_constraint_config.coeff = np.array([10.0, 10.0, 10.0, 10.0, 10.0, 0.0])

    # Configure TrajOpt composite profile
    trajopt_composite_profile = TrajOptDefaultCompositeProfile()
    trajopt_composite_profile.collision_constraint_config.enabled = False
    trajopt_composite_profile.collision_cost_config.enabled = True
    trajopt_composite_profile.collision_cost_config.safety_margin = 0.025
    trajopt_composite_profile.collision_cost_config.type = CollisionEvaluatorType.SINGLE_TIMESTEP
    trajopt_composite_profile.collision_cost_config.coeff = 20.0

    # Add profiles to dictionary
    ProfileDictionary_addTrajOptPlanProfile(profiles, TRAJOPT_DEFAULT_NAMESPACE, "CARTESIAN", trajopt_plan_profile)
    ProfileDictionary_addTrajOptCompositeProfile(profiles, TRAJOPT_DEFAULT_NAMESPACE, "DEFAULT", trajopt_composite_profile)

    print("Running TrajOpt planner...")

    # Plan using TaskComposer (auto-seeds Cartesian waypoints)
    composer = TaskComposer.from_config()
    result = composer.plan(robot, program, pipeline="TrajOptPipeline", profiles=profiles)

    if not result.successful:
        print(f"Planning failed: {result.message}")
        return False

    print("Planning successful!")
    print(f"Trajectory has {len(result)} waypoints")

    # Optional: visualize with viewer
    if TesseractViewer is not None:
        print("\nStarting viewer at http://localhost:8000")
        viewer = TesseractViewer()
        viewer.update_environment(robot.env, [0, 0, 0])
        viewer.update_trajectory(result.raw_results)
        viewer.start_serve_background()
        input("Press Enter to exit...")

    return True


if __name__ == "__main__":
    success = main()
    exit(0 if success else 1)
