"""
Puzzle Piece Auxiliary Axes Example (High-Level API)

Simplified version using high-level planning API.
Cartesian path planning with 9-DOF (KUKA IIWA 7-DOF + 2-DOF positioner).
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

TRAJOPT_NS = "TrajOptMotionPlannerTask"

TesseractViewer = None
if "pytest" not in sys.modules:
    try:
        from tesseract_robotics_viewer import TesseractViewer
    except ImportError:
        pass


def make_puzzle_tool_poses(robot):
    """Load toolpath poses from puzzle_bent.csv."""
    resource = robot.locator.locateResource("package://tesseract_support/urdf/puzzle_bent.csv")
    csv_path = resource.getFilePath()

    poses = []
    with open(csv_path, 'r') as f:
        reader = csv.reader(f)
        for lnum, row in enumerate(reader):
            if lnum < 2 or len(row) < 7:  # Skip header
                continue

            try:
                # Parse x,y,z (mm) and normal i,j,k
                x, y, z = float(row[1]) / 1000, float(row[2]) / 1000, float(row[3]) / 1000
                i, j, k = float(row[4]), float(row[5]), float(row[6])
            except (ValueError, IndexError):
                continue

            pos = np.array([x, y, z])
            norm = np.array([i, j, k])
            norm /= np.linalg.norm(norm)

            # Build frame from normal (z-axis)
            temp_x = -pos / np.linalg.norm(pos) if np.linalg.norm(pos) > 1e-6 else np.array([1, 0, 0])
            y_axis = np.cross(norm, temp_x)
            y_axis /= np.linalg.norm(y_axis)
            x_axis = np.cross(y_axis, norm)
            x_axis /= np.linalg.norm(x_axis)

            rot = np.column_stack([x_axis, y_axis, norm])
            poses.append(Pose.from_matrix_position(rot, pos))

    return poses


def create_profiles():
    """Create TrajOpt profiles for Cartesian path following."""
    profiles = ProfileDictionary()

    # Plan profile: Cartesian constraint with lower weight on tool-axis rotation
    plan = TrajOptDefaultPlanProfile()
    plan.joint_cost_config.enabled = False
    plan.cartesian_cost_config.enabled = False
    plan.cartesian_constraint_config.enabled = True
    plan.cartesian_constraint_config.coeff = np.array([5.0, 5.0, 5.0, 2.0, 2.0, 0.0])

    # Composite profile: collision cost only
    composite = TrajOptDefaultCompositeProfile()
    composite.collision_constraint_config.enabled = False
    composite.collision_cost_config.enabled = True
    composite.collision_cost_config.safety_margin = 0.025
    composite.collision_cost_config.type = CollisionEvaluatorType.SINGLE_TIMESTEP
    composite.collision_cost_config.coeff = 1.0

    ProfileDictionary_addTrajOptPlanProfile(profiles, TRAJOPT_NS, "CARTESIAN", plan)
    ProfileDictionary_addTrajOptCompositeProfile(profiles, TRAJOPT_NS, "DEFAULT", composite)
    return profiles


def main():
    # Load puzzle piece workcell with auxiliary axes
    robot = Robot.from_urdf(
        "package://tesseract_support/urdf/puzzle_piece_workcell.urdf",
        "package://tesseract_support/urdf/puzzle_piece_workcell.srdf"
    )

    # 9 DOF: KUKA IIWA (7) + auxiliary axes (2)
    joint_names = [
        "joint_a1", "joint_a2", "joint_a3", "joint_a4",
        "joint_a5", "joint_a6", "joint_a7",
        "joint_aux1", "joint_aux2"
    ]
    joint_pos = np.array([-0.785398, 0.4, 0.0, -1.9, 0.0, 1.0, 0.0, 0.0, 0.0])
    robot.set_joints(joint_pos, joint_names=joint_names)

    # Load tool poses from CSV
    tool_poses = make_puzzle_tool_poses(robot)
    print(f"Loaded {len(tool_poses)} tool poses")
    assert tool_poses, "No poses loaded from CSV"

    # Build motion program with Cartesian waypoints
    program = (MotionProgram("manipulator_aux", tcp_frame="grinder_frame", working_frame="part")
        .set_joint_names(joint_names))

    for pose in tool_poses:
        program.linear_to(CartesianTarget(pose, profile="CARTESIAN"))

    print(f"Program: {len(program)} waypoints")

    # Plan with custom TrajOpt profiles for Cartesian path following
    print("Planning with TrajOpt (9 DOF: 7 arm + 2 aux)...")
    composer = TaskComposer.from_config()
    profiles = create_profiles()
    result = composer.plan(robot, program, pipeline="TrajOptPipeline", profiles=profiles)

    assert result.successful, f"Planning failed: {result.message}"
    print(f"Success! {len(result)} trajectory waypoints")

    # Visualize
    if TesseractViewer is not None:
        print("\nViewer at http://localhost:8000")
        viewer = TesseractViewer()
        viewer.update_environment(robot.env, [0, 0, 0])
        viewer.update_trajectory(result.raw_results)
        viewer.start_serve_background()
        input("Press Enter to exit...")


if __name__ == "__main__":
    main()
