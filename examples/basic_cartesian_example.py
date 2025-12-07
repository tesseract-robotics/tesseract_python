"""
Basic Cartesian Example

This example demonstrates Cartesian motion planning using TrajOpt with the KUKA IIWA robot.
It creates a box obstacle and plans a trajectory that includes both freespace and
linear Cartesian moves.

Based on: tesseract_examples/src/basic_cartesian_example.cpp

Required environment variables:
- TESSERACT_RESOURCE_PATH: Path to tesseract repo (for tesseract_support)
- TESSERACT_TASK_COMPOSER_CONFIG_FILE: Path to task composer config YAML
"""

import os
import sys
import numpy as np

from tesseract_robotics.planning import (
    Robot,
    MotionProgram,
    JointTarget,
    CartesianTarget,
    StateTarget,
    MoveType,
    Transform,
    box,
    create_obstacle,
    TaskComposer,
)

# Optional: viewer for visualization (disabled in pytest/headless mode)
try:
    from tesseract_robotics_viewer import TesseractViewer
    HAS_VIEWER = os.environ.get("TESSERACT_HEADLESS", "0") != "1" and "pytest" not in sys.modules
except ImportError:
    HAS_VIEWER = False


def main():
    # Check for task composer config
    if not os.environ.get("TESSERACT_TASK_COMPOSER_CONFIG_FILE") and not os.environ.get("TESSERACT_TASK_COMPOSER_DIR"):
        print("Error: TESSERACT_TASK_COMPOSER_CONFIG_FILE or TESSERACT_TASK_COMPOSER_DIR not set")
        print("Run: source env.sh")
        return False

    # Load KUKA IIWA robot
    robot = Robot.from_tesseract_support("lbr_iiwa_14_r820")
    print(f"Loaded robot: {robot}")

    # Add box obstacle (simplified version of point cloud)
    create_obstacle(
        robot,
        name="box_obstacle",
        geometry=box(0.5, 0.5, 0.5),
        transform=Transform.from_xyz(1.0, 0, 0),
    )
    print("Added box obstacle at (1.0, 0, 0)")

    # Get joint names and set initial position
    joint_names = robot.get_joint_names("manipulator")
    joint_pos = np.array([-0.4, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0])
    robot.set_joints(joint_pos, joint_names=joint_names)

    # Create Cartesian waypoints (tool poses)
    # Quaternion(0, 0, 1, 0) = 180 deg rotation around Z (pointing down)
    wp1_pose = Transform.from_xyz_quat(0.5, -0.2, 0.62, 0, 0, 1.0, 0)
    wp2_pose = Transform.from_xyz_quat(0.5, 0.3, 0.62, 0, 0, 1.0, 0)

    # Build motion program with fluent API
    # - Start at joint position
    # - Freespace move to first Cartesian target
    # - Linear move to second Cartesian target
    # - Freespace move back to start
    program = (MotionProgram("manipulator", tcp_frame="tool0", profile="cartesian_program")
        .set_joint_names(joint_names)
        .move_to(StateTarget(joint_pos, names=joint_names, profile="freespace_profile"))
        .move_to(CartesianTarget(wp1_pose, profile="freespace_profile"))
        .linear_to(CartesianTarget(wp2_pose, profile="RASTER"))
        .move_to(StateTarget(joint_pos, names=joint_names, profile="freespace_profile"))
    )

    print("\nProgram created with TrajOpt Cartesian planning")
    print("  - Freespace to Cartesian wp1")
    print("  - Linear to Cartesian wp2")
    print("  - Freespace back to start")

    # Plan using TaskComposer
    print("\nRunning TrajOpt planner...")
    composer = TaskComposer.from_config()
    result = composer.plan(robot, program, pipeline="TrajOptPipeline")

    if not result.successful:
        print(f"Planning failed: {result.message}")
        return False

    print("Planning successful!")
    print(f"\nTrajectory has {len(result)} waypoints")

    # Optional: visualize with viewer
    if HAS_VIEWER and result.raw_results is not None:
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
