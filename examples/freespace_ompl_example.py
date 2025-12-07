"""
Freespace OMPL Example

This example demonstrates freespace motion planning using OMPL with the KUKA IIWA robot.
It adds a sphere obstacle to the environment and plans a collision-free path around it.

Based on: tesseract_examples/src/freespace_ompl_example.cpp

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
    Transform,
    sphere,
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

    # Load KUKA IIWA robot (one-liner!)
    robot = Robot.from_tesseract_support("lbr_iiwa_14_r820")
    print(f"Loaded robot with {len(robot.get_link_names())} links")

    # Add sphere obstacle (one-liner vs 20+ lines before)
    create_obstacle(
        robot,
        name="sphere_attached",
        geometry=sphere(0.15),
        transform=Transform.from_xyz(0.5, 0, 0.55),
    )
    print("Added sphere obstacle at (0.5, 0, 0.55)")

    # Get joint names
    joint_names = robot.get_joint_names("manipulator")

    # Define start and end positions
    joint_start_pos = np.array([-0.4, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0])
    joint_end_pos = np.array([0.4, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0])

    # Set initial state
    robot.set_joints(joint_start_pos, joint_names=joint_names)

    # Create motion program with fluent API (vs 20+ lines of manual poly wrapping)
    program = (MotionProgram("manipulator", tcp_frame="tool0")
        .set_joint_names(joint_names)
        .move_to(JointTarget(joint_start_pos))
        .move_to(JointTarget(joint_end_pos))
    )
    print(f"\nCreated program with {len(program)} waypoints")

    # Plan using TaskComposer
    print("\nRunning OMPL planner (FreespacePipeline)...")
    composer = TaskComposer.from_config()
    result = composer.plan(robot, program, pipeline="FreespacePipeline")

    if not result.successful:
        print(f"Planning failed: {result.message}")
        return False

    print("Planning successful!")
    print(f"\nTrajectory has {len(result)} waypoints:")
    for i, point in enumerate(result.trajectory):
        print(f"  [{i}] {point.positions}")

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
