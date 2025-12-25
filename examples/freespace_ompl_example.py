"""
Freespace OMPL Example (High-Level API)

Demonstrates freespace motion planning using OMPL with the KUKA IIWA robot.
Adds a sphere obstacle to the environment and plans a collision-free path around it.
"""

import sys
import numpy as np

from tesseract_robotics.planning import (
    Robot,
    MotionProgram,
    JointTarget,
    Pose,
    sphere,
    create_obstacle,
    TaskComposer,
)

TesseractViewer = None
if "pytest" not in sys.modules:
    try:
        from tesseract_robotics_viewer import TesseractViewer
    except ImportError:
        pass


def main():
    # Load KUKA IIWA robot
    robot = Robot.from_tesseract_support("lbr_iiwa_14_r820")
    print(f"Loaded robot with {len(robot.get_link_names())} links")

    # Add sphere obstacle (one-liner vs 20+ lines before)
    create_obstacle(
        robot,
        name="sphere_attached",
        geometry=sphere(0.15),
        transform=Pose.from_xyz(0.5, 0, 0.55),
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

    assert result.successful, f"Planning failed: {result.message}"
    print("Planning successful!")
    print(f"\nTrajectory has {len(result)} waypoints:")
    for i, point in enumerate(result.trajectory):
        print(f"  [{i}] {point.positions}")

    # Optional: visualize with viewer
    if TesseractViewer is not None and result.raw_results is not None:
        print("\nStarting viewer at http://localhost:8000")
        viewer = TesseractViewer()
        viewer.update_environment(robot.env, [0, 0, 0])
        viewer.update_trajectory(result.raw_results)
        viewer.start_serve_background()
        input("Press Enter to exit...")


if __name__ == "__main__":
    main()
