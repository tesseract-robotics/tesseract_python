"""
Planning Example with Task Composer

Demonstrates freespace motion planning using the high-level API.
The robot moves through Cartesian waypoints with collision avoidance.

This example shows how simple planning is with the Pythonic API vs low-level.
For low-level planner access, see tesseract_planning_lowlevel_example.py
"""
import sys
import numpy as np

from tesseract_robotics.planning import (
    Robot,
    MotionProgram,
    CartesianTarget,
    Pose,
    TaskComposer,
)

TesseractViewer = None
if "pytest" not in sys.modules:
    try:
        from tesseract_robotics_viewer import TesseractViewer
    except ImportError:
        pass


def main():
    # Load ABB IRB2400 robot
    robot = Robot.from_tesseract_support("abb_irb2400")
    print(f"Loaded robot with {len(robot.get_link_names())} links")

    # Get joint names and set initial position
    joint_names = robot.get_joint_names("manipulator")
    robot.set_joints(dict(zip(joint_names, [0.1] * 6)))

    # Create Cartesian waypoints
    # Quaternion(0.707, 0, 0.707, 0) = 90 deg rotation
    wp1 = Pose.from_xyz_quat(0.8, -0.3, 1.455, 0.707, 0, 0.707, 0)
    wp2 = Pose.from_xyz_quat(0.8, 0.3, 1.455, 0.707, 0, 0.707, 0)

    # Build motion program with fluent API
    program = (MotionProgram("manipulator", tcp_frame="tool0")
        .set_joint_names(joint_names)
        .move_to(CartesianTarget(wp1))
        .move_to(CartesianTarget(wp2))
    )

    print(f"\nProgram has {len(program)} waypoints")

    # Plan using TaskComposer
    print("Running FreespacePipeline (OMPL + TrajOpt)...")
    composer = TaskComposer.from_config()
    result = composer.plan(robot, program, pipeline="FreespacePipeline")

    if not result.successful:
        print(f"Planning failed: {result.message}")
        return False

    print("Planning successful!")
    print(f"\nTrajectory has {len(result)} waypoints:")
    for i, point in enumerate(result.trajectory):
        time_str = f" t={point.time:.3f}" if point.time else ""
        print(f"  [{i:2d}] {point.positions}{time_str}")

    # Optional: visualize with viewer
    if TesseractViewer is not None and result.raw_results is not None:
        print("\nStarting viewer at http://localhost:8000")
        viewer = TesseractViewer()
        viewer.update_environment(robot.env, [0, 0, 0])
        viewer.update_joint_positions(joint_names, np.array([1, -.2, .01, .3, -.5, 1]))
        viewer.start_serve_background()
        viewer.update_trajectory(result.raw_results)
        input("Press Enter to exit...")

    return True


if __name__ == "__main__":
    success = main()
    exit(0 if success else 1)
