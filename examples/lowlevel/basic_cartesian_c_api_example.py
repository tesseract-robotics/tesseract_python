"""
Basic Cartesian Example

Demonstrates Cartesian motion planning using TrajOpt with the KUKA IIWA robot.
Creates a box obstacle and plans a trajectory with freespace and linear Cartesian moves.
"""

import sys
import numpy as np

from tesseract_robotics.planning import (
    Robot,
    MotionProgram,
    JointTarget,
    CartesianTarget,
    StateTarget,
    MoveType,
    Pose,
    box,
    create_obstacle,
    TaskComposer,
)

TesseractViewer = None
if "pytest" not in sys.modules:
    try:
        from tesseract_robotics_viewer import TesseractViewer
    except ImportError:
        pass


def run():
    """Run example and return trajectory results for testing.
    Returns:
        dict with result, robot, joint_names
    """
    # Load KUKA IIWA robot
    robot = Robot.from_tesseract_support("lbr_iiwa_14_r820")
    print(f"Loaded robot: {robot}")

    # Add box obstacle (simplified version of point cloud)
    create_obstacle(
        robot,
        name="box_obstacle",
        geometry=box(0.5, 0.5, 0.5),
        transform=Pose.from_xyz(1.0, 0, 0),
    )
    print("Added box obstacle at (1.0, 0, 0)")

    # Get joint names and set initial position
    joint_names = robot.get_joint_names("manipulator")
    joint_pos = np.array([-0.4, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0])
    robot.set_joints(joint_pos, joint_names=joint_names)

    # Create Cartesian waypoints (tool poses)
    # Quaternion(0, 0, 1, 0) = 180 deg rotation around Z (pointing down)
    wp1_pose = Pose.from_xyz_quat(0.5, -0.2, 0.62, 0, 0, 1.0, 0)
    wp2_pose = Pose.from_xyz_quat(0.5, 0.3, 0.62, 0, 0, 1.0, 0)

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

    assert result.successful, f"Planning failed: {result.message}"

    print("Planning successful!")
    print(f"\nTrajectory has {len(result)} waypoints")

    return {"result": result, "robot": robot, "joint_names": joint_names}


def main():
    results = run()
    if TesseractViewer is not None and results["result"].raw_results is not None:
        print("\nStarting viewer at http://localhost:8000")
        viewer = TesseractViewer()
        viewer.update_environment(results["robot"].env, [0, 0, 0])
        viewer.update_trajectory(results["result"].raw_results)
        viewer.start_serve_background()
        input("Press Enter to exit...")
    return True


if __name__ == "__main__":
    success = main()
    exit(0 if success else 1)
