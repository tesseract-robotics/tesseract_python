"""
Basic Cartesian Example (High-Level API)

Minimal example of Cartesian motion planning with TrajOpt.
Same as basic_cartesian_example.py but more concise.
"""

import sys
import numpy as np

from tesseract_robotics.planning import (
    Robot,
    MotionProgram,
    CartesianTarget,
    StateTarget,
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
    robot = Robot.from_tesseract_support("lbr_iiwa_14_r820")

    create_obstacle(robot, "box_obstacle", box(0.5, 0.5, 0.5), Pose.from_xyz(1.0, 0, 0))

    joint_names = robot.get_joint_names("manipulator")
    joint_pos = np.array([-0.4, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0])
    robot.set_joints(joint_pos, joint_names=joint_names)

    wp1 = Pose.from_xyz_quat(0.5, -0.2, 0.62, 0, 0, 1.0, 0)
    wp2 = Pose.from_xyz_quat(0.5, 0.3, 0.62, 0, 0, 1.0, 0)

    program = (MotionProgram("manipulator", tcp_frame="tool0", profile="cartesian_program")
        .set_joint_names(joint_names)
        .move_to(StateTarget(joint_pos, names=joint_names, profile="freespace_profile"))
        .move_to(CartesianTarget(wp1, profile="freespace_profile"))
        .linear_to(CartesianTarget(wp2, profile="RASTER"))
        .move_to(StateTarget(joint_pos, names=joint_names, profile="freespace_profile"))
    )

    composer = TaskComposer.from_config()
    result = composer.plan(robot, program, pipeline="TrajOptPipeline")

    assert result.successful, f"Planning failed: {result.message}"
    print(f"Planning successful! Trajectory: {len(result)} waypoints")

    return {"result": result, "robot": robot, "joint_names": joint_names}


def main():
    results = run()
    if TesseractViewer is not None and results["result"].raw_results is not None:
        viewer = TesseractViewer()
        viewer.update_environment(results["robot"].env, [0, 0, 0])
        viewer.update_trajectory(results["result"].raw_results)
        viewer.start_serve_background()
        input("Press Enter to exit...")
    return True


if __name__ == "__main__":
    main()
