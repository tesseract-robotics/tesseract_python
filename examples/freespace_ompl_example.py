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
from tesseract_robotics.planning.profiles import create_freespace_pipeline_profiles

TesseractViewer = None
if "pytest" not in sys.modules:
    try:
        from tesseract_robotics_viewer import TesseractViewer
    except ImportError:
        pass


def run(pipeline="FreespacePipeline", num_planners=None):
    """Run example and return trajectory results for testing.

    Args:
        pipeline: Planning pipeline to use (default: FreespacePipeline)
        num_planners: Number of parallel OMPL planners (default: all CPUs)

    Returns:
        dict with result, robot, joint_names
    """
    # Load KUKA IIWA robot
    robot = Robot.from_tesseract_support("lbr_iiwa_14_r820")
    print(f"Loaded robot with {len(robot.get_link_names())} links")

    # Add sphere obstacle
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

    # Create motion program
    program = (MotionProgram("manipulator", tcp_frame="tool0")
        .set_joint_names(joint_names)
        .move_to(JointTarget(joint_start_pos))
        .move_to(JointTarget(joint_end_pos))
    )
    print(f"\nCreated program with {len(program)} waypoints")

    # Plan using TaskComposer
    print(f"\nRunning planner ({pipeline})...")
    composer = TaskComposer.from_config()
    profiles = create_freespace_pipeline_profiles(num_planners=num_planners)
    result = composer.plan(robot, program, pipeline=pipeline, profiles=profiles)

    assert result.successful, f"Planning failed: {result.message}"
    print(f"Planning successful! {len(result)} waypoints")

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


if __name__ == "__main__":
    main()
# test comment
