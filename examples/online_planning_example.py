"""
Online Planning Example (TrajOpt IFOPT)

Demonstrates real-time trajectory replanning as the environment changes.
Uses TrajOptIfopt for efficient trajectory optimization.

Based on: tesseract_planning/tesseract_examples/src/online_planning_example.cpp

Key Features:
- Trajectory replanning as obstacles move
- Uses TrajOptIfopt (SQP with OSQP solver)
- Simulated dynamic environment

Note: This example uses the TaskComposer pipeline interface.
For true real-time planning with stepSQPSolver(), additional low-level
bindings for trajopt_sqp would be needed.
"""

import sys
import time
import numpy as np

from tesseract_robotics.planning import (
    Robot,
    MotionProgram,
    CartesianTarget,
    StateTarget,
    Pose,
    sphere,
    create_obstacle,
    TaskComposer,
)
from tesseract_robotics.planning.profiles import create_trajopt_ifopt_default_profiles

TesseractViewer = None
if "pytest" not in sys.modules:
    try:
        from tesseract_robotics_viewer import TesseractViewer
    except ImportError:
        pass


def update_obstacle_position(robot, obstacle_name, position):
    """Update obstacle position in the environment."""
    from tesseract_robotics.tesseract_environment import ChangeJointOriginCommand
    from tesseract_robotics.tesseract_common import Isometry3d, Translation3d

    # Create isometry transform (identity rotation + translation)
    translation = Translation3d(position[0], position[1], position[2])
    transform = Isometry3d.Identity() * translation

    # Apply the change (joint name is "joint_{name}")
    cmd = ChangeJointOriginCommand(f"joint_{obstacle_name}", transform)
    robot.env.applyCommand(cmd)


def run(pipeline="TrajOptIfoptPipeline", num_iterations=5, num_planners=None):
    """Run online planning example with moving obstacle.

    Args:
        pipeline: Planning pipeline to use (default: TrajOptIfoptPipeline)
        num_iterations: Number of replanning iterations
        num_planners: Number of parallel planners (unused for TrajOptIfopt)

    Returns:
        dict with results
    """
    # Load robot
    robot = Robot.from_tesseract_support("lbr_iiwa_14_r820")

    # Add a "human" obstacle that will move
    create_obstacle(robot, "human", sphere(0.15), Pose.from_xyz(0.5, 0.0, 0.5))

    # Setup joint configuration
    joint_names = robot.get_joint_names("manipulator")
    home = np.array([0.0, 0.0, 0.0, -1.57, 0.0, 0.0, 0.0])
    robot.set_joints(home, joint_names=joint_names)

    # Target pose - tool should reach this while avoiding obstacles
    target_pose = Pose.from_xyz_quat(0.6, 0.0, 0.6, 0.0, 0.707, 0.0, 0.707)

    # Create profiles for TrajOptIfopt
    profiles = create_trajopt_ifopt_default_profiles()

    # Create task composer
    composer = TaskComposer.from_config()

    trajectories = []
    timings = []

    for iteration in range(num_iterations):
        # Move the obstacle to simulate a dynamic environment
        # Oscillates in Y direction
        t = iteration * 0.5
        obstacle_y = 0.3 * np.sin(t)
        update_obstacle_position(robot, "human", [0.5, obstacle_y, 0.5])

        # Get current state as start
        state = robot.get_state(joint_names)
        current_joints = state.joint_positions

        # Create motion program: current -> target
        program = (MotionProgram("manipulator", tcp_frame="tool0", profile="DEFAULT")
            .set_joint_names(joint_names)
            .move_to(StateTarget(current_joints, names=joint_names, profile="DEFAULT"))
            .move_to(CartesianTarget(target_pose, profile="DEFAULT"))
        )

        # Plan with timing
        start_time = time.time()
        try:
            result = composer.plan(robot, program, pipeline=pipeline, profiles=profiles)
            planning_time = time.time() - start_time

            if result.successful:
                trajectories.append(result)
                timings.append(planning_time)
                print(f"Iteration {iteration + 1}: Success in {planning_time:.3f}s, "
                      f"{len(result)} waypoints, obstacle_y={obstacle_y:.3f}")

                # Update robot state to end of trajectory (simulate execution)
                if len(result.trajectory) > 0:
                    robot.set_joints(result.trajectory[-1].positions, joint_names=joint_names)
            else:
                print(f"Iteration {iteration + 1}: Planning failed - {result.message}")
                timings.append(planning_time)
        except Exception as e:
            planning_time = time.time() - start_time
            timings.append(planning_time)
            print(f"Iteration {iteration + 1}: Exception - {e}")

    # Summary statistics
    if timings:
        avg_time = np.mean(timings)
        print(f"\nOnline Planning Summary:")
        print(f"  Iterations: {num_iterations}")
        print(f"  Successful: {len(trajectories)}")
        print(f"  Average planning time: {avg_time:.3f}s ({1.0/avg_time if avg_time > 0 else 0:.1f} Hz)")

    return {
        "trajectories": trajectories,
        "timings": timings,
        "robot": robot,
        "joint_names": joint_names,
        "success": len(trajectories) > 0,
    }


def main():
    results = run()

    if TesseractViewer is not None and results.get("trajectories"):
        viewer = TesseractViewer()
        viewer.update_environment(results["robot"].env, [0, 0, 0])
        # Show last trajectory
        if results["trajectories"][-1].raw_results is not None:
            viewer.update_trajectory(results["trajectories"][-1].raw_results)
        viewer.start_serve_background()
        input("Press Enter to exit...")

    return results["success"]


if __name__ == "__main__":
    main()
