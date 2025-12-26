"""
Raster Example (High-Level API)

Demonstrates industrial raster/process planning patterns:
- Multiple raster passes with linear motion
- Transitions between passes (freespace)
- Approach and retract segments

Based on: tesseract_planning/tesseract_motion_planners/examples/raster_example.cpp

Use cases: welding seams, painting, milling, grinding, polishing
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
from tesseract_robotics.planning.profiles import create_trajopt_default_profiles

TesseractViewer = None
if "pytest" not in sys.modules:
    try:
        from tesseract_robotics_viewer import TesseractViewer
    except ImportError:
        pass


def run(pipeline="TrajOptPipeline", num_planners=None):
    """Run example and return trajectory results for testing.

    Args:
        pipeline: Planning pipeline to use (default: TrajOptPipeline)
        num_planners: Number of parallel threads (unused for TrajOpt)

    Returns:
        dict with result, robot, joint_names
    """
    # Load ABB IRB2400 robot (same as C++ example)
    robot = Robot.from_urdf(
        "package://tesseract_support/urdf/abb_irb2400.urdf",
        "package://tesseract_support/urdf/abb_irb2400.srdf",
    )
    print(f"Loaded robot with {len(robot.get_link_names())} links")

    # Get joint info
    joint_names = robot.get_joint_names("manipulator")

    # Define home position (from C++ example)
    home_pos = np.zeros(6)
    robot.set_joints(home_pos, joint_names=joint_names)

    # Tool orientation: C++ Quaterniond(w=0, x=0, y=-1.0, z=0) = 180deg around Y
    # Python from_xyz_quat format: (x, y, z, qx, qy, qz, qw)
    tool_quat = [0.0, -1.0, 0.0, 0.0]  # (x=0, y=-1, z=0, w=0)

    # Waypoints from C++ example: X=0.8, Z=0.8, Y varies from -0.3 to 0.3
    x_const = 0.8
    z_const = 0.8
    y_values = [-0.3, -0.2, -0.1, 0.0, 0.1, 0.2, 0.3]

    # Create waypoints
    waypoints = [
        Pose.from_xyz_quat(x_const, y, z_const, *tool_quat)
        for y in y_values
    ]
    print(f"Created {len(waypoints)} waypoints")

    # Build motion program following C++ structure:
    # - Start from joint state
    # - Approach to first waypoint (freespace)
    # - Execute raster segment (linear moves)
    # - Repeat 3 times with transitions
    num_segments = 3

    program = (MotionProgram("manipulator", tcp_frame="tool0", profile="RASTER")
        .set_joint_names(joint_names)
    )

    # Start from home
    program.move_to(StateTarget(home_pos, names=joint_names, profile="FREESPACE"))

    for seg in range(num_segments):
        # Approach to first waypoint (freespace)
        program.move_to(CartesianTarget(waypoints[0], profile="FREESPACE"))

        # Linear moves through all waypoints
        for wp in waypoints[1:]:
            program.linear_to(CartesianTarget(wp, profile="RASTER"))

        # Transition back (freespace) if not last segment
        if seg < num_segments - 1:
            program.move_to(CartesianTarget(waypoints[0], profile="FREESPACE"))

    # Return to home
    program.move_to(StateTarget(home_pos, names=joint_names, profile="FREESPACE"))

    print(f"Created program with {len(program)} waypoints ({num_segments} segments)")

    # Plan using TaskComposer
    print(f"\nRunning planner ({pipeline})...")
    composer = TaskComposer.from_config()
    profiles = create_trajopt_default_profiles()
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
