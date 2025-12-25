"""
Pick and Place Example (High-Level API)

Same workflow as pick_and_place_example.py but using the high-level planning API.
Demonstrates Pose helpers, create_obstacle, create_fixed_joint.

Based on: tesseract_examples/src/pick_and_place_example.cpp
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
    create_fixed_joint,
    TaskComposer,
)
from tesseract_robotics.tesseract_command_language import ProfileDictionary
from tesseract_robotics.tesseract_motion_planners_trajopt import (
    TrajOptDefaultCompositeProfile,
    ProfileDictionary_addTrajOptCompositeProfile,
)
from tesseract_robotics.planning.profiles import create_freespace_pipeline_profiles

TRAJOPT_NS = "TrajOptMotionPlannerTask"

TesseractViewer = None
if "pytest" not in sys.modules:
    try:
        from tesseract_robotics_viewer import TesseractViewer
    except ImportError:
        pass

OFFSET = 0.005
BOX_SIZE = 0.1
LINK_BOX = "box"
LINK_BASE = "world"
LINK_TCP = "iiwa_tool0"


def create_profiles():
    """Minimal TrajOpt profiles for collision avoidance."""
    profiles = ProfileDictionary()
    composite = TrajOptDefaultCompositeProfile()
    composite.longest_valid_segment_length = 0.05
    composite.collision_constraint_config.enabled = False
    composite.collision_cost_config.enabled = True
    composite.collision_cost_config.safety_margin = 0.025
    composite.collision_cost_config.coeff = 20.0
    ProfileDictionary_addTrajOptCompositeProfile(profiles, TRAJOPT_NS, "DEFAULT", composite)
    return profiles


def run(pipeline="TrajOptPipeline", num_planners=None):
    """Run example and return trajectory results for testing.

    Args:
        pipeline: Planning pipeline to use (default: TrajOptPipeline)
        num_planners: Number of parallel OMPL planners (for FreespacePipeline)

    Returns:
        dict with pick_result, place_result, robot, joint_names
    """
    box_pos = [-0.2, 0.55]

    # Load robot
    robot = Robot.from_urdf(
        "package://tesseract_support/urdf/pick_and_place_plan.urdf",
        "package://tesseract_support/urdf/pick_and_place_plan.srdf"
    )
    robot.set_collision_margin(0.005)

    joint_names = [f"iiwa_joint_a{i}" for i in range(1, 8)]
    start_pos = np.array([0.0, 0.0, 0.0, -1.57, 0.0, 0.0, 0.0])
    robot.set_joints(start_pos, joint_names=joint_names)

    # Add box using high-level API
    create_obstacle(
        robot,
        name=LINK_BOX,
        geometry=box(BOX_SIZE, BOX_SIZE, BOX_SIZE),
        transform=Pose.from_xyz(box_pos[0], box_pos[1], BOX_SIZE / 2 + OFFSET),
        parent_link="workcell_base",
    )
    print(f"Added box at {box_pos}")

    composer = TaskComposer.from_config()

    # Create profiles based on pipeline
    if "Freespace" in pipeline or "OMPL" in pipeline:
        profiles = create_freespace_pipeline_profiles(num_planners=num_planners)
    else:
        profiles = create_profiles()

    # === PICK ===
    print("\n=== PICK ===")

    # Pick pose: pointing down (-Z), 180deg rotation around Y axis
    # Rotation matrix: [[-1,0,0], [0,1,0], [0,0,-1]] matches C++ original
    pick_z = BOX_SIZE + 0.772 + OFFSET
    pick_rotation = np.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]])
    pick_pose = Pose.from_matrix_position(pick_rotation, [box_pos[0], box_pos[1], pick_z])
    approach_pose = Pose.from_matrix_position(pick_rotation, [box_pos[0], box_pos[1], pick_z + 0.15])

    pick_program = (MotionProgram("manipulator", tcp_frame=LINK_TCP, working_frame=LINK_BASE)
        .set_joint_names(joint_names)
        .move_to(StateTarget(start_pos, names=joint_names, profile="FREESPACE"))
        .move_to(CartesianTarget(approach_pose, profile="FREESPACE"))
        .linear_to(CartesianTarget(pick_pose, profile="CARTESIAN"))
    )

    pick_result = composer.plan(robot, pick_program, pipeline=pipeline, profiles=profiles)
    assert pick_result.successful, f"PICK failed: {pick_result.message}"
    print(f"PICK OK: {len(pick_result)} waypoints")

    # === ATTACH BOX ===
    print("\n=== ATTACH ===")
    pick_final = pick_result.trajectory[-1].positions
    robot.set_joints(pick_final, joint_names=joint_names)

    # High-level joint creation
    attach_joint = create_fixed_joint(
        "joint_box2", LINK_TCP, LINK_BOX,
        origin=Pose.from_xyz(0, 0, BOX_SIZE / 2)
    )
    robot.move_link(attach_joint)

    robot.add_allowed_collision(LINK_BOX, LINK_TCP, "Never")
    robot.add_allowed_collision(LINK_BOX, "iiwa_link_7", "Never")
    robot.add_allowed_collision(LINK_BOX, "iiwa_link_6", "Never")
    print("Box attached")

    # === PLACE ===
    print("\n=== PLACE ===")

    # Place pose: C++ Eigen::Quaterniond(w=0, x=0, y=0.7071, z=0.7071) = rotation matrix [[-1,0,0],[0,0,1],[0,1,0]]
    place_rotation = np.array([[-1.0, 0.0, 0.0], [0.0, 0.0, 1.0], [0.0, 1.0, 0.0]])
    place_pos = [-0.148856, 0.73085, 1.16]
    place_pose = Pose.from_matrix_position(place_rotation, place_pos)

    # Approach: 25cm back in Y
    place_approach_pos = [place_pos[0], place_pos[1] - 0.25, place_pos[2]]
    place_approach_pose = Pose.from_matrix_position(place_rotation, place_approach_pos)

    place_program = (MotionProgram("manipulator", tcp_frame=LINK_TCP, working_frame=LINK_BASE)
        .set_joint_names(joint_names)
        .move_to(StateTarget(pick_final, names=joint_names))
        .linear_to(CartesianTarget(approach_pose, profile="CARTESIAN"))
        .move_to(CartesianTarget(place_approach_pose, profile="FREESPACE"))
        .linear_to(CartesianTarget(place_pose, profile="CARTESIAN"))
    )

    place_result = composer.plan(robot, place_program, pipeline=pipeline, profiles=profiles)
    assert place_result.successful, f"PLACE failed: {place_result.message}"
    print(f"PLACE OK: {len(place_result)} waypoints")

    return {
        "pick_result": pick_result,
        "place_result": place_result,
        "robot": robot,
        "joint_names": joint_names,
    }


def main():
    results = run()

    if TesseractViewer is not None:
        print("\nViewer at http://localhost:8000")
        viewer = TesseractViewer()
        viewer.update_environment(results["robot"].env, [0, 0, 0])
        viewer.update_trajectory(results["place_result"].raw_results)
        viewer.start_serve_background()
        input("Press Enter to exit...")

    print("\nDone!")


if __name__ == "__main__":
    main()
