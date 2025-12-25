"""
Pick and Place Example

Pick and place motion planning using TrajOpt. The robot picks up a box and moves it
toward a shelf.

Based on: tesseract_examples/src/pick_and_place_example.cpp

Uses custom TrajOpt profiles for planning with attached collision objects.
"""

import gc
gc.disable()

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
from tesseract_robotics.tesseract_scene_graph import Joint, JointType
from tesseract_robotics.tesseract_common import Isometry3d
from tesseract_robotics.tesseract_command_language import ProfileDictionary
from tesseract_robotics.tesseract_motion_planners_trajopt import (
    TrajOptDefaultPlanProfile,
    TrajOptDefaultCompositeProfile,
    ProfileDictionary_addTrajOptPlanProfile,
    ProfileDictionary_addTrajOptCompositeProfile,
)

TRAJOPT_DEFAULT_NAMESPACE = "TrajOptMotionPlannerTask"

TesseractViewer = None
if "pytest" not in sys.modules:
    try:
        from tesseract_robotics_viewer import TesseractViewer
    except ImportError:
        pass

OFFSET = 0.005
LINK_BOX_NAME = "box"
LINK_BASE_NAME = "world"  # matches C++ original
LINK_END_EFFECTOR_NAME = "iiwa_tool0"


def create_pick_and_place_profiles():
    """Create custom TrajOpt profiles matching C++ pick_and_place_example.cpp.

    C++ settings (from tesseract_examples):
    - Plan profile: cartesian constraint enabled, coeff=10
    - Composite profile:
      - collision_constraint: margin=0.0, buffer=0.005, coeff=10
      - collision_cost: margin=0.005, buffer=0.01, coeff=50
      - longest_valid_segment_length=0.05
    """
    profiles = ProfileDictionary()

    # Plan profile for CARTESIAN moves (LINEAR motions)
    plan_profile = TrajOptDefaultPlanProfile()
    plan_profile.joint_cost_config.enabled = False
    plan_profile.cartesian_cost_config.enabled = False
    plan_profile.cartesian_constraint_config.enabled = True
    plan_profile.cartesian_constraint_config.coeff = np.full(6, 10.0)

    # Composite profile - adjusted settings for feasibility
    # Use cost-only approach (like puzzle_piece_example) instead of hard constraint
    composite_profile = TrajOptDefaultCompositeProfile()
    composite_profile.longest_valid_segment_length = 0.05
    # Collision constraint disabled (hard constraint causes solver to fail)
    composite_profile.collision_constraint_config.enabled = False
    # Collision cost enabled with small margin
    composite_profile.collision_cost_config.enabled = True
    composite_profile.collision_cost_config.safety_margin = 0.025
    composite_profile.collision_cost_config.coeff = 20.0

    # Register profiles
    ProfileDictionary_addTrajOptPlanProfile(
        profiles, TRAJOPT_DEFAULT_NAMESPACE, "CARTESIAN", plan_profile
    )
    ProfileDictionary_addTrajOptCompositeProfile(
        profiles, TRAJOPT_DEFAULT_NAMESPACE, "DEFAULT", composite_profile
    )

    return profiles


def main():
    box_position = [-0.2, 0.55]
    box_size = 0.1

    # Load KUKA IIWA with workcell
    robot = Robot.from_urdf(
        "package://tesseract_support/urdf/pick_and_place_plan.urdf",
        "package://tesseract_support/urdf/pick_and_place_plan.srdf"
    )
    print(f"Loaded robot with {len(robot.get_link_names())} links")

    # Set default contact distance
    robot.set_collision_margin(0.005)

    # Define joint names for KUKA IIWA
    joint_names = [
        "iiwa_joint_a1", "iiwa_joint_a2", "iiwa_joint_a3", "iiwa_joint_a4",
        "iiwa_joint_a5", "iiwa_joint_a6", "iiwa_joint_a7"
    ]

    # Define initial joint position
    joint_start_pos = np.array([0.0, 0.0, 0.0, -1.57, 0.0, 0.0, 0.0])

    # Set initial state
    robot.set_joints(joint_start_pos, joint_names=joint_names)

    # Add box to environment
    create_obstacle(
        robot,
        name=LINK_BOX_NAME,
        geometry=box(box_size, box_size, box_size),
        transform=Pose.from_xyz(box_position[0], box_position[1], box_size / 2.0 + OFFSET),
        parent_link="workcell_base",
    )
    print(f"Added box at ({box_position[0]}, {box_position[1]}) with size {box_size}m")

    # Create task composer and custom profiles
    composer = TaskComposer.from_config()
    profiles = create_pick_and_place_profiles()

    # ==================== PICK PHASE ====================
    print("\n=== PICK PHASE ===")

    # Define pick pose (on top of the box)
    # Rotation: pointing down (-Z)
    pick_rotation = np.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]])
    pick_position = [box_position[0], box_position[1], box_size + 0.772 + OFFSET]
    pick_final_pose = Pose.from_matrix_position(pick_rotation, pick_position)

    # Approach pose (15cm above pick)
    approach_position = [pick_position[0], pick_position[1], pick_position[2] + 0.15]
    pick_approach_pose = Pose.from_matrix_position(pick_rotation, approach_position)

    # Create pick program
    pick_program = (MotionProgram("manipulator", tcp_frame=LINK_END_EFFECTOR_NAME, working_frame=LINK_BASE_NAME)
        .set_joint_names(joint_names)
        .move_to(StateTarget(joint_start_pos, names=joint_names, profile="FREESPACE"))
        .move_to(CartesianTarget(pick_approach_pose, profile="FREESPACE"))
        .linear_to(CartesianTarget(pick_final_pose, profile="CARTESIAN"))
    )

    print(f"Pick program: {len(pick_program)} instructions")
    print("Running TrajOpt planner for PICK...")

    # TaskComposer.plan() auto-seeds Cartesian waypoints
    pick_result = composer.plan(robot, pick_program, pipeline="TrajOptPipeline", profiles=profiles)

    if not pick_result.successful:
        print(f"PICK planning failed: {pick_result.message}")
        return False

    print("PICK planning successful!")
    print(f"Pick trajectory has {len(pick_result)} waypoints")

    # ==================== ATTACH BOX TO END EFFECTOR ====================
    print("\n=== ATTACHING BOX ===")

    # Move box link to be child of end effector
    joint_box2 = Joint("joint_box2")
    joint_box2.parent_link_name = LINK_END_EFFECTOR_NAME
    joint_box2.child_link_name = LINK_BOX_NAME
    joint_box2.type = JointType.FIXED
    # Box center is box_size/2 below end effector
    box_attach_mat = np.eye(4)
    box_attach_mat[:3, 3] = [0, 0, box_size / 2.0]
    joint_box2.parent_to_joint_origin_transform = Isometry3d(box_attach_mat)

    robot.move_link(joint_box2)

    # Add allowed collisions between box and nearby links (matches C++ exactly)
    robot.add_allowed_collision(LINK_BOX_NAME, LINK_END_EFFECTOR_NAME, "Never")
    robot.add_allowed_collision(LINK_BOX_NAME, "iiwa_link_7", "Never")
    robot.add_allowed_collision(LINK_BOX_NAME, "iiwa_link_6", "Never")

    print("Box attached to end effector with collision exceptions")

    # ==================== PLACE PHASE ====================
    print("\n=== PLACE PHASE ===")

    # Get final state from pick trajectory and update robot state
    pick_final = pick_result.trajectory[-1].positions
    robot.set_joints(pick_final, joint_names=joint_names)

    # Define place location (middle left shelf)
    # C++ Eigen::Quaterniond(w=0, x=0, y=0.7071068, z=0.7071068).matrix()
    # This is a 180° rotation around axis (0, 0.7071, 0.7071)
    # Rotation matrix: [[-1,0,0], [0,0,1], [0,1,0]]
    place_rotation = np.array([
        [-1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0],
        [0.0, 1.0, 0.0]
    ])
    place_position = [-0.148856, 0.73085, 1.16]  # middle_left_shelf
    place_pose = Pose.from_matrix_position(place_rotation, place_position)

    # Approach pose (25cm back in Y - matches C++)
    place_approach_position = np.array(place_position) + np.array([0.0, -0.25, 0.0])
    place_approach_pose = Pose.from_matrix_position(place_rotation, place_approach_position)

    # Retreat to pick approach
    retreat_pose = pick_approach_pose

    # Create place program (C++ uses LINEAR for retreat and place, FREESPACE for approach)
    place_program = (MotionProgram("manipulator", tcp_frame=LINK_END_EFFECTOR_NAME, working_frame=LINK_BASE_NAME)
        .set_joint_names(joint_names)
        .move_to(StateTarget(pick_final, names=joint_names))
        .linear_to(CartesianTarget(retreat_pose, profile="CARTESIAN"))
        .move_to(CartesianTarget(place_approach_pose, profile="FREESPACE"))
        .linear_to(CartesianTarget(place_pose, profile="CARTESIAN"))
    )

    print(f"Place program: {len(place_program)} instructions")
    print("Running TrajOpt planner for PLACE...")

    place_result = composer.plan(robot, place_program, pipeline="TrajOptPipeline", profiles=profiles)

    if not place_result.successful:
        print(f"PLACE planning failed: {place_result.message}")
        return False

    print("PLACE planning successful!")
    print(f"Place trajectory has {len(place_result)} waypoints")

    # Optional: visualize with viewer
    if TesseractViewer is not None:
        print("\nStarting viewer at http://localhost:8000")
        viewer = TesseractViewer()
        viewer.update_environment(robot.env, [0, 0, 0])
        viewer.update_trajectory(place_result.raw_results)
        viewer.start_serve_background()
        input("Press Enter to exit...")

    print("\nDone!")
    return True


if __name__ == "__main__":
    success = main()
    exit(0 if success else 1)
