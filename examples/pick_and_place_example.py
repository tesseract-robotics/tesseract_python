"""
Pick and Place Example

This example demonstrates pick and place motion planning using TrajOpt.
The robot picks up a box from the table and places it on a shelf.

Key features:
- Dynamic environment modification (adding box to scene)
- MoveLinkCommand to attach box to end effector
- ModifyAllowedCollisionsCommand to ignore box-gripper collisions
- Two-phase planning: PICK then PLACE

Based on: tesseract_examples/src/pick_and_place_example.cpp

Required environment variables:
- TESSERACT_RESOURCE_PATH: Path to tesseract repo (for tesseract_support)
- TESSERACT_TASK_COMPOSER_CONFIG_FILE: Path to task composer config YAML
"""
import sys
import os
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

# Viewer (skip in pytest)
TesseractViewer = None
if "pytest" not in sys.modules:
    try:
        from tesseract_robotics_viewer import TesseractViewer
    except ImportError:
        pass

# Constants
OFFSET = 0.005
LINK_BOX_NAME = "box"
LINK_BASE_NAME = "base_link"
LINK_END_EFFECTOR_NAME = "iiwa_tool0"


def main():
    # Configuration
    box_position = [-0.2, 0.55]  # x, y position of box on table
    box_size = 0.1  # 10cm cube

    # Check for task composer config
    if not os.environ.get("TESSERACT_TASK_COMPOSER_CONFIG_FILE") and not os.environ.get("TESSERACT_TASK_COMPOSER_DIR"):
        print("Error: TESSERACT_TASK_COMPOSER_CONFIG_FILE or TESSERACT_TASK_COMPOSER_DIR not set")
        print("Run: source env.sh")
        return False

    # Load KUKA IIWA with workcell (includes shelves/cabinet)
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

    # Create task composer
    composer = TaskComposer.from_config()

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
    pick_program = (MotionProgram("manipulator", tcp_frame=LINK_END_EFFECTOR_NAME)
        .set_joint_names(joint_names)
        .move_to(StateTarget(joint_start_pos, names=joint_names, profile="FREESPACE"))
        .move_to(CartesianTarget(pick_approach_pose, profile="FREESPACE"))
        .linear_to(CartesianTarget(pick_final_pose, profile="CARTESIAN"))
    )

    print(f"Pick program: {len(pick_program)} instructions")
    print("Running TrajOpt planner for PICK...")

    # TaskComposer.plan() auto-seeds Cartesian waypoints
    pick_result = composer.plan(robot, pick_program, pipeline="TrajOptPipeline")

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

    # Add allowed collisions between box and nearby links
    robot.add_allowed_collision(LINK_BOX_NAME, LINK_END_EFFECTOR_NAME, "Never")
    robot.add_allowed_collision(LINK_BOX_NAME, "iiwa_link_7", "Never")
    robot.add_allowed_collision(LINK_BOX_NAME, "iiwa_link_6", "Never")

    print("Box attached to end effector with collision exceptions")

    # ==================== PLACE PHASE ====================
    print("\n=== PLACE PHASE ===")

    # Get final state from pick trajectory
    pick_final = pick_result.trajectory[-1].positions

    # Define place location (middle left shelf)
    # 90deg rotation around Z axis
    cos_45, sin_45 = 0.7071068, 0.7071068
    place_rotation = np.array([
        [cos_45, -sin_45, 0],
        [sin_45, cos_45, 0],
        [0, 0, 1]
    ])
    place_position = [-0.148856, 0.73085, 1.16]
    place_pose = Pose.from_matrix_position(place_rotation, place_position)

    # Approach pose (25cm back in local Y)
    offset_local = np.array([0.0, -0.25, 0])
    offset_world = place_rotation @ offset_local
    place_approach_position = np.array(place_position) + offset_world
    place_approach_pose = Pose.from_matrix_position(place_rotation, place_approach_position)

    # Retreat to pick approach
    retreat_pose = pick_approach_pose

    # Create place program
    place_program = (MotionProgram("manipulator", tcp_frame=LINK_END_EFFECTOR_NAME)
        .set_joint_names(joint_names)
        .move_to(StateTarget(pick_final, names=joint_names))  # Start from pick final
        .linear_to(CartesianTarget(retreat_pose, profile="CARTESIAN"))  # Retreat
        .move_to(CartesianTarget(place_approach_pose, profile="FREESPACE"))  # To approach
        .linear_to(CartesianTarget(place_pose, profile="CARTESIAN"))  # Final place
    )

    print(f"Place program: {len(place_program)} instructions")
    print("Running TrajOpt planner for PLACE...")

    place_result = composer.plan(robot, place_program, pipeline="TrajOptPipeline")

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
