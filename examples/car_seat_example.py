"""
Car Seat Example (High-Level API)

Streamlined version of car_seat_example.py demonstrating:
- Dynamic mesh loading and convex hull generation
- Multi-phase pick-and-place planning
- Link attachment/detachment

Based on: tesseract_examples/src/car_seat_example.cpp
"""
import sys
import numpy as np

from tesseract_robotics.planning import Robot, MotionProgram, StateTarget, TaskComposer
from tesseract_robotics.tesseract_common import Isometry3d, AllowedCollisionMatrix
from tesseract_robotics.tesseract_environment import (
    AddLinkCommand,
    MoveLinkCommand,
    ModifyAllowedCollisionsCommand,
    ModifyAllowedCollisionsType,
)
from tesseract_robotics.tesseract_scene_graph import Link, Joint, JointType, Visual, Collision
from tesseract_robotics.tesseract_geometry import createMeshFromPath
from tesseract_robotics.tesseract_collision import makeConvexMesh

TesseractViewer = None
if "pytest" not in sys.modules:
    try:
        from tesseract_robotics_viewer import TesseractViewer
    except ImportError:
        pass

# Predefined joint positions
POSITIONS = {
    "Home": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "Pick1": [2.22, 0.39, 0.0, 0.5, 0.0, -3.14, -0.29, -1.45],
    "Place1": [4.15466, 0.537218, 0.0189056, 0.801223, 0.0580309, -0.0481182, -0.325783, -1.2813],
}


def add_seats(robot):
    """Create and add 3 seat links with visual and collision meshes."""
    locator = robot.locator
    visual_path = locator.locateResource(
        "package://tesseract_support/meshes/car_seat/visual/seat.dae"
    ).getFilePath()

    for i in range(3):
        seat_name = f"seat_{i + 1}"
        link = Link(seat_name)

        # Visual mesh
        visual = Visual()
        visual.origin = Isometry3d.Identity()
        visual_meshes = createMeshFromPath(visual_path)
        if visual_meshes:
            visual.geometry = visual_meshes[0]
        link.visual.append(visual)

        # Collision meshes (10 convex hulls)
        for m in range(1, 11):
            collision_url = f"package://tesseract_support/meshes/car_seat/collision/seat_{m}.stl"
            collision_path = locator.locateResource(collision_url).getFilePath()
            for mesh in createMeshFromPath(collision_path):
                collision = Collision()
                collision.origin = visual.origin
                collision.geometry = makeConvexMesh(mesh)
                link.collision.append(collision)

        # Fixed joint to world (rotated 180deg around Z, positioned along X)
        joint = Joint(f"joint_seat_{i + 1}")
        joint.parent_link_name = "world"
        joint.child_link_name = seat_name
        joint.type = JointType.FIXED
        transform = np.eye(4)
        transform[0, 0] = transform[1, 1] = -1.0  # 180deg Z rotation
        transform[:3, 3] = [0.5 + i, 2.15, 0.45]
        joint.parent_to_joint_origin_transform = Isometry3d(transform)

        if not robot.env.applyCommand(AddLinkCommand(link, joint)):
            raise RuntimeError(f"Failed to add {seat_name}")

    print("Added 3 seats to environment")


def attach_seat(robot, seat_name="seat_1"):
    """Attach seat to end effector with allowed collisions."""
    state = robot.env.getState()
    ee_tf = state.link_transforms["end_effector"]
    seat_tf = state.link_transforms[seat_name]

    # Create fixed joint: seat relative to end_effector
    joint = Joint(f"joint_{seat_name}_robot")
    joint.parent_link_name = "end_effector"
    joint.child_link_name = seat_name
    joint.type = JointType.FIXED
    relative_tf = np.linalg.inv(ee_tf.matrix()) @ seat_tf.matrix()
    joint.parent_to_joint_origin_transform = Isometry3d(relative_tf)

    if not robot.env.applyCommand(MoveLinkCommand(joint)):
        raise RuntimeError(f"Failed to attach {seat_name}")

    # Allow collisions with nearby links
    acm = AllowedCollisionMatrix()
    acm.addAllowedCollision(seat_name, "end_effector", "Adjacent")
    for link in ["cell_logo", "fence", "link_b", "link_r", "link_t"]:
        acm.addAllowedCollision(seat_name, link, "Never")

    robot.env.applyCommand(ModifyAllowedCollisionsCommand(acm, ModifyAllowedCollisionsType.ADD))
    print(f"Attached {seat_name} to end effector")


def plan_motion(robot, composer, joint_names, start_pos, end_pos, phase_name):
    """Plan a motion from start to end position."""
    program = (MotionProgram("manipulator", tcp_frame="end_effector")
        .set_joint_names(joint_names)
        .move_to(StateTarget(start_pos, names=joint_names, profile="FREESPACE"))
        .move_to(StateTarget(end_pos, names=joint_names, profile="FREESPACE"))
    )

    print(f"\n=== {phase_name} ===")
    print(f"Planning with TrajOpt...")
    result = composer.plan(robot, program, pipeline="TrajOptPipeline")

    assert result.successful, f"{phase_name} failed: {result.message}"
    print(f"{phase_name} OK: {len(result)} waypoints")
    return result


def main():
    # Load robot
    robot = Robot.from_urdf(
        "package://tesseract_support/urdf/car_seat_demo.urdf",
        "package://tesseract_support/urdf/car_seat_demo.srdf"
    )
    print(f"Loaded robot: {robot.env.getName()}")

    # Joint setup
    joint_group = robot.env.getJointGroup("manipulator")
    joint_names = list(joint_group.getJointNames())
    print(f"Joint names: {joint_names}")

    # Add seats and initialize
    add_seats(robot)

    # Convert position lists to arrays
    home_pos = np.array(POSITIONS["Home"])
    pick_pos = np.array(POSITIONS["Pick1"])
    place_pos = np.array(POSITIONS["Place1"])

    robot.set_joints(home_pos, joint_names=joint_names)

    composer = TaskComposer.from_config()

    # Phase 1: Move to pick position
    pick_result = plan_motion(robot, composer, joint_names, home_pos, pick_pos, "PICK")

    # Phase 2: Attach seat
    print("\n=== ATTACH SEAT ===")
    robot.set_joints(pick_pos, joint_names=joint_names)
    attach_seat(robot, "seat_1")

    # Phase 3: Move to place position
    place_result = plan_motion(robot, composer, joint_names, pick_pos, place_pos, "PLACE")

    # Visualize
    if TesseractViewer is not None:
        print("\nViewer at http://localhost:8000")
        viewer = TesseractViewer()
        viewer.update_environment(robot.env, [0, 0, 0])
        viewer.update_trajectory(place_result.raw_results)
        viewer.start_serve_background()
        input("Press Enter to exit...")

    print("\nDone!")


if __name__ == "__main__":
    main()
