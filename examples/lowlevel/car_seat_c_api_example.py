"""
Car Seat Example

This example demonstrates:
- Dynamic mesh loading using createMeshFromPath
- Convex hull generation using makeConvexMesh
- Adding dynamically created links with meshes to the environment
- Moving links to attach objects to the robot end effector
- Multi-phase motion planning with the high-level API

Based on: tesseract_examples/src/car_seat_example.cpp
"""
import sys
import numpy as np

from tesseract_robotics.planning import (
    Robot,
    MotionProgram,
    StateTarget,
    TaskComposer,
)
from tesseract_robotics.tesseract_common import (
    GeneralResourceLocator,
    Isometry3d,
    AllowedCollisionMatrix,
)
from tesseract_robotics.tesseract_environment import (
    AddLinkCommand,
    MoveLinkCommand,
    ModifyAllowedCollisionsCommand,
    ModifyAllowedCollisionsType,
)
from tesseract_robotics.tesseract_scene_graph import (
    Link,
    Joint,
    JointType,
    Visual,
    Collision,
)
from tesseract_robotics.tesseract_geometry import createMeshFromPath
from tesseract_robotics.tesseract_collision import makeConvexMesh

TesseractViewer = None
if "pytest" not in sys.modules:
    try:
        from tesseract_robotics_viewer import TesseractViewer
    except ImportError:
        pass


def get_predefined_positions():
    """Get predefined joint positions for the car seat robot."""
    positions = {}

    positions["Default"] = {
        "carriage_rail": 1.0,
        "joint_b": 0.0,
        "joint_e": 0.0,
        "joint_l": 0.0,
        "joint_r": 0.0,
        "joint_s": -1.5707,
        "joint_t": 0.0,
        "joint_u": -1.5707,
    }

    positions["Pick1"] = {
        "carriage_rail": 2.22,
        "joint_b": 0.39,
        "joint_e": 0.0,
        "joint_l": 0.5,
        "joint_r": 0.0,
        "joint_s": -3.14,
        "joint_t": -0.29,
        "joint_u": -1.45,
    }

    positions["Pick2"] = {
        "carriage_rail": 1.22,
        "joint_b": 0.39,
        "joint_e": 0.0,
        "joint_l": 0.5,
        "joint_r": 0.0,
        "joint_s": -3.14,
        "joint_t": -0.29,
        "joint_u": -1.45,
    }

    positions["Pick3"] = {
        "carriage_rail": 0.22,
        "joint_b": 0.39,
        "joint_e": 0.0,
        "joint_l": 0.5,
        "joint_r": 0.0,
        "joint_s": -3.14,
        "joint_t": -0.29,
        "joint_u": -1.45,
    }

    positions["Place1"] = {
        "carriage_rail": 4.15466,
        "joint_b": 0.537218,
        "joint_e": 0.0189056,
        "joint_l": 0.801223,
        "joint_r": 0.0580309,
        "joint_s": -0.0481182,
        "joint_t": -0.325783,
        "joint_u": -1.2813,
    }

    positions["Home"] = {
        "carriage_rail": 0.0,
        "joint_b": 0.0,
        "joint_e": 0.0,
        "joint_l": 0.0,
        "joint_r": 0.0,
        "joint_s": 0.0,
        "joint_t": 0.0,
        "joint_u": 0.0,
    }

    return positions


def get_position_vector(joint_names, pos_dict):
    """Get joint position vector from a position dictionary."""
    return np.array([pos_dict[name] for name in joint_names])


def add_seats(robot):
    """Create and add seat links to the environment."""
    locator = robot.locator

    # Get the base path for meshes
    visual_mesh_path = locator.locateResource(
        "package://tesseract_support/meshes/car_seat/visual/seat.dae"
    ).getFilePath()

    for i in range(3):
        seat_name = f"seat_{i + 1}"
        link_seat = Link(seat_name)

        # Create visual geometry from DAE mesh
        visual = Visual()
        visual.origin = Isometry3d.Identity()

        # Load mesh from file path
        visual_meshes = createMeshFromPath(visual_mesh_path)
        if visual_meshes:
            visual.geometry = visual_meshes[0]
        link_seat.visual.append(visual)

        # Create collision geometry from STL meshes with convex hull
        for m in range(1, 11):
            collision_mesh_url = f"package://tesseract_support/meshes/car_seat/collision/seat_{m}.stl"
            collision_mesh_path = locator.locateResource(collision_mesh_url).getFilePath()

            meshes = createMeshFromPath(collision_mesh_path)
            for mesh in meshes:
                collision = Collision()
                collision.origin = visual.origin
                # Convert to convex mesh for collision
                collision.geometry = makeConvexMesh(mesh)
                link_seat.collision.append(collision)

        # Create fixed joint to attach seat to world
        joint_seat = Joint(f"joint_seat_{i + 1}")
        joint_seat.parent_link_name = "world"
        joint_seat.child_link_name = seat_name
        joint_seat.type = JointType.FIXED

        # Create transformation: rotate 180 degrees around Z axis
        transform_mat = np.eye(4)
        transform_mat[0, 0] = -1.0  # Rotation by pi around Z axis
        transform_mat[1, 1] = -1.0
        transform_mat[:3, 3] = [0.5 + i, 2.15, 0.45]  # Position along X axis
        joint_seat.parent_to_joint_origin_transform = Isometry3d(transform_mat)

        cmd = AddLinkCommand(link_seat, joint_seat)
        if not robot.env.applyCommand(cmd):
            raise RuntimeError(f"Failed to add {seat_name}")

    print(f"Added 3 seats to environment")


def attach_seat_to_effector(robot, seat_name="seat_1"):
    """Attach a seat to the end effector."""
    state = robot.env.getState()

    # Get transforms for end effector and seat
    end_effector_tf = state.link_transforms["end_effector"]
    seat_tf = state.link_transforms[seat_name]

    # Move seat to be child of end effector
    joint_seat_robot = Joint(f"joint_{seat_name}_robot")
    joint_seat_robot.parent_link_name = "end_effector"
    joint_seat_robot.child_link_name = seat_name
    joint_seat_robot.type = JointType.FIXED
    # Calculate relative transform: seat in end_effector frame
    relative_tf = np.linalg.inv(end_effector_tf.matrix()) @ seat_tf.matrix()
    joint_seat_robot.parent_to_joint_origin_transform = Isometry3d(relative_tf)

    move_cmd = MoveLinkCommand(joint_seat_robot)
    if not robot.env.applyCommand(move_cmd):
        raise RuntimeError(f"Failed to attach {seat_name}")

    # Add allowed collisions between seat and nearby links
    acm = AllowedCollisionMatrix()
    acm.addAllowedCollision(seat_name, "end_effector", "Adjacent")
    acm.addAllowedCollision(seat_name, "cell_logo", "Never")
    acm.addAllowedCollision(seat_name, "fence", "Never")
    acm.addAllowedCollision(seat_name, "link_b", "Never")
    acm.addAllowedCollision(seat_name, "link_r", "Never")
    acm.addAllowedCollision(seat_name, "link_t", "Never")

    acm_cmd = ModifyAllowedCollisionsCommand(acm, ModifyAllowedCollisionsType.ADD)
    if not robot.env.applyCommand(acm_cmd):
        raise RuntimeError("Failed to modify allowed collisions")

    print(f"Attached {seat_name} to end effector")


def main():
    # Load car seat demo robot using high-level API
    robot = Robot.from_urdf(
        "package://tesseract_support/urdf/car_seat_demo.urdf",
        "package://tesseract_support/urdf/car_seat_demo.srdf"
    )
    print(f"Loaded robot: {robot.env.getName()}")

    # Get predefined positions
    positions = get_predefined_positions()

    # Get joint names
    joint_group = robot.env.getJointGroup("manipulator")
    joint_names = list(joint_group.getJointNames())
    print(f"Joint names: {joint_names}")

    # Add seats to environment (requires low-level mesh operations)
    print("\nAdding seats to environment...")
    add_seats(robot)

    # Move to home position
    robot.set_joints(positions["Home"])

    # Create task composer
    composer = TaskComposer.from_config()

    # ==================== PICK PHASE ====================
    print("\n=== PICK SEAT 1 ===")

    start_pos = get_position_vector(joint_names, positions["Home"])
    pick_pos = get_position_vector(joint_names, positions["Pick1"])

    # High-level motion program
    pick_program = (MotionProgram("manipulator", tcp_frame="end_effector")
        .set_joint_names(joint_names)
        .move_to(StateTarget(start_pos, names=joint_names, profile="FREESPACE"))
        .move_to(StateTarget(pick_pos, names=joint_names, profile="FREESPACE"))
    )

    print(f"Pick program: {len(pick_program)} instructions")
    print("Running TrajOpt planner for PICK...")

    pick_result = composer.plan(robot, pick_program, pipeline="TrajOptPipeline")

    if not pick_result.successful:
        print(f"PICK planning failed: {pick_result.message}")
        return False

    print(f"PICK successful! {len(pick_result)} waypoints")

    # ==================== ATTACH SEAT ====================
    print("\n=== ATTACHING SEAT ===")
    robot.set_joints(positions["Pick1"])
    attach_seat_to_effector(robot, "seat_1")

    # ==================== PLACE PHASE ====================
    print("\n=== PLACE SEAT 1 ===")

    place_start_pos = get_position_vector(joint_names, positions["Pick1"])
    place_end_pos = get_position_vector(joint_names, positions["Place1"])

    place_program = (MotionProgram("manipulator", tcp_frame="end_effector")
        .set_joint_names(joint_names)
        .move_to(StateTarget(place_start_pos, names=joint_names, profile="FREESPACE"))
        .move_to(StateTarget(place_end_pos, names=joint_names, profile="FREESPACE"))
    )

    print(f"Place program: {len(place_program)} instructions")
    print("Running TrajOpt planner for PLACE...")

    place_result = composer.plan(robot, place_program, pipeline="TrajOptPipeline")

    if not place_result.successful:
        print(f"PLACE planning failed: {place_result.message}")
        return False

    print(f"PLACE successful! {len(place_result)} waypoints")

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
