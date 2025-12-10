"""
Car Seat Example

This example demonstrates:
- Dynamic mesh loading using createMeshFromResource
- Convex hull generation using makeConvexMesh
- Adding dynamically created links with meshes to the environment
- Moving links to attach objects to the robot end effector
- Multi-phase motion planning with TrajOpt

Based on: tesseract_examples/src/car_seat_example.cpp

Required environment variables:
- TESSERACT_RESOURCE_PATH: Path to tesseract repo (for tesseract_support)
- TESSERACT_TASK_COMPOSER_CONFIG_FILE: Path to task composer config YAML
"""
import sys
import os
import gc
import numpy as np
import math

from tesseract_robotics.tesseract_common import (
    GeneralResourceLocator,
    FilesystemPath,
    Isometry3d,
    ManipulatorInfo,
    AllowedCollisionMatrix,
)
from tesseract_robotics.tesseract_environment import (
    Environment,
    AddLinkCommand,
    MoveLinkCommand,
    ModifyAllowedCollisionsCommand,
    ModifyAllowedCollisionsType,
    AnyPoly_wrap_EnvironmentConst,
)
from tesseract_robotics.tesseract_scene_graph import (
    Link,
    Joint,
    JointType,
    Visual,
    Collision,
)
from tesseract_robotics.tesseract_geometry import (
    createMeshFromPath,
)
from tesseract_robotics.tesseract_collision import makeConvexMesh
from tesseract_robotics.tesseract_command_language import (
    CompositeInstruction,
    MoveInstruction,
    MoveInstructionType_FREESPACE,
    StateWaypoint,
    StateWaypointPoly_wrap_StateWaypoint,
    MoveInstructionPoly_wrap_MoveInstruction,
    ProfileDictionary,
    AnyPoly_wrap_CompositeInstruction,
    AnyPoly_wrap_ProfileDictionary,
    AnyPoly_as_CompositeInstruction,
)
from tesseract_robotics.tesseract_task_composer import (
    createTaskComposerPluginFactory,
    TaskComposerDataStorage,
)
from tesseract_robotics.tesseract_motion_planners import assignCurrentStateAsSeed

# Viewer (skip in pytest)
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


def add_seats(locator):
    """Create commands to add seats to the environment."""
    commands = []

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
        # Rotation by pi around Z axis
        transform_mat[0, 0] = -1.0
        transform_mat[1, 1] = -1.0
        # Translation: position seats along X axis
        transform_mat[:3, 3] = [0.5 + i, 2.15, 0.45]
        joint_seat.parent_to_joint_origin_transform = Isometry3d(transform_mat)

        commands.append(AddLinkCommand(link_seat, joint_seat))

    return commands


def main():
    # Get config file path
    task_composer_filename = os.environ.get("TESSERACT_TASK_COMPOSER_CONFIG_FILE")
    if not task_composer_filename:
        print("Error: TESSERACT_TASK_COMPOSER_CONFIG_FILE environment variable not set")
        print("Set it to: tesseract_planning/tesseract_task_composer/config/task_composer_plugins.yaml")
        return False

    # Initialize resource locator and environment
    locator = GeneralResourceLocator()

    # Load car seat demo robot
    urdf_url = "package://tesseract_support/urdf/car_seat_demo.urdf"
    srdf_url = "package://tesseract_support/urdf/car_seat_demo.srdf"
    urdf_path = FilesystemPath(locator.locateResource(urdf_url).getFilePath())
    srdf_path = FilesystemPath(locator.locateResource(srdf_url).getFilePath())

    env = Environment()
    if not env.init(urdf_path, srdf_path, locator):
        print("Failed to initialize environment")
        return False

    print(f"Environment initialized: {env.getName()}")

    # Get predefined positions
    positions = get_predefined_positions()

    # Get joint names from manipulator
    joint_group = env.getJointGroup("manipulator")
    joint_names = list(joint_group.getJointNames())
    print(f"Joint names: {joint_names}")

    # Create seats and add to environment
    print("\nAdding seats to environment...")
    seat_commands = add_seats(locator)
    for cmd in seat_commands:
        if not env.applyCommand(cmd):
            print("Failed to add seat to environment")
            return False
    print(f"Added {len(seat_commands)} seats")

    # Move to home position
    env.setState(positions["Home"])

    # Create task composer factory
    factory = createTaskComposerPluginFactory(task_composer_filename, locator)

    # Create executor
    executor = factory.createTaskComposerExecutor("TaskflowExecutor")

    # Create profile dictionary (use defaults)
    profiles = ProfileDictionary()

    # ==================== PICK PHASE ====================
    print("\n=== PICK SEAT 1 ===")

    # Create manipulator info
    manip_info = ManipulatorInfo()
    manip_info.manipulator = "manipulator"
    manip_info.working_frame = "world"
    manip_info.tcp_frame = "end_effector"

    # Create pick program
    pick_program = CompositeInstruction("FREESPACE")
    pick_program.setManipulatorInfo(manip_info)
    pick_program.setDescription("Pick up the first seat!")

    # Start and end positions
    start_pos = get_position_vector(joint_names, positions["Home"])
    pick_pos = get_position_vector(joint_names, positions["Pick1"])

    # Start waypoint
    wp_start = StateWaypoint(joint_names, start_pos)
    start_instruction = MoveInstruction(
        StateWaypointPoly_wrap_StateWaypoint(wp_start),
        MoveInstructionType_FREESPACE,
        "FREESPACE"
    )
    start_instruction.setDescription("Start Instruction")

    # Pick waypoint
    wp_pick = StateWaypoint(joint_names, pick_pos)
    pick_instruction = MoveInstruction(
        StateWaypointPoly_wrap_StateWaypoint(wp_pick),
        MoveInstructionType_FREESPACE,
        "FREESPACE"
    )
    pick_instruction.setDescription("Freespace pick seat 1")

    # Add instructions
    pick_program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(start_instruction))
    pick_program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(pick_instruction))

    # Assign current state as seed
    assignCurrentStateAsSeed(pick_program, env)

    print(f"Pick program: {len(pick_program)} instructions")

    # Create and run pick task
    pick_task = factory.createTaskComposerNode("TrajOptPipeline")
    pick_output_key = pick_task.getOutputKeys().get("program")
    pick_input_key = pick_task.getInputKeys().get("planning_input")

    pick_data = TaskComposerDataStorage()
    pick_data.setData(pick_input_key, AnyPoly_wrap_CompositeInstruction(pick_program))
    pick_data.setData("environment", AnyPoly_wrap_EnvironmentConst(env))
    pick_data.setData("profiles", AnyPoly_wrap_ProfileDictionary(profiles))

    print("Running TrajOpt planner for PICK...")
    pick_future = executor.run(pick_task, pick_data)
    pick_future.wait()

    if not pick_future.context.isSuccessful():
        print("PICK planning failed!")
        del pick_future
        del pick_data
        del pick_task
        del executor
        del factory
        gc.collect()
        return False

    print("PICK planning successful!")

    # Get pick results
    pick_results = AnyPoly_as_CompositeInstruction(
        pick_future.context.data_storage.getData(pick_output_key)
    )
    print(f"Pick trajectory has {len(pick_results)} waypoints")

    # ==================== ATTACH SEAT TO END EFFECTOR ====================
    print("\n=== ATTACHING SEAT ===")

    # Get the state at pick position
    env.setState(positions["Pick1"])
    state = env.getState()

    # Get transforms for end effector and seat
    end_effector_tf = state.link_transforms["end_effector"]
    seat_tf = state.link_transforms["seat_1"]

    # Move seat to be child of end effector
    joint_seat_robot = Joint("joint_seat_1_robot")
    joint_seat_robot.parent_link_name = "end_effector"
    joint_seat_robot.child_link_name = "seat_1"
    joint_seat_robot.type = JointType.FIXED
    # Calculate relative transform: seat in end_effector frame
    # Use .matrix() to get numpy arrays from Isometry3d objects
    relative_tf = np.linalg.inv(end_effector_tf.matrix()) @ seat_tf.matrix()
    joint_seat_robot.parent_to_joint_origin_transform = Isometry3d(relative_tf)

    move_cmd = MoveLinkCommand(joint_seat_robot)
    if not env.applyCommand(move_cmd):
        print("Failed to attach seat to end effector")
        return False

    # Add allowed collisions between seat and nearby links
    acm = AllowedCollisionMatrix()
    acm.addAllowedCollision("seat_1", "end_effector", "Adjacent")
    acm.addAllowedCollision("seat_1", "cell_logo", "Never")
    acm.addAllowedCollision("seat_1", "fence", "Never")
    acm.addAllowedCollision("seat_1", "link_b", "Never")
    acm.addAllowedCollision("seat_1", "link_r", "Never")
    acm.addAllowedCollision("seat_1", "link_t", "Never")

    acm_cmd = ModifyAllowedCollisionsCommand(acm, ModifyAllowedCollisionsType.ADD)
    if not env.applyCommand(acm_cmd):
        print("Failed to modify allowed collisions")
        return False

    print("Seat attached to end effector with collision exceptions")

    # ==================== PLACE PHASE ====================
    print("\n=== PLACE SEAT 1 ===")

    # Create place program
    place_program = CompositeInstruction("FREESPACE")
    place_program.setManipulatorInfo(manip_info)
    place_program.setDescription("Place the first seat!")

    # Start and end positions
    place_start_pos = get_position_vector(joint_names, positions["Pick1"])
    place_end_pos = get_position_vector(joint_names, positions["Place1"])

    # Start waypoint
    wp_place_start = StateWaypoint(joint_names, place_start_pos)
    place_start_instruction = MoveInstruction(
        StateWaypointPoly_wrap_StateWaypoint(wp_place_start),
        MoveInstructionType_FREESPACE,
        "FREESPACE"
    )
    place_start_instruction.setDescription("Start Instruction")

    # Place waypoint
    wp_place = StateWaypoint(joint_names, place_end_pos)
    place_instruction = MoveInstruction(
        StateWaypointPoly_wrap_StateWaypoint(wp_place),
        MoveInstructionType_FREESPACE,
        "FREESPACE"
    )
    place_instruction.setDescription("Freespace place seat 1")

    # Add instructions
    place_program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(place_start_instruction))
    place_program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(place_instruction))

    # Assign current state as seed
    assignCurrentStateAsSeed(place_program, env)

    print(f"Place program: {len(place_program)} instructions")

    # Create and run place task
    place_task = factory.createTaskComposerNode("TrajOptPipeline")
    place_output_key = place_task.getOutputKeys().get("program")
    place_input_key = place_task.getInputKeys().get("planning_input")

    place_data = TaskComposerDataStorage()
    place_data.setData(place_input_key, AnyPoly_wrap_CompositeInstruction(place_program))
    place_data.setData("environment", AnyPoly_wrap_EnvironmentConst(env))
    place_data.setData("profiles", AnyPoly_wrap_ProfileDictionary(profiles))

    print("Running TrajOpt planner for PLACE...")
    place_future = executor.run(place_task, place_data)
    place_future.wait()

    if not place_future.context.isSuccessful():
        print("PLACE planning failed!")
        del pick_future, place_future
        del pick_data, place_data
        del pick_task, place_task
        del executor
        del factory
        gc.collect()
        return False

    print("PLACE planning successful!")

    # Get place results
    place_results = AnyPoly_as_CompositeInstruction(
        place_future.context.data_storage.getData(place_output_key)
    )
    print(f"Place trajectory has {len(place_results)} waypoints")

    # Optional: visualize with viewer
    if TesseractViewer is not None:
        print("\nStarting viewer at http://localhost:8000")
        viewer = TesseractViewer()
        viewer.update_environment(env, [0, 0, 0])
        viewer.update_trajectory(place_results)
        viewer.start_serve_background()
        input("Press Enter to exit...")

    # Explicit cleanup to prevent segfault at interpreter shutdown
    del pick_future, place_future
    del pick_data, place_data
    del pick_task, place_task
    del executor
    del factory
    gc.collect()

    print("\nDone!")
    return True


if __name__ == "__main__":
    success = main()
    exit(0 if success else 1)
