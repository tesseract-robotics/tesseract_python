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
import gc
import numpy as np

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
    ChangeCollisionMarginsCommand,
    AnyPoly_wrap_EnvironmentConst,
)
from tesseract_robotics.tesseract_scene_graph import (
    Link,
    Joint,
    JointType,
    Visual,
    Collision,
)
from tesseract_robotics.tesseract_geometry import Box
from tesseract_robotics.tesseract_command_language import (
    CompositeInstruction,
    MoveInstruction,
    MoveInstructionType_FREESPACE,
    MoveInstructionType_LINEAR,
    StateWaypoint,
    CartesianWaypoint,
    StateWaypointPoly_wrap_StateWaypoint,
    CartesianWaypointPoly_wrap_CartesianWaypoint,
    MoveInstructionPoly_wrap_MoveInstruction,
    ProfileDictionary,
    AnyPoly_wrap_CompositeInstruction,
    AnyPoly_wrap_ProfileDictionary,
    AnyPoly_as_CompositeInstruction,
    InstructionPoly_as_MoveInstructionPoly,
)
from tesseract_robotics.tesseract_task_composer import (
    createTaskComposerPluginFactory,
    TaskComposerDataStorage,
)
from tesseract_robotics.tesseract_motion_planners import assignCurrentStateAsSeed

# Optional: viewer for visualization
TesseractViewer = None
if os.environ.get("TESSERACT_HEADLESS", "0") != "1" and "pytest" not in sys.modules:
    try:
        from tesseract_robotics_viewer import TesseractViewer
    except ImportError:
        pass

# Constants
OFFSET = 0.005
LINK_BOX_NAME = "box"
LINK_BASE_NAME = "base_link"
LINK_END_EFFECTOR_NAME = "iiwa_tool0"


def add_box(box_x, box_y, box_side):
    """Create command to add box obstacle to the environment."""
    link_box = Link(LINK_BOX_NAME)

    # Create visual
    visual = Visual()
    visual.origin = Isometry3d.Identity()
    visual.geometry = Box(box_side, box_side, box_side)
    link_box.visual.append(visual)

    # Create collision (same as visual)
    collision = Collision()
    collision.origin = visual.origin
    collision.geometry = visual.geometry
    link_box.collision.append(collision)

    # Create joint to attach box to workcell_base
    joint_box = Joint("joint_box")
    joint_box.parent_link_name = "workcell_base"
    joint_box.child_link_name = LINK_BOX_NAME
    joint_box.type = JointType.FIXED
    # Translation: position box center above table
    transform_mat = np.eye(4)
    transform_mat[:3, 3] = [box_x, box_y, box_side / 2.0 + OFFSET]
    joint_box.parent_to_joint_origin_transform = Isometry3d(transform_mat)

    return AddLinkCommand(link_box, joint_box)


def main():
    # Configuration
    box_position = [-0.2, 0.55]  # x, y position of box on table
    box_size = 0.1  # 10cm cube

    # Get config file path
    task_composer_filename = os.environ.get("TESSERACT_TASK_COMPOSER_CONFIG_FILE")
    if not task_composer_filename:
        print("Error: TESSERACT_TASK_COMPOSER_CONFIG_FILE environment variable not set")
        print("Set it to: tesseract_planning/tesseract_task_composer/config/task_composer_plugins.yaml")
        return False

    # Initialize resource locator and environment
    locator = GeneralResourceLocator()

    # Load KUKA IIWA with workcell (pick_and_place_plan includes shelves/cabinet)
    urdf_url = "package://tesseract_support/urdf/pick_and_place_plan.urdf"
    srdf_url = "package://tesseract_support/urdf/pick_and_place_plan.srdf"
    urdf_path = FilesystemPath(locator.locateResource(urdf_url).getFilePath())
    srdf_path = FilesystemPath(locator.locateResource(srdf_url).getFilePath())

    env = Environment()
    if not env.init(urdf_path, srdf_path, locator):
        print("Failed to initialize environment")
        return False

    print(f"Environment initialized: {env.getName()}")

    # Set default contact distance
    margin_cmd = ChangeCollisionMarginsCommand(0.005)
    if not env.applyCommand(margin_cmd):
        print("Failed to set collision margin")
        return False

    # Define joint names for KUKA IIWA
    joint_names = [
        "iiwa_joint_a1", "iiwa_joint_a2", "iiwa_joint_a3", "iiwa_joint_a4",
        "iiwa_joint_a5", "iiwa_joint_a6", "iiwa_joint_a7"
    ]

    # Define initial joint position
    joint_start_pos = np.array([0.0, 0.0, 0.0, -1.57, 0.0, 0.0, 0.0])

    # Set initial state
    env.setState(joint_names, joint_start_pos)

    # Add box to environment
    box_cmd = add_box(box_position[0], box_position[1], box_size)
    if not env.applyCommand(box_cmd):
        print("Failed to add box to environment")
        return False

    print(f"Added box at ({box_position[0]}, {box_position[1]}) with size {box_size}m")

    # Create task composer factory
    factory = createTaskComposerPluginFactory(task_composer_filename, locator)

    # Create executor
    executor = factory.createTaskComposerExecutor("TaskflowExecutor")

    # Create profile dictionary (use defaults)
    profiles = ProfileDictionary()

    # ==================== PICK PHASE ====================
    print("\n=== PICK PHASE ===")

    # Create manipulator info
    manip_info = ManipulatorInfo()
    manip_info.manipulator = "manipulator"
    manip_info.working_frame = LINK_BASE_NAME
    manip_info.tcp_frame = LINK_END_EFFECTOR_NAME

    # Create pick program
    pick_program = CompositeInstruction("DEFAULT")
    pick_program.setManipulatorInfo(manip_info)

    # Start waypoint
    wp_start = StateWaypoint(joint_names, joint_start_pos)
    start_instruction = MoveInstruction(
        StateWaypointPoly_wrap_StateWaypoint(wp_start),
        MoveInstructionType_FREESPACE,
        "FREESPACE"
    )
    start_instruction.setDescription("Start Instruction")

    # Define pick pose (on top of the box)
    pick_pose_mat = np.eye(4)
    pick_pose_mat[:3, :3] = np.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]])
    pick_pose_mat[:3, 3] = [box_position[0], box_position[1], box_size + 0.772 + OFFSET]
    pick_final_pose = Isometry3d(pick_pose_mat)

    # Approach pose
    approach_mat = np.array(pick_pose_mat)
    approach_mat[2, 3] += 0.15
    pick_approach_pose = Isometry3d(approach_mat)

    # Create waypoints
    pick_wp0 = CartesianWaypoint(pick_approach_pose)
    pick_wp1 = CartesianWaypoint(pick_final_pose)

    # Plan freespace to approach
    pick_plan_a0 = MoveInstruction(
        CartesianWaypointPoly_wrap_CartesianWaypoint(pick_wp0),
        MoveInstructionType_FREESPACE,
        "FREESPACE"
    )
    pick_plan_a0.setDescription("From start to pick approach")

    # Plan linear to pick position
    pick_plan_a1 = MoveInstruction(
        CartesianWaypointPoly_wrap_CartesianWaypoint(pick_wp1),
        MoveInstructionType_LINEAR,
        "CARTESIAN"
    )
    pick_plan_a1.setDescription("Pick approach")

    # Add instructions
    pick_program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(start_instruction))
    pick_program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(pick_plan_a0))
    pick_program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(pick_plan_a1))

    # Assign current state as seed for Cartesian waypoints
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
        # Explicit cleanup in correct order to prevent segfault
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

    move_cmd = MoveLinkCommand(joint_box2)
    if not env.applyCommand(move_cmd):
        print("Failed to attach box to end effector")
        return False

    # Add allowed collisions between box and nearby links
    acm = AllowedCollisionMatrix()
    acm.addAllowedCollision(LINK_BOX_NAME, LINK_END_EFFECTOR_NAME, "Never")
    acm.addAllowedCollision(LINK_BOX_NAME, "iiwa_link_7", "Never")
    acm.addAllowedCollision(LINK_BOX_NAME, "iiwa_link_6", "Never")

    acm_cmd = ModifyAllowedCollisionsCommand(acm, ModifyAllowedCollisionsType.ADD)
    if not env.applyCommand(acm_cmd):
        print("Failed to modify allowed collisions")
        return False

    print("Box attached to end effector with collision exceptions")

    # ==================== PLACE PHASE ====================
    print("\n=== PLACE PHASE ===")

    # Get final state from pick trajectory for starting place
    pick_final_instr = None
    for i in range(len(pick_results) - 1, -1, -1):
        instr = pick_results[i]
        if instr.isMoveInstruction():
            pick_final_instr = InstructionPoly_as_MoveInstructionPoly(instr)
            break

    if pick_final_instr is None:
        print("Failed to get final pick instruction")
        return False

    # Define place location (middle left shelf)
    # Quaternion(0, 0, 0.7071068, 0.7071068) = 90deg rotation around Z axis
    place_pose_mat = np.eye(4)
    # Rotation: 90deg around Z axis
    cos_45 = 0.7071068
    sin_45 = 0.7071068
    place_pose_mat[:3, :3] = np.array([
        [cos_45, -sin_45, 0],
        [sin_45, cos_45, 0],
        [0, 0, 1]
    ])
    # Translation to shelf position
    place_pose_mat[:3, 3] = [-0.148856, 0.73085, 1.16]
    place_pose = Isometry3d(place_pose_mat)

    # Approach pose for place: back off in -Y direction (in the rotated frame)
    place_approach_mat = np.array(place_pose_mat)
    offset_local = np.array([0.0, -0.25, 0])  # In place frame
    offset_world = place_pose_mat[:3, :3] @ offset_local
    place_approach_mat[:3, 3] = place_pose_mat[:3, 3] + offset_world
    place_approach_pose = Isometry3d(place_approach_mat)

    # Retreat pose (back to pick approach for simplicity)
    retreat_pose = pick_approach_pose

    # Create place program
    place_program = CompositeInstruction("DEFAULT")
    place_program.setManipulatorInfo(manip_info)

    # Waypoints
    place_wp0 = CartesianWaypoint(retreat_pose)
    place_wp1 = CartesianWaypoint(place_approach_pose)
    place_wp2 = CartesianWaypoint(place_pose)

    # Instructions: start from pick final state
    place_program.appendMoveInstruction(pick_final_instr)

    # Retreat from pick
    place_plan_a0 = MoveInstruction(
        CartesianWaypointPoly_wrap_CartesianWaypoint(place_wp0),
        MoveInstructionType_LINEAR,
        "CARTESIAN"
    )
    place_plan_a0.setDescription("Place retraction")
    place_program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(place_plan_a0))

    # Freespace to place approach
    place_plan_a1 = MoveInstruction(
        CartesianWaypointPoly_wrap_CartesianWaypoint(place_wp1),
        MoveInstructionType_FREESPACE,
        "FREESPACE"
    )
    place_plan_a1.setDescription("Place freespace")
    place_program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(place_plan_a1))

    # Linear to place position
    place_plan_a2 = MoveInstruction(
        CartesianWaypointPoly_wrap_CartesianWaypoint(place_wp2),
        MoveInstructionType_LINEAR,
        "CARTESIAN"
    )
    place_plan_a2.setDescription("Place approach")
    place_program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(place_plan_a2))

    # Assign current state as seed for Cartesian waypoints
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
        # Explicit cleanup in correct order to prevent segfault
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
        # Show place trajectory (you could combine pick + place)
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
