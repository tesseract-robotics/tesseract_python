"""Tests for tesseract_task_composer bindings."""
import os
import numpy as np
import pytest

import tesseract_robotics  # triggers env var setup
from tesseract_robotics.tesseract_common import (
    FilesystemPath,
    Isometry3d,
    Translation3d,
    ManipulatorInfo,
    GeneralResourceLocator,
)
from tesseract_robotics.tesseract_environment import Environment
from tesseract_robotics.tesseract_command_language import (
    CartesianWaypoint,
    JointWaypoint,
    StateWaypoint,
    MoveInstruction,
    CompositeInstruction,
    MoveInstructionType_FREESPACE,
    ProfileDictionary,
    DEFAULT_PROFILE_KEY,
    MoveInstructionPoly_wrap_MoveInstruction,
    StateWaypointPoly_wrap_StateWaypoint,
    CartesianWaypointPoly_wrap_CartesianWaypoint,
    JointWaypointPoly_wrap_JointWaypoint,
    InstructionPoly_as_MoveInstructionPoly,
    WaypointPoly_as_StateWaypointPoly,
)
from tesseract_robotics.tesseract_task_composer import (
    TaskComposerPluginFactory,
    TaskComposerDataStorage,
    AnyPoly_wrap_CompositeInstruction,
    AnyPoly_wrap_ProfileDictionary,
    AnyPoly_wrap_EnvironmentConst,
    AnyPoly_as_CompositeInstruction,
)


TESSERACT_SUPPORT_DIR = os.environ.get("TESSERACT_SUPPORT_DIR", "")
TESSERACT_TASK_COMPOSER_CONFIG = os.environ.get("TESSERACT_TASK_COMPOSER_CONFIG_FILE", "")


@pytest.fixture
def iiwa_environment():
    """Load IIWA robot environment for testing."""
    if not TESSERACT_SUPPORT_DIR:
        pytest.skip("TESSERACT_SUPPORT_DIR not set")

    locator = GeneralResourceLocator()
    env = Environment()
    urdf_path = FilesystemPath(os.path.join(TESSERACT_SUPPORT_DIR, "urdf/lbr_iiwa_14_r820.urdf"))
    srdf_path = FilesystemPath(os.path.join(TESSERACT_SUPPORT_DIR, "urdf/lbr_iiwa_14_r820.srdf"))

    if not env.init(urdf_path, srdf_path, locator):
        pytest.skip("Failed to initialize IIWA environment")

    manip_info = ManipulatorInfo()
    manip_info.tcp_frame = "tool0"
    manip_info.manipulator = "manipulator"
    manip_info.working_frame = "base_link"

    return env, manip_info


def freespace_example_program_iiwa(manipulator_info, goal=None, freespace_profile=DEFAULT_PROFILE_KEY):
    """Create a freespace motion program for IIWA robot."""
    if goal is None:
        goal = Isometry3d.Identity() * Translation3d(0.2, 0.2, 1.0)

    program = CompositeInstruction(DEFAULT_PROFILE_KEY)
    program.setManipulatorInfo(manipulator_info)
    joint_names = ["joint_a1", "joint_a2", "joint_a3", "joint_a4", "joint_a5", "joint_a6", "joint_a7"]
    joint_values = np.zeros((7,))

    wp1 = StateWaypointPoly_wrap_StateWaypoint(StateWaypoint(joint_names, joint_values))
    start_instruction = MoveInstructionPoly_wrap_MoveInstruction(
        MoveInstruction(wp1, MoveInstructionType_FREESPACE, freespace_profile)
    )
    start_instruction.setDescription("Start Instruction")

    wp2 = CartesianWaypointPoly_wrap_CartesianWaypoint(CartesianWaypoint(goal))
    plan_f0 = MoveInstructionPoly_wrap_MoveInstruction(
        MoveInstruction(wp2, MoveInstructionType_FREESPACE, freespace_profile)
    )
    plan_f0.setDescription("freespace_motion")

    program.appendMoveInstruction(start_instruction)
    program.appendMoveInstruction(plan_f0)

    wp3 = JointWaypointPoly_wrap_JointWaypoint(JointWaypoint(joint_names, np.zeros((7,))))
    plan_f1 = MoveInstructionPoly_wrap_MoveInstruction(
        MoveInstruction(wp3, MoveInstructionType_FREESPACE)
    )
    program.appendMoveInstruction(plan_f1)

    return program


class TestTaskComposerPluginFactory:
    """Test TaskComposerPluginFactory."""

    def test_create_factory(self):
        if not TESSERACT_TASK_COMPOSER_CONFIG:
            pytest.skip("TESSERACT_TASK_COMPOSER_CONFIG_FILE not set")

        config_path = FilesystemPath(TESSERACT_TASK_COMPOSER_CONFIG)
        locator = GeneralResourceLocator()
        factory = TaskComposerPluginFactory(config_path, locator)
        assert factory is not None


class TestTaskComposerTrajOptPipeline:
    """Test TrajOpt pipeline through task composer."""

    def test_trajopt_pipeline(self, iiwa_environment):
        if not TESSERACT_TASK_COMPOSER_CONFIG:
            pytest.skip("TESSERACT_TASK_COMPOSER_CONFIG_FILE not set")

        env, manip_info = iiwa_environment

        config_path = FilesystemPath(TESSERACT_TASK_COMPOSER_CONFIG)
        locator = GeneralResourceLocator()
        factory = TaskComposerPluginFactory(config_path, locator)

        task = factory.createTaskComposerNode("TrajOptPipeline")
        assert task is not None
        print("trajopt task name: " + task.getName())

        output_key = task.getOutputKeys().get("program")
        input_key = task.getInputKeys().get("planning_input")

        profiles = ProfileDictionary()
        program = freespace_example_program_iiwa(manip_info)

        problem_anypoly = AnyPoly_wrap_CompositeInstruction(program)
        environment_anypoly = AnyPoly_wrap_EnvironmentConst(env)
        profiles_anypoly = AnyPoly_wrap_ProfileDictionary(profiles)

        task_data = TaskComposerDataStorage()
        task_data.setData(input_key, problem_anypoly)
        task_data.setData("environment", environment_anypoly)
        task_data.setData("profiles", profiles_anypoly)

        task_executor = factory.createTaskComposerExecutor("TaskflowExecutor")
        assert task_executor is not None

        future = None
        try:
            future = task_executor.run(task, task_data)
            future.wait()

            output_program = AnyPoly_as_CompositeInstruction(
                future.context.data_storage.getData(output_key)
            )
            assert len(output_program) == 11

            # Verify output program structure
            for i in range(len(output_program)):
                instr = output_program[i]
                if instr.isMoveInstruction():
                    move_instr = InstructionPoly_as_MoveInstructionPoly(instr)
                    wp = move_instr.getWaypoint()
                    if wp.isStateWaypoint():
                        state_wp = WaypointPoly_as_StateWaypointPoly(wp)
                        assert len(state_wp.getPosition()) == 7

        finally:
            # Cleanup to prevent segfault on exit
            del task_data
            del problem_anypoly
            del environment_anypoly
            del profiles_anypoly
            if future is not None:
                del future
            del task_executor
            del task
