import re
import traceback
import os
import numpy as np
import numpy.testing as nptest

from tesseract_robotics.tesseract_common import ResourceLocator, SimpleLocatedResource
from tesseract_robotics.tesseract_environment import Environment
from tesseract_robotics.tesseract_common import FilesystemPath, Isometry3d, Translation3d, Quaterniond, \
    ManipulatorInfo, AnyPoly, AnyPoly_wrap_double
from tesseract_robotics.tesseract_command_language import CartesianWaypoint, WaypointPoly, \
    MoveInstructionType_FREESPACE, MoveInstruction, InstructionPoly, StateWaypoint, StateWaypointPoly, \
    CompositeInstruction, MoveInstructionPoly, CartesianWaypointPoly, ProfileDictionary, \
        AnyPoly_as_CompositeInstruction, CompositeInstructionOrder_ORDERED, DEFAULT_PROFILE_KEY, \
        AnyPoly_wrap_CompositeInstruction, DEFAULT_PROFILE_KEY, JointWaypoint, JointWaypointPoly, \
        InstructionPoly_as_MoveInstructionPoly, WaypointPoly_as_StateWaypointPoly, \
        MoveInstructionPoly_wrap_MoveInstruction, StateWaypointPoly_wrap_StateWaypoint, \
        CartesianWaypointPoly_wrap_CartesianWaypoint, JointWaypointPoly_wrap_JointWaypoint

# from tesseract_robotics.tesseract_motion_planners import PlannerRequest, PlannerResponse, generateInterpolatedProgram
# from tesseract_robotics.tesseract_motion_planners_ompl import OMPLDefaultPlanProfile, RRTConnectConfigurator, \
#     OMPLProblemGeneratorFn, OMPLMotionPlanner, ProfileDictionary_addProfile_OMPLPlanProfile
# from tesseract_robotics.tesseract_time_parameterization import TimeOptimalTrajectoryGeneration, \
#     InstructionsTrajectory
# from tesseract_robotics.tesseract_motion_planners_trajopt import TrajOptDefaultPlanProfile, TrajOptDefaultCompositeProfile, \
#     TrajOptProblemGeneratorFn, TrajOptMotionPlanner, ProfileDictionary_addProfile_TrajOptPlanProfile, \
#     ProfileDictionary_addProfile_TrajOptCompositeProfile
from tesseract_robotics.tesseract_task_composer import TaskComposerPluginFactory, TaskComposerProblem, \
    TaskComposerDataStorage, TaskComposerInput

from ..tesseract_support_resource_locator import TesseractSupportResourceLocator

OMPL_DEFAULT_NAMESPACE = "OMPLMotionPlannerTask"
TRAJOPT_DEFAULT_NAMESPACE = "TrajOptMotionPlannerTask"

TESSERACT_SUPPORT_DIR = os.environ["TESSERACT_SUPPORT_DIR"]
TESSERACT_TASK_COMPOSER_DIR = os.environ["TESSERACT_TASK_COMPOSER_DIR"]

def get_environment():
    locator = TesseractSupportResourceLocator()
    env = Environment()
    tesseract_support = os.environ["TESSERACT_SUPPORT_DIR"]
    urdf_path = FilesystemPath(os.path.join(tesseract_support, "urdf/lbr_iiwa_14_r820.urdf"))
    srdf_path = FilesystemPath(os.path.join(tesseract_support, "urdf/lbr_iiwa_14_r820.srdf"))
    assert env.init(urdf_path, srdf_path, locator)
    manip_info = ManipulatorInfo()
    manip_info.tcp_frame = "tool0"
    manip_info.manipulator = "manipulator"
    manip_info.working_frame = "base_link"
    
    return env, manip_info

def freespace_example_progam_iiwa(manipulator_info, goal = None, composite_profile = DEFAULT_PROFILE_KEY, 
                                freespace_profile = DEFAULT_PROFILE_KEY):
    if goal is None:
        goal = Isometry3d.Identity() * Translation3d(0.2, 0.2, 1.0)
    program = CompositeInstruction(DEFAULT_PROFILE_KEY, CompositeInstructionOrder_ORDERED, manipulator_info)
    joint_names = ["joint_a1", "joint_a2", "joint_a3", "joint_a4", "joint_a5", "joint_a6", "joint_a7"]
    joint_values = np.zeros((7,))
    wp1 = StateWaypointPoly_wrap_StateWaypoint(StateWaypoint(joint_names, joint_values))
    start_instruction = MoveInstructionPoly_wrap_MoveInstruction(MoveInstruction(wp1, MoveInstructionType_FREESPACE, freespace_profile))
    start_instruction.setDescription("Start Instruction")

    wp2 = CartesianWaypointPoly_wrap_CartesianWaypoint(CartesianWaypoint(goal))
    plan_f0 = MoveInstructionPoly_wrap_MoveInstruction(MoveInstruction(wp2, MoveInstructionType_FREESPACE, freespace_profile))
    plan_f0.setDescription("freespace_motion")
    program.appendMoveInstruction(start_instruction)
    program.appendMoveInstruction(plan_f0)

    wp3 = JointWaypointPoly_wrap_JointWaypoint(JointWaypoint(joint_names, np.zeros((7,))))
    plan_f1 = MoveInstructionPoly_wrap_MoveInstruction(MoveInstruction(wp3, MoveInstructionType_FREESPACE))
    program.appendMoveInstruction(plan_f1)

    return program


def test_task_composer_trajopt_example():
    env, manip_info = get_environment()

    config_path = FilesystemPath(os.path.join(TESSERACT_TASK_COMPOSER_DIR, "config/task_composer_plugins.yaml"))
    factory = TaskComposerPluginFactory(config_path)

    task = factory.createTaskComposerNode("TrajOptPipeline")
    
    input_key = task.getInputKeys()[0]
    output_key = task.getOutputKeys()[0]

    profiles = ProfileDictionary()

    program = freespace_example_progam_iiwa(manip_info)

    program_anypoly = AnyPoly_wrap_CompositeInstruction(program)
    task_data = TaskComposerDataStorage()
    task_data.setData(input_key, program_anypoly)

    task_problem = TaskComposerProblem(env, task_data)

    task_input = TaskComposerInput(task_problem, profiles)
    

    task_executor = factory.createTaskComposerExecutor("TaskflowExecutor")

    output_program = None
    try:
        future = task_executor.run(task.get(), task_input)
        future.wait()

        output_program = AnyPoly_as_CompositeInstruction(task_input.data_storage.getData(output_key))
        assert len(output_program) == 11

        # Print out the output program
        for i in range(len(output_program)):
            instr = output_program[i]
            if instr.isMoveInstruction():
                move_instr = InstructionPoly_as_MoveInstructionPoly(instr)
                wp = move_instr.getWaypoint()
                if wp.isStateWaypoint():
                    state_wp = WaypointPoly_as_StateWaypointPoly(wp)
                    print(f"State Waypoint {i}: {state_wp.getPosition()}")


        print("Done")
    finally:

        # Cleanup memory to prevent segfault on exit
        del task_problem
        del task_input
        del output_program
        del future
        del task_data
        del task_executor
        del task
    
