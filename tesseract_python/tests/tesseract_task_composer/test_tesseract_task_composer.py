import re
import traceback
import os
import numpy as np
import numpy.testing as nptest

from tesseract_robotics.tesseract_common import ResourceLocator, SimpleLocatedResource, ProfileDictionary, \
    AnyPoly_wrap_ProfileDictionary
from tesseract_robotics.tesseract_environment import Environment, AnyPoly_wrap_EnvironmentConst
from tesseract_robotics.tesseract_common import FilesystemPath, Isometry3d, Translation3d, Quaterniond, \
    ManipulatorInfo, AnyPoly, AnyPoly_wrap_double
from tesseract_robotics.tesseract_command_language import CartesianWaypoint, WaypointPoly, \
    MoveInstructionType_FREESPACE, MoveInstruction, InstructionPoly, StateWaypoint, StateWaypointPoly, \
    CompositeInstruction, MoveInstructionPoly, CartesianWaypointPoly, \
        AnyPoly_as_CompositeInstruction, CompositeInstructionOrder_ORDERED, DEFAULT_PROFILE_KEY, \
        AnyPoly_wrap_CompositeInstruction, DEFAULT_PROFILE_KEY, JointWaypoint, JointWaypointPoly, \
        InstructionPoly_as_MoveInstructionPoly, WaypointPoly_as_StateWaypointPoly, \
        InstructionPoly_wrap_MoveInstruction, WaypointPoly_wrap_StateWaypoint, \
        WaypointPoly_wrap_CartesianWaypoint, WaypointPoly_wrap_JointWaypoint
from tesseract_robotics import tesseract_common

# from tesseract_robotics.tesseract_motion_planners import PlannerRequest, PlannerResponse, generateInterpolatedProgram
# from tesseract_robotics.tesseract_motion_planners_ompl import OMPLDefaultMoveProfile, RRTConnectConfigurator, \
#     OMPLProblemGeneratorFn, OMPLMotionPlanner, ProfileDictionary_addProfile_OMPLMoveProfile
# from tesseract_robotics.tesseract_time_parameterization import TimeOptimalTrajectoryGeneration, \
#     InstructionsTrajectory
# from tesseract_robotics.tesseract_motion_planners_trajopt import TrajOptDefaultMoveProfile, TrajOptDefaultCompositeProfile, \
#     TrajOptProblemGeneratorFn, TrajOptMotionPlanner, ProfileDictionary_addProfile_TrajOptMoveProfile, \
#     ProfileDictionary_addProfile_TrajOptCompositeProfile
from tesseract_robotics.tesseract_task_composer import TaskComposerPluginFactory, \
    TaskComposerDataStorage, TaskComposerContext, TaskComposerDataStorageUPtr, TaskComposerLog

from ..tesseract_support_resource_locator import TesseractSupportResourceLocator

OMPL_DEFAULT_NAMESPACE = "OMPLMotionPlannerTask"
TRAJOPT_DEFAULT_NAMESPACE = "TrajOptMotionPlannerTask"

TESSERACT_SUPPORT_DIR = os.environ["TESSERACT_SUPPORT_DIR"]
TESSERACT_TASK_COMPOSER_DIR = os.environ["TESSERACT_TASK_COMPOSER_DIR"]

tesseract_common.setLogLevel(tesseract_common.CONSOLE_BRIDGE_LOG_DEBUG)

def get_environment():
    env = Environment()
    locator = TesseractSupportResourceLocator()
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
    program = CompositeInstruction(DEFAULT_PROFILE_KEY, manipulator_info, CompositeInstructionOrder_ORDERED)
    joint_names = ["joint_a1", "joint_a2", "joint_a3", "joint_a4", "joint_a5", "joint_a6", "joint_a7"]
    joint_values = np.zeros((7,))
    wp1 = WaypointPoly_wrap_StateWaypoint(StateWaypoint(joint_names, joint_values))
    start_instruction = InstructionPoly_wrap_MoveInstruction(MoveInstruction(wp1, MoveInstructionType_FREESPACE, freespace_profile))
    start_instruction.setDescription("Start Instruction")

    wp2 = WaypointPoly_wrap_CartesianWaypoint(CartesianWaypoint(goal))
    plan_f0 = InstructionPoly_wrap_MoveInstruction(MoveInstruction(wp2, MoveInstructionType_FREESPACE, freespace_profile))
    plan_f0.setDescription("freespace_motion")
    program.append(start_instruction)
    program.append(plan_f0)

    wp3 = WaypointPoly_wrap_JointWaypoint(JointWaypoint(joint_names, np.zeros((7,))))
    plan_f1 = InstructionPoly_wrap_MoveInstruction(MoveInstruction(wp3, MoveInstructionType_FREESPACE))
    program.append(plan_f1)

    return program


def test_task_composer_trajopt_example():

    output_program = None
    future = None
    task_executor = None
    task = None

    env, manip_info = get_environment()

    config_path = FilesystemPath(os.path.join(TESSERACT_TASK_COMPOSER_DIR, "config/task_composer_plugins.yaml"))
    p_locator = TesseractSupportResourceLocator()
    factory = TaskComposerPluginFactory(config_path, p_locator)

    task = factory.createTaskComposerNode("TrajOptPipeline")
    print("trajopt task name: " + task.getName())
    
    output_key = task.getOutputKeys().get("program")
    # input_key = task.getInputKeys().get("program")

    profiles = ProfileDictionary()

    program = freespace_example_progam_iiwa(manip_info)

    problem_anypoly = AnyPoly_wrap_CompositeInstruction(program)
    environment_anypoly = AnyPoly_wrap_EnvironmentConst(env)
    profiles_anypoly = AnyPoly_wrap_ProfileDictionary(profiles)

    task_data = TaskComposerDataStorage()
    task_data.setData("planning_input", problem_anypoly)
    task_data.setData("environment", environment_anypoly)
    task_data.setData("profiles", profiles_anypoly)
    log = TaskComposerLog()
    log.context = TaskComposerContext(task.getName(), task_data, True)
    
    task_executor = factory.createTaskComposerExecutor("TaskflowExecutor")    

    output_program = None
    try:
        future = task_executor.run(task.get(), log.context)
        future.wait()

        print(log.dotgraph)

        assert log.context.isSuccessful()

        output_program = AnyPoly_as_CompositeInstruction(future.context.data_storage.getData(output_key))
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
        # del planning_task_problem
        # del output_program
        del task_data
        del problem_anypoly
        del environment_anypoly
        del profiles_anypoly
        del future
        del task_executor
        del task
        
    
