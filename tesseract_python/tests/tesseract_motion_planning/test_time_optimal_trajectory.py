from tesseract_robotics.tesseract_command_language import CompositeInstruction, StateWaypoint, WaypointPoly, \
    InstructionPoly, MoveInstruction, Instructions, MoveInstructionType_FREESPACE, StateWaypointPoly,\
    MoveInstructionPoly, InstructionPoly_as_MoveInstructionPoly, WaypointPoly_as_StateWaypointPoly, \
    InstructionPoly_wrap_MoveInstruction, WaypointPoly_wrap_StateWaypoint, DEFAULT_PROFILE_KEY

from tesseract_robotics.tesseract_time_parameterization import TimeOptimalTrajectoryGeneration, \
    InstructionsTrajectory, TimeOptimalTrajectoryGenerationCompositeProfile 
from tesseract_robotics.tesseract_environment import Environment
from tesseract_robotics.tesseract_common import FilesystemPath, ManipulatorInfo, ProfileDictionary
from ..tesseract_support_resource_locator import TesseractSupportResourceLocator
import numpy as np
import os

def get_environment():
    env = Environment()
    locator = TesseractSupportResourceLocator()
    tesseract_support = os.environ["TESSERACT_SUPPORT_DIR"]
    urdf_path = FilesystemPath(os.path.join(tesseract_support, "urdf/lbr_iiwa_14_r820.urdf"))
    srdf_path = FilesystemPath(os.path.join(tesseract_support, "urdf/lbr_iiwa_14_r820.srdf"))
    assert env.init(urdf_path, srdf_path, locator)
    manip_info = ManipulatorInfo()
    manip_info.manipulator = "manipulator"
    manip_info.tcp_frame = "tool0"
    manip_info.working_frame = "base_link"
    joint_names = list(env.getJointGroup("manipulator").getJointNames())

    return env, manip_info, joint_names

def create_straight_trajectory():

    num = 10
    max_ = 2.0
    joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7"]

    program = CompositeInstruction()

    for i in range(num):
        p = np.zeros((7,),dtype=np.float64)
        p[0] = i * (max_/num)
        swp = StateWaypoint(joint_names, p)        
        program.append(InstructionPoly_wrap_MoveInstruction(MoveInstruction(
            WaypointPoly_wrap_StateWaypoint(swp), MoveInstructionType_FREESPACE)))

    p = np.zeros((7,),dtype=np.float64)
    p[0] = max_
    swp = StateWaypoint(joint_names, p)
    program.append(InstructionPoly_wrap_MoveInstruction(MoveInstruction(
            WaypointPoly_wrap_StateWaypoint(swp), MoveInstructionType_FREESPACE)))

    return program

def test_time_parameterization():

    time_parameterization = TimeOptimalTrajectoryGeneration("test")

    program = create_straight_trajectory()
    env = get_environment()
    program.setManipulatorInfo(env[1])
    max_velocity = np.array([[2.088, 2.082, 3.27, 3.6, 3.3, 3.078]],dtype=np.float64)
    max_velocity = np.hstack((-max_velocity.T, max_velocity.T))
    max_acceleration = np.array([[ 1, 1, 1, 1, 1, 1, 1]],dtype=np.float64)
    max_acceleration = np.hstack((-max_acceleration.T, max_acceleration.T))
    max_jerk = np.array([[ 1, 1, 1, 1, 1, 1, 1]],dtype=np.float64)
    max_jerk = np.hstack((-max_jerk.T, max_jerk.T))
    profile = TimeOptimalTrajectoryGenerationCompositeProfile()
    profile.velocity_limits = max_velocity
    profile.acceleration_limits = max_velocity
    profile.override_limits = True
    profiles = ProfileDictionary()
    profiles.addProfile("unit_test","TEST_PROFILE",profile)
    assert time_parameterization.compute(program, env[0], profiles)
    instr1 = program[-1]
    instr1_1 = InstructionPoly_as_MoveInstructionPoly(instr1)
    result_wp1 = instr1_1.getWaypoint()
    assert WaypointPoly_as_StateWaypointPoly(result_wp1).getTime() > 1.0
    instr1_2 = InstructionPoly_as_MoveInstructionPoly(program[-1])
    assert WaypointPoly_as_StateWaypointPoly(instr1_2.getWaypoint()).getTime() < 7.0
