import re
import traceback
import os
import numpy as np

from tesseract_robotics.tesseract_common import ResourceLocator, SimpleLocatedResource
from tesseract_robotics.tesseract_environment import Environment
from tesseract_robotics.tesseract_common import FilesystemPath, ManipulatorInfo
from tesseract_robotics.tesseract_command_language import JointWaypoint, CartesianWaypoint, Waypoint, \
    MoveInstructionType_FREESPACE, MoveInstructionType_START, MoveInstruction, Instruction, \
    NullInstruction, MoveInstruction, isMoveInstruction, isStateWaypoint
from tesseract_robotics.tesseract_motion_planners import PlannerRequest
from tesseract_robotics.tesseract_motion_planners_simple import SimplePlannerLVSPlanProfile

from ..tesseract_support_resource_locator import TesseractSupportResourceLocator

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

def test_get_environment():
    get_environment()

def test_interpolatestatewaypoint_jointcart_freespace():
    env, manip_info, joint_names = get_environment()

    request = PlannerRequest()
    request.env = env
    request.env_state = env.getState()
    joint_group = env.getJointGroup(manip_info.manipulator)
    wp1 = JointWaypoint(joint_names, np.zeros((7,),dtype=np.float64))
    wp1_seed = JointWaypoint(joint_names, request.env_state.getJointValues(joint_names))
    wp2 = CartesianWaypoint(joint_group.calcFwdKin(np.ones((7,),dtype=np.float64))[manip_info.tcp_frame])
    instr1 = MoveInstruction(Waypoint(wp1), MoveInstructionType_START, "TEST_PROFILE", manip_info)
    instr1_seed = MoveInstruction(Waypoint(wp1), MoveInstructionType_START, "TEST_PROFILE", manip_info)
    instr1_seed.setWaypoint(Waypoint(wp1_seed))
    instr2 = MoveInstruction(Waypoint(wp2), MoveInstructionType_FREESPACE, "TEST_PROFILE", manip_info)
    instr3 = Instruction(NullInstruction())

    profile = SimplePlannerLVSPlanProfile(3.14,0.5,1.57,5)
    composite = profile.generate(instr1,instr1_seed,instr2,instr3,request,manip_info)

    for c in composite:
        assert isMoveInstruction(c)
        assert isStateWaypoint(c.as_MoveInstruction().getWaypoint())
        # assert c.as_MoveInstruction().getProfile() == instr2.getProfile()

    mi = composite[-1].as_const_MoveInstruction()
    last_position = mi.getWaypoint().as_const_StateWaypoint().position
    final_pose = joint_group.calcFwdKin(last_position)[manip_info.tcp_frame]
    assert wp2.isApprox(final_pose, 1e-3)
