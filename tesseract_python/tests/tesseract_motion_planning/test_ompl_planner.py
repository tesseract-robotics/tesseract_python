import re
import traceback
import os
import numpy as np
import numpy.testing as nptest

from tesseract_robotics.tesseract_common import ResourceLocator, SimpleLocatedResource
from tesseract_robotics.tesseract_environment import Environment
from tesseract_robotics.tesseract_common import FilesystemPath, Isometry3d, Translation3d, Quaterniond, \
    ManipulatorInfo
from tesseract_robotics.tesseract_command_language import JointWaypoint, CartesianWaypoint, WaypointPoly, \
    MoveInstructionType_FREESPACE, MoveInstruction, InstructionPoly, MoveInstructionPoly,\
    CompositeInstruction,  ProfileDictionary, CartesianWaypointPoly, JointWaypointPoly, \
    InstructionPoly_as_MoveInstructionPoly, WaypointPoly_as_StateWaypointPoly, \
    JointWaypointPoly_wrap_JointWaypoint, CartesianWaypointPoly_wrap_CartesianWaypoint, \
    MoveInstructionPoly_wrap_MoveInstruction
from tesseract_robotics.tesseract_motion_planners import PlannerRequest, PlannerResponse
from tesseract_robotics.tesseract_motion_planners_ompl import RRTConnectConfigurator, \
    OMPLMotionPlanner, OMPLRealVectorPlanProfile
from tesseract_robotics.tesseract_motion_planners_simple import generateInterpolatedProgram

from ..tesseract_support_resource_locator import TesseractSupportResourceLocator

OMPL_DEFAULT_NAMESPACE = "OMPLMotionPlannerTask"

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
    joint_names = list(env.getJointGroup("manipulator").getJointNames())

    return env, manip_info, joint_names

def test_ompl_freespace_joint_cart():

    start_state = np.array([-0.5, 0.5, 0.0, -1.3348, 0.0, 1.4959, 0.0],dtype=np.float64)
    end_state = np.array([0.5, 0.5, 0.0, -1.3348, 0.0, 1.4959, 0.0],dtype=np.float64)

    env, manip, joint_names = get_environment()
    kin_group = env.getKinematicGroup(manip.manipulator)

    wp1 = JointWaypoint(joint_names, start_state)

    goal = kin_group.calcFwdKin(end_state)[manip.tcp_frame]
    wp2 = CartesianWaypoint(goal)

    start_instruction = MoveInstruction(JointWaypointPoly_wrap_JointWaypoint(wp1), MoveInstructionType_FREESPACE, "TEST_PROFILE")
    plan_f1 = MoveInstruction(CartesianWaypointPoly_wrap_CartesianWaypoint(wp2), MoveInstructionType_FREESPACE, "TEST_PROFILE")

    program = CompositeInstruction("TEST_PROFILE")
    program.setManipulatorInfo(manip)
    program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(start_instruction))
    program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(plan_f1))

    interpolated_program = generateInterpolatedProgram(program, env, 3.14, 1.0, 3.14, 10)

    plan_profile = OMPLRealVectorPlanProfile()

    profiles = ProfileDictionary()
    profiles.addProfile(OMPL_DEFAULT_NAMESPACE, "TEST_PROFILE", plan_profile)
    
    request = PlannerRequest()
    request.instructions = interpolated_program
    request.env = env
    request.profiles = profiles
    

    ompl_planner = OMPLMotionPlanner(OMPL_DEFAULT_NAMESPACE) 
    response = PlannerResponse()

    response=ompl_planner.solve(request)
    assert response.successful
    
    results = response.results.flatten()

    assert len(results) == 11
    for instr in results:
        instr.isMoveInstruction()
        move_instr1 = InstructionPoly_as_MoveInstructionPoly(instr)
        wp1 = move_instr1.getWaypoint()
        wp1.isStateWaypoint()
        wp = WaypointPoly_as_StateWaypointPoly(wp1)
        assert len(wp.getNames()) == 7
        assert isinstance(wp.getPosition(),np.ndarray)
        assert len(wp.getPosition()) == 7

