import re
import traceback
import os
import numpy as np
import numpy.testing as nptest

from tesseract_robotics.tesseract_common import ResourceLocator, SimpleLocatedResource
from tesseract_robotics.tesseract_environment import Environment
from tesseract_robotics.tesseract_common import FilesystemPath, Isometry3d, Translation3d, Quaterniond, \
    ManipulatorInfo
from tesseract_robotics.tesseract_command_language import  JointWaypoint, CartesianWaypoint, WaypointPoly, \
    MoveInstructionType_FREESPACE, MoveInstruction, InstructionPoly, \
    CompositeInstruction, ProfileDictionary, CartesianWaypointPoly, JointWaypointPoly, MoveInstructionPoly, \
    InstructionPoly_as_MoveInstructionPoly, WaypointPoly_as_StateWaypointPoly, \
    JointWaypointPoly_wrap_JointWaypoint, CartesianWaypointPoly_wrap_CartesianWaypoint, \
    MoveInstructionPoly_wrap_MoveInstruction
from tesseract_robotics.tesseract_motion_planners import PlannerRequest, PlannerResponse
from tesseract_robotics.tesseract_motion_planners_trajopt import TrajOptDefaultPlanProfile, TrajOptDefaultCompositeProfile, \
    TrajOptMotionPlanner
from tesseract_robotics.tesseract_motion_planners_simple import generateInterpolatedProgram

from ..tesseract_support_resource_locator import TesseractSupportResourceLocator

TRAJOPT_DEFAULT_NAMESPACE = "TrajOptMotionPlannerTask"

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
    manip_info.manipulator_ik_solver = "KDLInvKinChainLMA"
    joint_names = list(env.getJointGroup("manipulator").getJointNames())

    return env, manip_info, joint_names

def test_trajopt_freespace_joint_cart():

    env, manip, joint_names = get_environment()

    wp1 = JointWaypoint(joint_names, np.array([0,0,0,-1.57,0,0,0],dtype=np.float64))
    wp2 = CartesianWaypoint(Isometry3d.Identity() * Translation3d(-.2,.4,0.2) * Quaterniond(0,0,1.0,0))

    start_instruction = MoveInstruction(JointWaypointPoly_wrap_JointWaypoint(wp1), MoveInstructionType_FREESPACE, "TEST_PROFILE")
    plan_f1 = MoveInstruction(CartesianWaypointPoly_wrap_CartesianWaypoint(wp2), MoveInstructionType_FREESPACE, "TEST_PROFILE")

    program = CompositeInstruction("TEST_PROFILE")
    program.setManipulatorInfo(manip)
    program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(start_instruction))
    program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(plan_f1))

    interpolated_program = generateInterpolatedProgram(program, env, 3.14, 1.0, 3.14, 10)

    plan_profile = TrajOptDefaultPlanProfile()
    composite_profile = TrajOptDefaultCompositeProfile()

    profiles = ProfileDictionary()
    profiles.addProfile(TRAJOPT_DEFAULT_NAMESPACE, "TEST_PROFILE", plan_profile)
    profiles.addProfile(TRAJOPT_DEFAULT_NAMESPACE, "TEST_PROFILE", composite_profile)


    test_planner = TrajOptMotionPlanner(TRAJOPT_DEFAULT_NAMESPACE)
    

    request = PlannerRequest()
    request.instructions = interpolated_program
    request.env = env
    request.profiles = profiles
    
    response = test_planner.solve(request)
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
