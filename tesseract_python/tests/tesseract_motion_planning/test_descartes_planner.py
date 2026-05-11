import re
import traceback
import os
import numpy as np
import numpy.testing as nptest

from tesseract_robotics.tesseract_common import ResourceLocator, SimpleLocatedResource, ProfileDictionary
from tesseract_robotics.tesseract_environment import Environment
from tesseract_robotics.tesseract_common import FilesystemPath, Isometry3d, Translation3d, Quaterniond, \
    ManipulatorInfo, GeneralResourceLocator
from tesseract_robotics.tesseract_command_language import JointWaypoint, CartesianWaypoint, WaypointPoly, \
    MoveInstructionType_LINEAR, MoveInstruction, InstructionPoly, CartesianWaypointPoly, MoveInstructionPoly, \
    CompositeInstruction, InstructionPoly_as_MoveInstructionPoly, WaypointPoly_as_StateWaypointPoly,\
    WaypointPoly_wrap_CartesianWaypoint, InstructionPoly_wrap_MoveInstruction
from tesseract_robotics.tesseract_motion_planners import PlannerRequest, PlannerResponse
from tesseract_robotics.tesseract_motion_planners_descartes import DescartesDefaultMoveProfileD, \
    DescartesMotionPlannerD, DescartesMoveProfileD, cast_DescartesMoveProfileD
from tesseract_robotics.tesseract_motion_planners_simple import generateInterpolatedProgram

DESCARTES_DEFAULT_NAMESPACE = "DescartesMotionPlannerTask"

def get_environment():
    locator = GeneralResourceLocator()
    env = Environment()    
    urdf_path = FilesystemPath(locator.locateResource("package://tesseract/support/urdf/abb_irb2400.urdf").getFilePath())
    srdf_path = FilesystemPath(locator.locateResource("package://tesseract/support/urdf/abb_irb2400.srdf").getFilePath())
    assert env.init(urdf_path, srdf_path, locator)
    manip_info = ManipulatorInfo()
    manip_info.tcp_frame = "tool0"
    manip_info.manipulator = "manipulator"
    manip_info.manipulator_ik_solver = "OPWInvKin"
    manip_info.working_frame = "base_link"
    joint_names = list(env.getJointGroup("manipulator").getJointNames())
    
    return env, manip_info, joint_names

def test_descartes_freespace_fixed_poses():

    env, manip, joint_names = get_environment()
    kin_group = env.getKinematicGroup(manip.manipulator,manip.manipulator_ik_solver)

    wp1 = CartesianWaypoint(Isometry3d.Identity() * Translation3d(0.8,-0.2,0.8) * Quaterniond(0,0,-1.0,0))
    wp2 = CartesianWaypoint(Isometry3d.Identity() * Translation3d(0.8,0.2,0.8) * Quaterniond(0,0,-1.0,0))

    start_instruction = MoveInstruction(WaypointPoly_wrap_CartesianWaypoint(wp1), MoveInstructionType_LINEAR, "TEST_PROFILE", manip)
    plan_f1 = MoveInstruction(WaypointPoly_wrap_CartesianWaypoint(wp2), MoveInstructionType_LINEAR, "TEST_PROFILE", manip)

    program = CompositeInstruction()
    program.setManipulatorInfo(manip)
    program.append(InstructionPoly_wrap_MoveInstruction(start_instruction))
    program.append(InstructionPoly_wrap_MoveInstruction(plan_f1))

    interpolated_program = generateInterpolatedProgram(program, env, 3.14, 1.0, 3.14, 10)

    plan_profile = DescartesDefaultMoveProfileD()
    # DescartesDefaultMoveProfileD is not upcasting automatically, use helper function
    plan_profile1 = cast_DescartesMoveProfileD(plan_profile)

    profiles = ProfileDictionary()
    profiles.addProfile(DESCARTES_DEFAULT_NAMESPACE,"TEST_PROFILE",plan_profile1)
    
    single_descartes_planner = DescartesMotionPlannerD(DESCARTES_DEFAULT_NAMESPACE)
    

    request = PlannerRequest()
    request.instructions = interpolated_program
    request.env = env
    request.profiles = profiles
    
    response = single_descartes_planner.solve(request)
    assert response.successful

    results = response.results.flatten()

    assert len(results) == 11
    for instr in results:
        # assert isMoveInstruction(instr)
        move_instr=InstructionPoly_as_MoveInstructionPoly(instr)
        wp1 = move_instr.getWaypoint()
        # assert isStateWaypoint(wp1)
        wp = WaypointPoly_as_StateWaypointPoly(wp1)
        assert len(wp.getNames()) == 6
        assert isinstance(wp.getPosition(),np.ndarray)
        assert len(wp.getPosition()) == 6
