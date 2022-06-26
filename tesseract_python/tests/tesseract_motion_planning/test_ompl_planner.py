import re
import traceback
import os
import numpy as np
import numpy.testing as nptest

from tesseract_robotics.tesseract_common import ResourceLocator, SimpleLocatedResource
from tesseract_robotics.tesseract_environment import Environment
from tesseract_robotics.tesseract_common import FilesystemPath, Isometry3d, Translation3d, Quaterniond, \
    ManipulatorInfo
from tesseract_robotics.tesseract_command_language import JointWaypoint, CartesianWaypoint, Waypoint, \
    PlanInstructionType_FREESPACE, PlanInstructionType_START, PlanInstruction, Instruction, \
    isMoveInstruction, isStateWaypoint, CompositeInstruction, flatten, isMoveInstruction, isStateWaypoint, \
    ProfileDictionary
from tesseract_robotics.tesseract_motion_planners import PlannerRequest, PlannerResponse, generateSeed, \
    OMPL_DEFAULT_NAMESPACE
from tesseract_robotics.tesseract_motion_planners_ompl import OMPLDefaultPlanProfile, RRTConnectConfigurator, \
    OMPLProblemGeneratorFn, OMPLMotionPlanner, OMPLMotionPlannerStatusCategory, \
    ProfileDictionary_addProfile_OMPLPlanProfile

from ..tesseract_support_resource_locator import TesseractSupportResourceLocator

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

    cur_state = env.getState()

    wp1 = JointWaypoint(joint_names, start_state)

    goal = kin_group.calcFwdKin(end_state)[manip.tcp_frame]
    wp2 = CartesianWaypoint(goal)

    start_instruction = PlanInstruction(Waypoint(wp1), PlanInstructionType_START, "TEST_PROFILE")
    plan_f1 = PlanInstruction(Waypoint(wp2), PlanInstructionType_FREESPACE, "TEST_PROFILE")

    program = CompositeInstruction("TEST_PROFILE")
    program.setStartInstruction(Instruction(start_instruction))
    program.setManipulatorInfo(manip)
    program.append(Instruction(plan_f1))

    seed = generateSeed(program, cur_state, env, 3.14, 1.0, 3.14, 10)

    plan_profile = OMPLDefaultPlanProfile()
    plan_profile.planners.clear()
    plan_profile.planners.append(RRTConnectConfigurator())

    profiles = ProfileDictionary()
    ProfileDictionary_addProfile_OMPLPlanProfile(profiles,OMPL_DEFAULT_NAMESPACE, "TEST_PROFILE", plan_profile)
    
    request = PlannerRequest()
    request.instructions = program
    request.seed = seed    
    request.env = env
    request.env_state = cur_state
    request.profiles = profiles
    

    ompl_planner = OMPLMotionPlanner()    
    response = PlannerResponse()

    assert ompl_planner.solve(request, response)
    assert response.status.value() == OMPLMotionPlannerStatusCategory.SolutionFound

    results = flatten(response.results)

    assert len(results) == 11
    for instr in results:
        assert isMoveInstruction(instr)
        move_instr1 = instr.as_MoveInstruction()
        wp1 = move_instr1.getWaypoint()
        assert isStateWaypoint(wp1)
        wp = wp1.as_StateWaypoint()
        assert len(wp.joint_names) == 7
        assert isinstance(wp.position,np.ndarray)
        assert len(wp.position) == 7

