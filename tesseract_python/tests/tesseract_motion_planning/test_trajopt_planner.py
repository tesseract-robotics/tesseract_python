import re
import traceback
import os
import numpy as np
import numpy.testing as nptest

from tesseract_robotics.tesseract_common import ResourceLocator, SimpleLocatedResource
from tesseract_robotics.tesseract_environment import Environment
from tesseract_robotics.tesseract_common import FilesystemPath, Isometry3d, Translation3d, Quaterniond, \
    ManipulatorInfo
from tesseract_robotics.tesseract_command_language import  JointWaypoint, CartesianWaypoint, Waypoint, \
    PlanInstructionType_FREESPACE, PlanInstructionType_START, PlanInstruction, Instruction, \
    isMoveInstruction, isStateWaypoint, CompositeInstruction, flatten, isMoveInstruction, isStateWaypoint, \
    ProfileDictionary
from tesseract_robotics.tesseract_motion_planners import PlannerRequest, PlannerResponse, generateSeed, \
    TRAJOPT_DEFAULT_NAMESPACE
from tesseract_robotics.tesseract_motion_planners_trajopt import TrajOptDefaultPlanProfile, TrajOptDefaultCompositeProfile, \
    TrajOptProblemGeneratorFn, TrajOptMotionPlanner, TrajOptMotionPlannerStatusCategory, \
    ProfileDictionary_addProfile_TrajOptPlanProfile, ProfileDictionary_addProfile_TrajOptCompositeProfile

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
    manip_info.manipulator_ik_solver = "KDLInvKinChainLMA"
    joint_names = list(env.getJointGroup("manipulator").getJointNames())

    return env, manip_info, joint_names

def test_trajopt_freespace_joint_cart():

    env, manip, joint_names = get_environment()


    cur_state = env.getState()

    wp1 = JointWaypoint(joint_names, np.array([0,0,0,-1.57,0,0,0],dtype=np.float64))
    wp2 = CartesianWaypoint(Isometry3d.Identity() * Translation3d(-.2,.4,0.2) * Quaterniond(0,0,1.0,0))

    start_instruction = PlanInstruction(Waypoint(wp1), PlanInstructionType_START, "TEST_PROFILE")
    plan_f1 = PlanInstruction(Waypoint(wp2), PlanInstructionType_FREESPACE, "TEST_PROFILE")

    program = CompositeInstruction("TEST_PROFILE")
    program.setStartInstruction(Instruction(start_instruction))
    program.setManipulatorInfo(manip)
    program.append(Instruction(plan_f1))

    seed = generateSeed(program, cur_state, env, 3.14, 1.0, 3.14, 10)

    plan_profile = TrajOptDefaultPlanProfile()
    composite_profile = TrajOptDefaultCompositeProfile()

    profiles = ProfileDictionary()
    ProfileDictionary_addProfile_TrajOptPlanProfile(profiles, TRAJOPT_DEFAULT_NAMESPACE, "TEST_PROFILE", plan_profile)
    ProfileDictionary_addProfile_TrajOptCompositeProfile(profiles, TRAJOPT_DEFAULT_NAMESPACE, "TEST_PROFILE", composite_profile)


    test_planner = TrajOptMotionPlanner()
    

    request = PlannerRequest()
    request.seed = seed
    request.instructions = program
    request.env = env
    request.env_state = cur_state
    request.profiles = profiles
    
    response = PlannerResponse()

    assert test_planner.solve(request, response)
    assert response.status.value() == TrajOptMotionPlannerStatusCategory.SolutionFound

    results = flatten(response.results)

    assert len(results) == 11
    for instr in results:
        assert isMoveInstruction(instr)
        wp1 = instr.as_MoveInstruction().getWaypoint()
        assert isStateWaypoint(wp1)
        wp = wp1.as_StateWaypoint()
        assert len(wp.joint_names) == 7
        assert isinstance(wp.position,np.ndarray)
        assert len(wp.position) == 7
