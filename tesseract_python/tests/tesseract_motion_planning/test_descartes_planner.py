from pathlib import Path
import re
import traceback
import os
import numpy as np
import numpy.testing as nptest

from tesseract_robotics.tesseract_common import SimpleResourceLocator, SimpleResourceLocatorFn
from tesseract_robotics.tesseract_environment import Environment
from tesseract_robotics.tesseract_common import FilesystemPath, Isometry3d, Translation3d, Quaterniond, \
    ManipulatorInfo
from tesseract_robotics.tesseract_command_language import JointWaypoint, CartesianWaypoint, Waypoint, \
    PlanInstructionType_LINEAR, PlanInstructionType_START, PlanInstruction, Instruction, \
    isMoveInstruction, isStateWaypoint, CompositeInstruction, flatten, isMoveInstruction, isStateWaypoint, \
    ProfileDictionary
from tesseract_robotics.tesseract_motion_planners import PlannerRequest, PlannerResponse, generateSeed, \
    DESCARTES_DEFAULT_NAMESPACE
from tesseract_robotics.tesseract_motion_planners_descartes import DescartesDefaultPlanProfileD, \
    DescartesMotionPlannerD, DescartesMotionPlannerStatusCategory, DescartesPlanProfileD, \
    ProfileDictionary_addProfile_DescartesPlanProfileD, cast_DescartesPlanProfileD

def _locate_resource(url):
    try:
        try:
            if Path(url).exists():
                return url
        except:
            pass
        url_match = re.match(r"^package:\/\/tesseract_support\/(.*)$",url)
        if (url_match is None):
            return ""    
        if not "TESSERACT_SUPPORT_DIR" in os.environ:
            return ""
        tesseract_support = os.environ["TESSERACT_SUPPORT_DIR"]
        return os.path.join(tesseract_support, os.path.normpath(url_match.group(1)))
    except:
        traceback.print_exc()

def get_environment():
    locate_resource_fn = SimpleResourceLocatorFn(_locate_resource)
    locator = SimpleResourceLocator(locate_resource_fn)
    env = Environment()
    tesseract_support = os.environ["TESSERACT_SUPPORT_DIR"]
    urdf_path = FilesystemPath(os.path.join(tesseract_support, "urdf/abb_irb2400.urdf"))
    srdf_path = FilesystemPath(os.path.join(tesseract_support, "urdf/abb_irb2400.srdf"))
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

    cur_state = env.getState()

    wp1 = CartesianWaypoint(Isometry3d.Identity() * Translation3d(0.8,-0.2,0.8) * Quaterniond(0,0,-1.0,0))
    wp2 = CartesianWaypoint(Isometry3d.Identity() * Translation3d(0.8,0.2,0.8) * Quaterniond(0,0,-1.0,0))

    start_instruction = PlanInstruction(Waypoint(wp1), PlanInstructionType_START, "TEST_PROFILE", manip)
    plan_f1 = PlanInstruction(Waypoint(wp2), PlanInstructionType_LINEAR, "TEST_PROFILE", manip)

    program = CompositeInstruction()
    program.setStartInstruction(Instruction(start_instruction))
    program.setManipulatorInfo(manip)
    program.append(Instruction(plan_f1))

    seed = generateSeed(program, cur_state, env, 3.14, 1.0, 3.14, 10)

    plan_profile = DescartesDefaultPlanProfileD()
    # DescartesDefaultPlanProfileD is not upcasting automatically, use helper function
    plan_profile1 = cast_DescartesPlanProfileD(plan_profile)

    profiles = ProfileDictionary()
    ProfileDictionary_addProfile_DescartesPlanProfileD(profiles,DESCARTES_DEFAULT_NAMESPACE,"TEST_PROFILE",plan_profile1)
    
    single_descartes_planner = DescartesMotionPlannerD()
    plan_profile.num_threads = 1
    

    request = PlannerRequest()
    request.seed = seed
    request.instructions = program
    request.env = env
    request.env_state = cur_state
    request.profiles = profiles
    
    response = PlannerResponse()

    assert single_descartes_planner.solve(request, response)
    assert response.status.value() == DescartesMotionPlannerStatusCategory.SolutionFound

    results = flatten(response.results)

    assert len(results) == 11
    for instr in results:
        assert isMoveInstruction(instr)
        wp1 = instr.as_MoveInstruction().getWaypoint()
        assert isStateWaypoint(wp1)
        wp = wp1.as_StateWaypoint()
        assert len(wp.joint_names) == 6
        assert isinstance(wp.position,np.ndarray)
        assert len(wp.position) == 6
