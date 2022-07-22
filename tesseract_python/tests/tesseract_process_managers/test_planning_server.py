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
    MoveInstructionType_FREESPACE, MoveInstructionType_START, MoveInstruction, Instruction, \
    isMoveInstruction, isStateWaypoint, CompositeInstruction, flatten, isMoveInstruction, isStateWaypoint
from tesseract_robotics.tesseract_process_managers import ProcessPlanningServer, ProcessPlanningRequest, \
    FREESPACE_PLANNER_NAME

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
    
    return env, manip_info

def test_planning_server_freespace():

    env, manip = get_environment()

    joint_names = ["joint_a1", "joint_a2", "joint_a3", "joint_a4", "joint_a5", "joint_a6", "joint_a7"]

    wp1 = JointWaypoint(joint_names, np.array([0,0,0,-1.57,0,0,0],dtype=np.float64))
    wp2 = CartesianWaypoint(Isometry3d.Identity() * Translation3d(-.2,.4,0.2) * Quaterniond(0,0,1.0,0))

    start_instruction = MoveInstruction(Waypoint(wp1), MoveInstructionType_START, "DEFAULT")
    plan_f1 = MoveInstruction(Waypoint(wp2), MoveInstructionType_FREESPACE, "DEFAULT")

    program = CompositeInstruction("DEFAULT")
    program.setStartInstruction(Instruction(start_instruction))
    program.setManipulatorInfo(manip)
    program.append(Instruction(plan_f1))

    planning_server = ProcessPlanningServer(env, 1)
    planning_server.loadDefaultProcessPlanners()
    request = ProcessPlanningRequest()
    request.name = FREESPACE_PLANNER_NAME
    request.instructions = Instruction(program)

    response = planning_server.run(request)
    planning_server.waitForAll()

    assert response.interface.isSuccessful()

    results = flatten(response.problem.getResults().as_CompositeInstruction())

    # assert len(results) == 37
    for instr in results:
        assert isMoveInstruction(instr)
        move_instr1 = instr.as_MoveInstruction()
        wp1 = move_instr1.getWaypoint()
        assert isStateWaypoint(wp1)
        wp = wp1.as_StateWaypoint()
        assert len(wp.joint_names) == 7
        assert isinstance(wp.position,np.ndarray)
        assert len(wp.position) == 7
