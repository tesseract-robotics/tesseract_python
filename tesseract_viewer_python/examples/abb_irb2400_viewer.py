from tesseract_robotics.tesseract_common import FilesystemPath, Isometry3d, Translation3d, Quaterniond, \
    ManipulatorInfo
from tesseract_robotics.tesseract_environment import Environment
from tesseract_robotics.tesseract_common import ResourceLocator, SimpleLocatedResource
from tesseract_robotics.tesseract_command_language import CartesianWaypoint, Waypoint, \
    PlanInstructionType_FREESPACE, PlanInstructionType_START, PlanInstruction, Instruction, \
    CompositeInstruction, flatten
from tesseract_robotics.tesseract_process_managers import ProcessPlanningServer, ProcessPlanningRequest, \
    FREESPACE_PLANNER_NAME
import os
import re
import traceback
from tesseract_robotics_viewer import TesseractViewer
import numpy as np
import time
import sys

TESSERACT_SUPPORT_DIR = os.environ["TESSERACT_SUPPORT_DIR"]

class TesseractSupportResourceLocator(ResourceLocator):
    def __init__(self):
        super().__init__()
    
    def locateResource(self, url):
        try:
            try:
                if os.path.exists(url):
                    return SimpleLocatedResource(url, url, self)
            except:
                pass
            url_match = re.match(r"^package:\/\/tesseract_support\/(.*)$",url)
            if (url_match is None):
                print("url_match failed")
                return None
            if not "TESSERACT_SUPPORT_DIR" in os.environ:
                return None
            tesseract_support = os.environ["TESSERACT_SUPPORT_DIR"]
            filename = os.path.join(tesseract_support, os.path.normpath(url_match.group(1)))
            ret = SimpleLocatedResource(url, filename, self)
            return ret
        except:
            traceback.print_exc()

abb_irb2400_urdf_fname = FilesystemPath(os.path.join(TESSERACT_SUPPORT_DIR,"urdf","abb_irb2400.urdf"))
abb_irb2400_srdf_fname = FilesystemPath(os.path.join(TESSERACT_SUPPORT_DIR,"urdf","abb_irb2400.srdf"))

t_env = Environment()

# locator_fn must be kept alive by maintaining a reference
locator = TesseractSupportResourceLocator()
t_env.init(abb_irb2400_urdf_fname, abb_irb2400_srdf_fname, locator)

manip_info = ManipulatorInfo()
manip_info.tcp_frame = "tool0"
manip_info.manipulator = "manipulator"
manip_info.working_frame = "base_link"

viewer = TesseractViewer()

viewer.update_environment(t_env, [0,0,0])

joint_names = ["joint_%d" % (i+1) for i in range(6)]
viewer.update_joint_positions(joint_names, np.array([1,-.2,.01,.3,-.5,1]))

viewer.start_serve_background()

t_env.setState(joint_names, np.ones(6)*0.1)

wp1 = CartesianWaypoint(Isometry3d.Identity() * Translation3d(0.8,-0.3,1.455) * Quaterniond(0.70710678,0,0.70710678,0))
wp2 = CartesianWaypoint(Isometry3d.Identity() * Translation3d(0.8,0.3,1.455) * Quaterniond(0.70710678,0,0.70710678,0))
wp3 = CartesianWaypoint(Isometry3d.Identity() * Translation3d(0.8,0.3,1) * Quaterniond(0.70710678,0,0.70710678,0))

start_instruction = PlanInstruction(Waypoint(wp1), PlanInstructionType_START, "DEFAULT")
plan_f1 = PlanInstruction(Waypoint(wp2), PlanInstructionType_FREESPACE, "DEFAULT")

program = CompositeInstruction("DEFAULT")
program.setStartInstruction(Instruction(start_instruction))
program.setManipulatorInfo(manip_info)
program.append(Instruction(plan_f1))

planning_server = ProcessPlanningServer(t_env, 1)
planning_server.loadDefaultProcessPlanners()
request = ProcessPlanningRequest()
request.name = FREESPACE_PLANNER_NAME
request.instructions = Instruction(program)

response = planning_server.run(request)
planning_server.waitForAll()

assert response.interface.isSuccessful()

results = flatten(response.getResults().as_CompositeInstruction())

viewer.update_trajectory(results)

if sys.version_info[0] < 3:
    input("press enter")
else:
    input("press enter")

