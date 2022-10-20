# Tesseract Python

[![Python](https://img.shields.io/badge/python-3.7+-blue.svg)](https://github.com/tesseract-robotics/tesseract_python)

![PyPI](https://img.shields.io/pypi/v/tesseract-robotics)

Platform             | CI Status
---------------------|:---------
Linux (Focal)        | [![Build Status](https://github.com/tesseract-robotics/tesseract_python/workflows/Focal-Build/badge.svg)](https://github.com/tesseract-robotics/tesseract_python/actions)
Windows              | [![Build Status](https://github.com/tesseract-robotics/tesseract_python/workflows/Windows-Noetic-Build/badge.svg)](https://github.com/tesseract-robotics/tesseract_python/actions)
Wheels               | [![Build Status](https://github.com/tesseract-robotics/tesseract_python/actions/workflows/wheels.yml/badge.svg)](https://github.com/tesseract-robotics/tesseract_python/actions)

[![Github Issues](https://img.shields.io/github/issues/tesseract-robotics/tesseract_python.svg)](http://github.com/tesseract-robotics/tesseract_python/issues)

[![license - apache 2.0](https://img.shields.io/:license-Apache%202.0-yellowgreen.svg)](https://opensource.org/licenses/Apache-2.0)
[![license - bsd 2 clause](https://img.shields.io/:license-BSD%202--Clause-blue.svg)](https://opensource.org/licenses/BSD-2-Clause)

[![support level: consortium](https://img.shields.io/badge/support%20level-consortium-brightgreen.png)](http://rosindustrial.org/news/2016/10/7/better-supporting-a-growing-ros-industrial-software-platform)

`tesseract_python` contains Python wrappers for the Tesseract robot motion planner, generated using SWIG. These wrappers
contain most of the Tesseract functionality, including scene loading and management (URDF,SRDF, meshes), collision
checking (Bullet, FCL), kinematics (KDL, OPW, UR), planning (OMPL, Descartes, TrajOpt), and visualization 
(tesseract_viewer_python)

Standalone packages are provided on PyPi (pip install) for Windows and Linux, containing all the native dependencies 
for Python 3.7+.

**Note that these are low level wrappers.** The lifecycle of objects follow the underlying C++ objects, meaning
that the target of C++ references may be destroyed before the reference, leading to a memory error. These wrappers
do not attempt to change the memory lifecycle of the underlying C++ objects.

## Installation

Standalone packages are provided on PyPi (pip install) for Windows and Linux, containing Tesseract, Tesseract
Planning, andall the native dependencies 
for Python 3.7+. These packages have been tested on Windows 10, Ubuntu 20.04, and Ubuntu 22.04, but should work
on any relatively recent x64 Windows or Linux operating system. Packages are available for Python 3.7 - 3.10.

To install on Windows:
```
python -m pip install tesseract-robotics tesseract-robotics-viewer
```
To install on Ubuntu 20.04 and Ubuntu 22.04:

```
sudo apt install python3-pip python3-numpy
# The supplied version of pip on Ubuntu 20.04 is too old for manylinux_2_31, upgrade pip
python3 -m pip install -U pip
python3 -m pip install --user  tesseract_robotics tesseract_robotics_viewer
```

## Example

ABB Tesseract viewer plan and viewer example:

Install `tesseract_robotics` and `tesseract_robotics_viewer` as shown in Installation section.

Clone `tesseract` and `tesseract_python` repositories to retrieve example assets. This is not necessary
if the example assets are not used.

```
git clone --depth=1 https://github.com/tesseract-robotics/tesseract.git
git clone --depth=1 https://github.com/tesseract-robotics/tesseract_python.git
```

Set the `TESSERACT_SUPPORT_DIR` so the example can find the URDF resources:

Linux:

```
export TESSERACT_SUPPORT_DIR=`pwd`/tesseract/tesseract_support
```

Windows:

```
set TESSERACT_SUPPORT_DIR=%CD%/tesseract/tesseract_support
```

Now run the example!

Windows:

```
cd tesseract_python\tesseract_viewer_python\examples
python abb_irb2400_viewer.py
```

Linux:

```
cd tesseract_python/tesseract_viewer_python/examples
python3 abb_irb2400_viewer.py
```

And point a modern browser to `http://localhost:8000` to see the animation!

Example source:

```python
from tesseract_robotics.tesseract_common import FilesystemPath, Isometry3d, Translation3d, Quaterniond, \
    ManipulatorInfo
from tesseract_robotics.tesseract_environment import Environment
from tesseract_robotics.tesseract_common import ResourceLocator, SimpleLocatedResource
from tesseract_robotics.tesseract_command_language import CartesianWaypoint, Waypoint, \
    MoveInstructionType_FREESPACE, MoveInstructionType_START, MoveInstruction, Instruction, \
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

start_instruction = MoveInstruction(Waypoint(wp1), MoveInstructionType_START, "DEFAULT")
plan_f1 = MoveInstruction(Waypoint(wp2), MoveInstructionType_FREESPACE, "DEFAULT")

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

results = flatten(response.problem.getResults().as_CompositeInstruction())

viewer.update_trajectory(joint_names, results)

if sys.version_info[0] < 3:
    input("press enter")
else:
    input("press enter")

```

## Tesseract Python Supported Packages

* **tesseract_collision** – This package contains privides a common interface for collision checking prividing several implementation of a Bullet collision library and FCL collision library. It includes both continuous and discrete collision checking for convex-convex, convex-concave and concave-concave shapes.
* **tesseract_common** – This package contains common functionality needed by the majority of the packages.
* **tesseract_environment** – This package contains the Tesseract Environment which provides functionality to add,remove,move and modify links and joint. It also manages adding object to the contact managers and provides the ability.
* **tesseract_geometry** – This package contains geometry types used by Tesseract including primitive shapes, mesh, convex hull mesh, octomap and signed distance field.
* **tesseract_kinematics** –  This package contains a common interface for Forward and Inverse kinematics for Chain, Tree's and Graphs including implementation using KDL and OPW Kinematics.
* **tesseract_scene_graph** – This package contains the scene graph which is the data structure used to manage the connectivity of objects in the environment. It inherits from boost graph and provides addition functionality for adding,removing and modifying Links and Joints along with search implementation.
* **tesseract_support** – This package contains support data used for unit tests and examples throughout Tesseract.
* **tesseract_urdf** - This package contains a custom urdf parser supporting addition shapes and features currently not supported by urdfdom.
* **tesseract_visualization** – This package contains visualization utilities and libraries.
* **tesseract_command_language** - This is a generic programing language used as input for motion and process planning. It is very similar to how you currently program a robot on an industrial teach pendant.
* **tesseract_motion_planners** – This package contains a common interface for Planners and includes implementation for OMPL, TrajOpt, TrajOpt IFOPT and Descartes.
* **tesseract_process_managers** – This package contains a common interface for Process Planning and includes implementation for a wide variaty of process found industrial automation like painting, griding, welding, pick and place and more.
* **tesseract_time_parameterization** – This package contains a time parameterization algorithms and includes iteritive spline.

## Related Repositories

* [Tesseract](https://github.com/tesseract-robotics/tesseract)
* [Tesseract Planning](https://github.com/tesseract-robotics/tesseract_planning)
* [Tesseract ROS](https://github.com/tesseract-robotics/tesseract_ros)

## Documentation

* [Wiki](https://ros-industrial-tesseract-python.rtfd.io)
* [Doxygen](https://tesseract-robotics.github.io/tesseract_python/)
* [Benchmark](https://tesseract-robotics.github.io/tesseract_python/dev/bench)

## TODO's

Warning: These packages are under heavy development and are subject to change.

See [issue #66](https://github.com/tesseract-robotics/tesseract/issues/66)

## Build Instructions

Building the tesseract_python package is complicated and not recommended for novice users. See the wheels.yml
workflow for details on how to build the packages and all dependencies.
