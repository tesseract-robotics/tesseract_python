# Tesseract Python

[![Python](https://img.shields.io/badge/python-3.8+-blue.svg)](https://github.com/tesseract-robotics/tesseract_python)

![PyPI](https://img.shields.io/pypi/v/tesseract-robotics)

Platform             | CI Status
---------------------|:---------
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
for Python 3.8+.

The Tesseract Python package is developed and maintained by Wason Technology, LLC.

**Note that these are low level wrappers.** The lifecycle of objects follow the underlying C++ objects, meaning
that the target of C++ references may be destroyed before the reference, leading to a memory error. These wrappers
do not attempt to change the memory lifecycle of the underlying C++ objects.

## Documentation

See https://tesseract-robotics.github.io/tesseract_python/ for documentation.

## Installation

Standalone packages are provided on PyPi (pip install) for Windows and Linux, containing Tesseract, Tesseract
Planning, andall the native dependencies 
for Python 3.8+. These packages have been tested on Windows 10, and Ubuntu 22.04, but should work
on any relatively recent x64 Windows or Linux operating system. Packages are available for Python 3.8 - 3.13

To install on Windows:
```
python -m pip install tesseract-robotics tesseract-robotics-viewer
```
To install on Ubuntu 24.04 and Ubuntu 22.04:

```
sudo apt install python3-pip python3-numpy
python3 -m pip install -U pip
python3 -m pip install --user  tesseract_robotics tesseract_robotics_viewer
```

## Example

ABB Tesseract viewer plan and viewer example:

Install `tesseract_robotics` and `tesseract_robotics_viewer` as shown in Installation section.

Clone `tesseract`, `tesseract_planning`, and `tesseract_python` repositories to retrieve example assets. This is not 
necessary if the example assets are not used.

```
git clone --depth=1 https://github.com/tesseract-robotics/tesseract.git
git clone --depth=1 https://github.com/tesseract-robotics/tesseract_planning.git
git clone --depth=1 https://github.com/tesseract-robotics/tesseract_python.git
```

Set the `TESSERACT_RESOURCE_PATH` and `TESSERACT_TASK_COMPOSER_CONFIG_FILE` environmental variables so the example 
can find required resources:

Linux:

```
export TESSERACT_RESOURCE_PATH=`pwd`/tesseract
export TESSERACT_TASK_COMPOSER_CONFIG_FILE=`pwd`/tesseract_planning/tesseract_task_composer/config/task_composer_plugins_no_trajopt_ifopt.yaml
```

Windows:

```
set TESSERACT_RESOURCE_PATH=%CD%/tesseract
set TESSERACT_TASK_COMPOSER_CONFIG_FILE=%CD%/tesseract_planning/tesseract_task_composer/config/task_composer_plugins_no_trajopt_ifopt.yaml
```

Now run the example!

Windows:

```
cd tesseract_python\examples
python tesseract_planning_example_composer.py
```

Linux:

```
cd tesseract_python/examples
python3 tesseract_planning_example_composer.py
```

And point a modern browser to `http://localhost:8000` to see the animation!

Example source:

```python
import re
import traceback
import os
import numpy as np
import numpy.testing as nptest

from tesseract_robotics.tesseract_common import GeneralResourceLocator
from tesseract_robotics.tesseract_environment import Environment, AnyPoly_wrap_EnvironmentConst
from tesseract_robotics.tesseract_common import FilesystemPath, Isometry3d, Translation3d, Quaterniond, \
    ManipulatorInfo, AnyPoly, AnyPoly_wrap_double
from tesseract_robotics.tesseract_command_language import CartesianWaypoint, WaypointPoly, \
    MoveInstructionType_FREESPACE, MoveInstruction, InstructionPoly, StateWaypoint, StateWaypointPoly, \
    CompositeInstruction, MoveInstructionPoly, CartesianWaypointPoly, ProfileDictionary, \
        AnyPoly_as_CompositeInstruction, CompositeInstructionOrder_ORDERED, DEFAULT_PROFILE_KEY, \
        AnyPoly_wrap_CompositeInstruction, DEFAULT_PROFILE_KEY, JointWaypoint, JointWaypointPoly, \
        InstructionPoly_as_MoveInstructionPoly, WaypointPoly_as_StateWaypointPoly, \
        MoveInstructionPoly_wrap_MoveInstruction, StateWaypointPoly_wrap_StateWaypoint, \
        CartesianWaypointPoly_wrap_CartesianWaypoint, JointWaypointPoly_wrap_JointWaypoint, \
        AnyPoly_wrap_ProfileDictionary

from tesseract_robotics.tesseract_task_composer import TaskComposerPluginFactory, \
    TaskComposerDataStorage, TaskComposerContext

from tesseract_robotics_viewer import TesseractViewer

# Run example FreespacePipeline planner

OMPL_DEFAULT_NAMESPACE = "OMPLMotionPlannerTask"
TRAJOPT_DEFAULT_NAMESPACE = "TrajOptMotionPlannerTask"

task_composer_filename = os.environ["TESSERACT_TASK_COMPOSER_CONFIG_FILE"]

# Initialize the resource locator and environment
locator = GeneralResourceLocator()
abb_irb2400_urdf_package_url = "package://tesseract_support/urdf/abb_irb2400.urdf"
abb_irb2400_srdf_package_url = "package://tesseract_support/urdf/abb_irb2400.srdf"
abb_irb2400_urdf_fname = FilesystemPath(locator.locateResource(abb_irb2400_urdf_package_url).getFilePath())
abb_irb2400_srdf_fname = FilesystemPath(locator.locateResource(abb_irb2400_srdf_package_url).getFilePath())

t_env = Environment()

# locator_fn must be kept alive by maintaining a reference
assert t_env.init(abb_irb2400_urdf_fname, abb_irb2400_srdf_fname, locator)

# Fill in the manipulator information. This is used to find the kinematic chain for the manipulator. This must
# match the SRDF, although the exact tcp_frame can differ if a tool is used.
manip_info = ManipulatorInfo()
manip_info.tcp_frame = "tool0"
manip_info.manipulator = "manipulator"
manip_info.working_frame = "base_link"

# Create a viewer and set the environment so the results can be displayed later
viewer = TesseractViewer()
viewer.update_environment(t_env, [0,0,0])

# Set the initial state of the robot
joint_names = ["joint_%d" % (i+1) for i in range(6)]
viewer.update_joint_positions(joint_names, np.array([1,-.2,.01,.3,-.5,1]))

# Start the viewer
viewer.start_serve_background()

# Set the initial state of the robot
t_env.setState(joint_names, np.ones(6)*0.1)

# Create the input command program waypoints
wp1 = CartesianWaypoint(Isometry3d.Identity() * Translation3d(0.8,-0.3,1.455) * Quaterniond(0.70710678,0,0.70710678,0))
wp2 = CartesianWaypoint(Isometry3d.Identity() * Translation3d(0.8,0.3,1.455) * Quaterniond(0.70710678,0,0.70710678,0))
wp3 = CartesianWaypoint(Isometry3d.Identity() * Translation3d(0.8,0.5,1.455) * Quaterniond(0.70710678,0,0.70710678,0))

# Create the input command program instructions. Note the use of explicit construction of the CartesianWaypointPoly
# using the *_wrap_CartesianWaypoint functions. This is required because the Python bindings do not support implicit
# conversion from the CartesianWaypoint to the CartesianWaypointPoly.
start_instruction = MoveInstruction(CartesianWaypointPoly_wrap_CartesianWaypoint(wp1), MoveInstructionType_FREESPACE, "DEFAULT")
plan_f1 = MoveInstruction(CartesianWaypointPoly_wrap_CartesianWaypoint(wp2), MoveInstructionType_FREESPACE, "DEFAULT")
plan_f2 = MoveInstruction(CartesianWaypointPoly_wrap_CartesianWaypoint(wp3), MoveInstructionType_FREESPACE, "DEFAULT")

# Create the input command program. Note the use of *_wrap_MoveInstruction functions. This is required because the
# Python bindings do not support implicit conversion from the MoveInstruction to the MoveInstructionPoly.
program = CompositeInstruction("DEFAULT")
program.setManipulatorInfo(manip_info)
program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(start_instruction))
program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(plan_f1))
# program.appendMoveInstruction(MoveInstructionPoly(plan_f2))

# Create the task composer plugin factory and load the plugins
config_path = FilesystemPath(task_composer_filename)
factory = TaskComposerPluginFactory(config_path, locator)

# Create the task composer node. In this case the FreespacePipeline is used. Many other are available.
task = factory.createTaskComposerNode("FreespacePipeline")

# Get the output keys for the task
output_key = task.getOutputKeys().get("program")
input_key = task.getInputKeys().get("planning_input")

# Create a profile dictionary. Profiles can be customized by adding to this dictionary and setting the profiles
# in the instructions.
profiles = ProfileDictionary()

# Create an AnyPoly containing the program. This explicit step is required because the Python bindings do not
# support implicit conversion from the CompositeInstruction to the AnyPoly.
program_anypoly = AnyPoly_wrap_CompositeInstruction(program)
environment_anypoly = AnyPoly_wrap_EnvironmentConst(t_env)
profiles_anypoly = AnyPoly_wrap_ProfileDictionary(profiles)

# Create the task data
task_data = TaskComposerDataStorage()
task_data.setData(input_key, program_anypoly)
task_data.setData("environment", environment_anypoly)
task_data.setData("profiles", profiles_anypoly)

# Create an executor to run the task
task_executor = factory.createTaskComposerExecutor("TaskflowExecutor")

# Run the task and wait for completion
future = task_executor.run(task.get(), task_data)
future.wait()

if not future.context.isSuccessful():
    print("Planning task failed")
    exit(1)

# Retrieve the output, converting the AnyPoly back to a CompositeInstruction
results = AnyPoly_as_CompositeInstruction(future.context.data_storage.getData(output_key))

# Display the output
# Print out the resulting waypoints
for instr in results:
    assert instr.isMoveInstruction()
    move_instr1 = InstructionPoly_as_MoveInstructionPoly(instr)
    wp1 = move_instr1.getWaypoint()
    assert wp1.isStateWaypoint()
    wp = WaypointPoly_as_StateWaypointPoly(wp1)
    print(f"Joint Positions: {wp.getPosition().flatten()} time: {wp.getTime()}")

# Update the viewer with the results to animate the trajectory
# Open web browser to http://localhost:8000 to view the results
viewer.update_trajectory(results)
viewer.plot_trajectory(results, manip_info)

input("press enter to exit")
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
* **tesseract_task_composer** – This package contains a common interface for task pipelines and includes implementation for a wide variaty of process found industrial automation like painting, griding, welding, pick and place and more.
* **tesseract_time_parameterization** – This package contains a time parameterization algorithms and includes iterative spline.

## Related Repositories

* [Tesseract](https://github.com/tesseract-robotics/tesseract)
* [Tesseract Planning](https://github.com/tesseract-robotics/tesseract_planning)
* [Tesseract ROS](https://github.com/tesseract-robotics/tesseract_ros)
* [General Robotics Toolbox](https://github.com/rpiRobotics/rpi_general_robotics_toolbox_py)
* [PyRI Open Source Teach Pendant](https://github.com/pyri-project)

## Documentation

* [Wiki](https://ros-industrial-tesseract-python.rtfd.io)
* [Doxygen](https://tesseract-robotics.github.io/tesseract_python/)
* [Benchmark](https://tesseract-robotics.github.io/tesseract_python/dev/bench)

## Build Instructions

Building the tesseract_python package is complicated and not recommended for novice users. See the wheels.yml
workflow for details on how to build the packages and all dependencies.
