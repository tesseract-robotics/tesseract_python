# Tesseract Python (nanobind)

Python bindings for Tesseract robotics using nanobind and scikit-build-core.

## Install

```bash
pip install tesseract-robotics
```

## From Source

```bash
# 1. Clone and setup
git clone https://github.com/tesseract-robotics/tesseract_python.git
cd tesseract_python

# 2. Build C++ dependencies
source env.sh
cd ws && colcon build --merge-install

# 3. Install Python package
cd ../tesseract_python_nb
pip install -e .

# 4. Test
pytest tests/
```

## Modules

- tesseract_common - Eigen types, transforms, resource locators
- tesseract_geometry - Box, Sphere, Cylinder, Mesh, ConvexMesh
- tesseract_scene_graph - Link, Joint, SceneGraph
- tesseract_collision - BulletDiscreteBVH, ContactResultMap
- tesseract_kinematics - JointGroup, KinematicGroup, IK/FK
- tesseract_state_solver - StateSolver
- tesseract_environment - Environment, Commands
- tesseract_srdf - SRDFModel, KinematicsInformation
- tesseract_command_language - Waypoints, Instructions
- tesseract_motion_planners - OMPL planner, profiles
- tesseract_motion_planners_trajopt - TrajOpt planner (optional)
- tesseract_time_parameterization - TOTG, ISP
- tesseract_task_composer - TaskComposerPluginFactory

## Example

```python
from tesseract_robotics.tesseract_environment import Environment
from tesseract_robotics.tesseract_common import GeneralResourceLocator

env = Environment()
locator = GeneralResourceLocator()
env.init("/path/to/robot.urdf", "/path/to/robot.srdf", locator)

print(f"Joints: {env.getJointNames()}")
print(f"Links: {env.getLinkNames()}")
```

## API Differences from SWIG

nanobind requires explicit wrapping for type-erased containers:

```python
# SWIG (implicit)
program.appendMoveInstruction(move_instr)

# nanobind (explicit wrap)
from tesseract_robotics.tesseract_command_language import MoveInstructionPoly_wrap_MoveInstruction
program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(move_instr))
```

See [MIGRATION_NOTES.md](MIGRATION_NOTES.md) for details.

## Requirements

- Python 3.12+
- Tesseract C++ libraries
- CMake 3.15+

## Architecture

- nanobind for Python bindings (replacing SWIG)
- Stable ABI (abi3) - single wheel works across Python versions
- scikit-build-core for modern Python packaging
- Native Eigen/numpy integration
