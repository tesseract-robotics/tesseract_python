# Tesseract Python (nanobind)

[![PyPI](https://img.shields.io/pypi/v/tesseract-robotics-nanobind.svg)](https://pypi.org/project/tesseract-robotics-nanobind/)
[![Python](https://img.shields.io/badge/python-3.9%20|%203.10%20|%203.11%20|%203.12-blue.svg)](https://github.com/tesseract-robotics/tesseract_nanobind)
[![Build Status](https://github.com/tesseract-robotics/tesseract_nanobind/actions/workflows/wheels.yml/badge.svg)](https://github.com/tesseract-robotics/tesseract_nanobind/actions)
[![Documentation](https://img.shields.io/badge/docs-online-brightgreen.svg)](https://tesseract-robotics.github.io/tesseract_nanobind/)
[![license - Apache 2.0](https://img.shields.io/:license-Apache%202.0-yellowgreen.svg)](https://opensource.org/licenses/Apache-2.0)

> **Note:** This is a friendly fork of [tesseract_python](https://github.com/tesseract-robotics/tesseract_python) that replaces SWIG bindings with modern [nanobind](https://github.com/wjakob/nanobind) bindings.

Python bindings for [Tesseract](https://github.com/tesseract-robotics/tesseract) robotics motion planning using [nanobind](https://github.com/wjakob/nanobind).

## Features

- Scene loading and management (URDF, SRDF, meshes)
- Collision checking (Bullet, FCL)
- Kinematics (KDL, OPW, UR)
- Motion planning (OMPL, Descartes, TrajOpt)
- Time parameterization (TOTG, ISP, Ruckig)
- Task composition and pipelines
- Pythonic high-level API

## Installation

```bash
pip install tesseract-robotics-nanobind
```

**Platform support:** Linux x86_64. macOS arm64 coming soon.

## Quick Start

```python
from tesseract_robotics.planning import (
    Robot, MotionProgram, JointTarget, CartesianTarget,
    Pose, box, create_obstacle, TaskComposer,
)

# Load robot
robot = Robot.from_urdf(
    "package://tesseract_support/urdf/abb_irb2400.urdf",
    "package://tesseract_support/urdf/abb_irb2400.srdf"
)

# Add obstacle
create_obstacle(robot, "box", box(0.5, 0.5, 0.5), Pose.from_xyz(0.5, 0, 0.3))

# Build motion program
program = (MotionProgram("manipulator", tcp_frame="tool0")
    .set_joint_names(robot.get_joint_names("manipulator"))
    .move_to(JointTarget([0, 0, 0, 0, 0, 0]))
    .move_to(CartesianTarget(Pose.from_xyz(0.5, 0.3, 0.8)))
)

# Plan
composer = TaskComposer.from_config()
result = composer.plan(robot, program)

if result.successful:
    for pt in result.trajectory:
        print(pt.positions)
```

## Low-Level API

For direct C++ API access:

```python
from tesseract_robotics.tesseract_environment import Environment
from tesseract_robotics.tesseract_common import GeneralResourceLocator

env = Environment()
locator = GeneralResourceLocator()
env.init("/path/to/robot.urdf", "/path/to/robot.srdf", locator)

print(f"Joints: {env.getJointNames()}")
print(f"Links: {env.getLinkNames()}")
```

## Examples

See the `examples/` directory for:
- `basic_cartesian_example.py` - Simple Cartesian planning
- `freespace_ompl_example.py` - OMPL freespace planning
- `pick_and_place_example.py` - Pick and place with TrajOpt
- `puzzle_piece_example.py` - Cartesian path following
- And more...

## Development

Enable pre-commit hook (runs test suite before each commit):

```bash
git config core.hooksPath .githooks
```

## Acknowledgments

This project builds upon the excellent work of [John Wason](https://github.com/johnwason) and the [Tesseract Robotics](https://github.com/tesseract-robotics) team. The original [tesseract_python](https://github.com/tesseract-robotics/tesseract_python) SWIG bindings laid the foundation for this nanobind implementation.

Special thanks to:
- **John Wason** (Wason Technology, LLC) - Original tesseract_python author and Tesseract maintainer
- **Levi Armstrong** - Tesseract core developer
- **Jelle Feringa** ([Terrestrial](http://terrestrial.construction)) - nanobind port developer
- The ROS-Industrial consortium for supporting Tesseract development

## License

Apache 2.0
