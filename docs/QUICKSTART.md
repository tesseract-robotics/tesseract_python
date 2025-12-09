# Quick Start Guide

## Installation

**Platform:** Linux x86_64 (Python 3.9-3.12). macOS coming soon.

```bash
pip install tesseract-robotics-nanobind
```

## Verify Installation

```python
import tesseract_robotics
from tesseract_robotics.tesseract_environment import Environment

env = Environment()
print("Success!")
```

## Example

```python
from tesseract_robotics.planning import (
    Robot, MotionProgram, JointTarget, CartesianTarget,
    Pose, box, create_obstacle, TaskComposer,
)

# Load robot from bundled tesseract_support
robot = Robot.from_tesseract_support("abb_irb2400")

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

## Next Steps

- Check `examples/` for more usage patterns
- See [MIGRATION_NOTES.md](MIGRATION_NOTES.md) for API differences from SWIG bindings
