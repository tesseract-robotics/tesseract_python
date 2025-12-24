# Examples

High-level Pythonic API examples for tesseract_robotics.

These examples use `tesseract_robotics.planning` - a clean, Pythonic interface built on top of the low-level bindings. This is the recommended way to use the library.

## Examples

| Example | Description |
|---------|-------------|
| `freespace_ompl_example.py` | OMPL freespace motion planning |
| `basic_cartesian_example.py` | Cartesian path planning with TrajOpt |
| `glass_upright_example.py` | Constrained motion (keep end-effector upright) |
| `puzzle_piece_auxillary_axes_example.py` | 9-DOF planning with external positioner |
| `pick_and_place_example.py` | Pick-and-place with attached collision objects |
| `car_seat_example.py` | Complex multi-phase pick-and-place with meshes |
| `tesseract_collision_example.py` | Collision checking |
| `tesseract_kinematics_example.py` | Forward/inverse kinematics |

## Quick Start

```python
from tesseract_robotics.planning import (
    Robot,
    MotionProgram,
    CartesianTarget,
    JointTarget,
    Pose,
    TaskComposer,
)

# Load robot
robot = Robot.from_urdf("package://my_robot/urdf/robot.urdf",
                        "package://my_robot/urdf/robot.srdf")

# Build motion program
program = (MotionProgram("manipulator")
    .move_to(JointTarget([0, 0, 0, -1.57, 0, 0, 0]))
    .move_to(CartesianTarget(Pose.from_xyz(0.5, 0.0, 0.5)))
)

# Plan
composer = TaskComposer.from_config()
result = composer.plan(robot, program)
if result.successful:
    print(f"Trajectory: {len(result)} waypoints")
```

## Low-Level API

For direct access to the C++ bindings, see [lowlevel/](lowlevel/).
