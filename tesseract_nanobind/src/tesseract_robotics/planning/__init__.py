"""
tesseract_robotics.planning - Pythonic API for Tesseract Motion Planning

This module provides a high-level, Pythonic interface to the Tesseract motion
planning libraries. It wraps the lower-level nanobind bindings with convenient
factory functions, automatic type conversions, and builder patterns.

Key Features:
- Automatic poly-type wrapping (no more `_wrap_` functions)
- Simple robot/environment loading from URDF/SRDF
- Builder pattern for motion programs
- Simplified task composer workflow
- Transform/pose helpers

Example Usage:
    from tesseract_robotics.planning import (
        Robot,
        MotionProgram,
        CartesianTarget,
        JointTarget,
        plan_freespace,
    )

    # Load robot from URDF/SRDF
    robot = Robot.from_urdf("package://my_robot/urdf/robot.urdf",
                            "package://my_robot/urdf/robot.srdf")

    # Create motion program with builder pattern
    program = (MotionProgram("manipulator")
        .move_to(JointTarget([0, 0, 0, 0, 0, 0]))
        .move_to(CartesianTarget([0.5, 0.0, 0.5], [0, 0, 1, 0]))
        .move_to(JointTarget([0.5, 0, 0, 0, 0, 0]))
    )

    # Plan and execute
    result = plan_freespace(robot, program)
    if result.successful:
        print(f"Found trajectory with {len(result.trajectory)} waypoints")
"""

from tesseract_robotics.planning.core import (
    Robot,
    RobotState,
)

from tesseract_robotics.planning.program import (
    MotionProgram,
    CartesianTarget,
    JointTarget,
    StateTarget,
    MoveType,
)

from tesseract_robotics.planning.transforms import (
    Pose,
    Transform,  # Backwards compatibility alias
    translation,
    rotation_x,
    rotation_y,
    rotation_z,
    rotation_from_quaternion,
    rotation_from_axis_angle,
)

from tesseract_robotics.planning.geometry import (
    box,
    sphere,
    cylinder,
    cone,
    mesh_from_file,
    convex_mesh_from_file,
    create_obstacle,
    create_fixed_joint,
)

from tesseract_robotics.planning.planner import (
    PlanningResult,
    plan_freespace,
    plan_ompl,
    plan_cartesian,
    PlannerConfig,
    assign_current_state_as_seed,
)

from tesseract_robotics.planning.composer import (
    TaskComposer,
)

__all__ = [
    # Core
    "Robot",
    "RobotState",
    # Program building
    "MotionProgram",
    "CartesianTarget",
    "JointTarget",
    "StateTarget",
    "MoveType",
    # Transforms
    "Pose",
    "Transform",  # Backwards compatibility alias
    "translation",
    "rotation_x",
    "rotation_y",
    "rotation_z",
    "rotation_from_quaternion",
    "rotation_from_axis_angle",
    # Geometry
    "box",
    "sphere",
    "cylinder",
    "cone",
    "mesh_from_file",
    "convex_mesh_from_file",
    "create_obstacle",
    "create_fixed_joint",
    # Planning
    "PlanningResult",
    "plan_freespace",
    "plan_ompl",
    "plan_cartesian",
    "PlannerConfig",
    "assign_current_state_as_seed",
    # Task Composer
    "TaskComposer",
]
