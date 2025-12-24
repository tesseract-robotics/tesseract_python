"""
Simplified planning functions.

Provides top-level functions for common planning operations that handle
all the setup automatically.

Example:
    from tesseract_robotics.planning import Robot, MotionProgram, plan_freespace

    robot = Robot.from_tesseract_support("abb_irb2400")

    program = (MotionProgram("manipulator")
        .move_to(JointTarget([0, 0, 0, 0, 0, 0]))
        .move_to(CartesianTarget(Transform.from_xyz(0.5, 0, 0.5)))
    )

    result = plan_freespace(robot, program)
    if result.successful:
        print(f"Found {len(result)} waypoints")
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Optional, Union

import numpy as np

from tesseract_robotics.tesseract_command_language import (
    CompositeInstruction,
    ProfileDictionary,
)
from tesseract_robotics.tesseract_motion_planners import (
    assignCurrentStateAsSeed as _assignCurrentStateAsSeed,
)

# Re-export PlanningResult from composer
from tesseract_robotics.planning.composer import PlanningResult, TaskComposer


@dataclass
class PlannerConfig:
    """
    Configuration options for planning.

    Attributes:
        pipeline: Planning pipeline name
        max_velocity_scaling: Scale factor for velocity limits (0-1)
        max_acceleration_scaling: Scale factor for acceleration limits (0-1)
        collision_safety_margin: Minimum distance from obstacles
        smoothing: Enable trajectory smoothing
        time_parameterization: Enable time parameterization
    """
    pipeline: str = "TrajOptPipeline"
    max_velocity_scaling: float = 1.0
    max_acceleration_scaling: float = 1.0
    collision_safety_margin: float = 0.025
    smoothing: bool = True
    time_parameterization: bool = True


def plan_freespace(
    robot: "Robot",
    program: Union["MotionProgram", CompositeInstruction],
    config: Optional[PlannerConfig] = None,
    profiles: Optional[ProfileDictionary] = None,
) -> PlanningResult:
    """
    Plan freespace motion.

    Uses TrajOpt for collision-free point-to-point motion.

    Args:
        robot: Robot instance
        program: Motion program or CompositeInstruction
        config: Planner configuration
        profiles: Custom motion profiles

    Returns:
        PlanningResult with trajectory if successful

    Example:
        from tesseract_robotics.planning import (
            Robot, MotionProgram, JointTarget, plan_freespace
        )

        robot = Robot.from_tesseract_support("abb_irb2400")
        program = (MotionProgram("manipulator")
            .move_to(JointTarget([0, 0, 0, 0, 0, 0]))
            .move_to(JointTarget([0.5, 0, 0, 0, 0, 0]))
        )

        result = plan_freespace(robot, program)
        if result:
            for point in result:
                print(point.positions)
    """
    config = config or PlannerConfig()

    composer = TaskComposer.from_config()
    return composer.plan(
        robot,
        program,
        pipeline=config.pipeline,
        profiles=profiles,
    )


def plan_ompl(
    robot: "Robot",
    program: Union["MotionProgram", CompositeInstruction],
    config: Optional[PlannerConfig] = None,
    profiles: Optional[ProfileDictionary] = None,
) -> PlanningResult:
    """
    Plan freespace motion using OMPL sampling-based planners.

    Uses OMPL (RRT, RRT*, PRM, etc.) for collision-free motion planning.
    Good for complex environments where optimization-based planners may
    get stuck in local minima.

    Args:
        robot: Robot instance
        program: Motion program or CompositeInstruction
        config: Planner configuration
        profiles: Custom motion profiles

    Returns:
        PlanningResult with trajectory if successful

    Example:
        from tesseract_robotics.planning import (
            Robot, MotionProgram, JointTarget, plan_ompl
        )

        robot = Robot.from_tesseract_support("abb_irb2400")
        program = (MotionProgram("manipulator")
            .move_to(JointTarget([0, 0, 0, 0, 0, 0]))
            .move_to(JointTarget([0.5, 0.5, 0.5, 0, 0, 0]))
        )

        result = plan_ompl(robot, program)
        if result:
            for point in result:
                print(point.positions)
    """
    config = config or PlannerConfig(pipeline="OMPLPipeline")

    composer = TaskComposer.from_config()
    return composer.plan(
        robot,
        program,
        pipeline=config.pipeline,
        profiles=profiles,
    )


def plan_cartesian(
    robot: "Robot",
    program: Union["MotionProgram", CompositeInstruction],
    config: Optional[PlannerConfig] = None,
    profiles: Optional[ProfileDictionary] = None,
) -> PlanningResult:
    """
    Plan Cartesian motion.

    Uses Descartes for precise Cartesian path following.

    Args:
        robot: Robot instance
        program: Motion program with linear/Cartesian targets
        config: Planner configuration
        profiles: Custom motion profiles

    Returns:
        PlanningResult with trajectory if successful

    Example:
        from tesseract_robotics.planning import (
            Robot, MotionProgram, CartesianTarget, Transform, plan_cartesian
        )

        robot = Robot.from_tesseract_support("abb_irb2400")

        # Create linear path
        program = (MotionProgram("manipulator")
            .move_to(CartesianTarget(Transform.from_xyz(0.5, -0.2, 0.5)))
            .linear_to(CartesianTarget(Transform.from_xyz(0.5, 0.2, 0.5)))
        )

        result = plan_cartesian(robot, program)
    """
    config = config or PlannerConfig(pipeline="DescartesPipeline")

    composer = TaskComposer.from_config()
    return composer.plan(
        robot,
        program,
        pipeline=config.pipeline,
        profiles=profiles,
    )


def assign_current_state_as_seed(
    program: CompositeInstruction,
    robot: "Robot",
) -> None:
    """
    Assign current robot state as seed for Cartesian waypoints.

    This is required for planning with Cartesian waypoints to provide
    an initial joint configuration hint.

    Args:
        program: CompositeInstruction to seed
        robot: Robot instance with current state
    """
    _assignCurrentStateAsSeed(program, robot.env)
