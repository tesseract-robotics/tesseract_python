"""
Task Composer helpers for high-level planning pipelines.

Wraps the TaskComposer system with a simpler interface that handles
all the AnyPoly wrapping and boilerplate.

Example:
    from tesseract_robotics.planning import Robot, MotionProgram, TaskComposer

    robot = Robot.from_tesseract_support("abb_irb2400")
    program = MotionProgram("manipulator").move_to(...)

    composer = TaskComposer.from_config()
    result = composer.plan(robot, program, pipeline="TrajOptPipeline")

    if result.successful:
        for waypoint in result.trajectory:
            print(waypoint)
"""
from __future__ import annotations

import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterator, List, Optional, Sequence, Union

import numpy as np

from tesseract_robotics.tesseract_common import (
    GeneralResourceLocator,
    FilesystemPath,
)
from tesseract_robotics.tesseract_command_language import (
    CompositeInstruction,
    ProfileDictionary,
    InstructionPoly_as_MoveInstructionPoly,
    WaypointPoly_as_StateWaypointPoly,
)
from tesseract_robotics.tesseract_task_composer import (
    TaskComposerPluginFactory,
    TaskComposerDataStorage,
    AnyPoly_wrap_CompositeInstruction,
    AnyPoly_wrap_ProfileDictionary,
    AnyPoly_wrap_EnvironmentConst,
    AnyPoly_as_CompositeInstruction,
)


@dataclass
class TrajectoryPoint:
    """
    Single point in a planned trajectory.

    Attributes:
        joint_names: Names of joints
        positions: Joint positions (radians)
        velocities: Joint velocities (optional)
        accelerations: Joint accelerations (optional)
        time: Time from start (optional)
    """
    joint_names: List[str]
    positions: np.ndarray
    velocities: Optional[np.ndarray] = None
    accelerations: Optional[np.ndarray] = None
    time: Optional[float] = None

    def as_dict(self) -> Dict[str, float]:
        """Return positions as {name: position} dictionary."""
        return dict(zip(self.joint_names, self.positions))

    def __repr__(self) -> str:
        pos_str = ", ".join(f"{p:.4f}" for p in self.positions)
        time_str = f", time={self.time:.3f}" if self.time is not None else ""
        return f"TrajectoryPoint([{pos_str}]{time_str})"


@dataclass
class PlanningResult:
    """
    Result from a planning operation.

    Attributes:
        successful: Whether planning succeeded
        message: Status or error message
        trajectory: List of trajectory points (if successful)
        raw_results: Raw CompositeInstruction output
    """
    successful: bool
    message: str = ""
    trajectory: List[TrajectoryPoint] = field(default_factory=list)
    raw_results: Optional[CompositeInstruction] = None

    def __bool__(self) -> bool:
        """Allow `if result:` checks."""
        return self.successful

    def __len__(self) -> int:
        """Return number of waypoints in trajectory."""
        return len(self.trajectory)

    def __iter__(self) -> Iterator[TrajectoryPoint]:
        """Iterate over trajectory points."""
        return iter(self.trajectory)

    def __getitem__(self, idx: int) -> TrajectoryPoint:
        """Get trajectory point by index."""
        return self.trajectory[idx]

    def to_numpy(self) -> np.ndarray:
        """
        Convert trajectory to numpy array.

        Returns:
            Array of shape (N, num_joints) with joint positions
        """
        if not self.trajectory:
            return np.array([])
        return np.array([pt.positions for pt in self.trajectory])


class TaskComposer:
    """
    High-level interface for task composer planning pipelines.

    Wraps the TaskComposerPluginFactory and handles all the AnyPoly
    boilerplate for setting up and running planning tasks.

    Example:
        composer = TaskComposer.from_config()

        # Plan with TrajOpt
        result = composer.plan(robot, program, pipeline="TrajOptPipeline")

        # Plan with OMPL
        result = composer.plan(robot, program, pipeline="OMPLPipeline")

        # Plan freespace motion (auto-selects pipeline)
        result = composer.plan_freespace(robot, program)
    """

    def __init__(
        self,
        factory: TaskComposerPluginFactory,
        locator: Optional[GeneralResourceLocator] = None,
    ):
        """
        Initialize TaskComposer.

        Args:
            factory: Initialized TaskComposerPluginFactory
            locator: Resource locator
        """
        self.factory = factory
        self.locator = locator or GeneralResourceLocator()
        self._executor = None

    @classmethod
    def from_config(
        cls,
        config_path: Optional[Union[str, Path]] = None,
        locator: Optional[GeneralResourceLocator] = None,
    ) -> TaskComposer:
        """
        Create TaskComposer from config file.

        If no config_path is provided, uses TESSERACT_TASK_COMPOSER_CONFIG_FILE
        environment variable or falls back to the default config.

        Args:
            config_path: Path to task composer config YAML
            locator: Resource locator

        Returns:
            Initialized TaskComposer
        """
        locator = locator or GeneralResourceLocator()

        if config_path is None:
            # Try environment variable first
            config_path = os.environ.get("TESSERACT_TASK_COMPOSER_CONFIG_FILE")

            if config_path is None:
                # Fall back to TESSERACT_TASK_COMPOSER_DIR
                composer_dir = os.environ.get("TESSERACT_TASK_COMPOSER_DIR")
                if composer_dir:
                    config_path = os.path.join(
                        composer_dir, "config/task_composer_plugins_no_trajopt_ifopt.yaml"
                    )

        if config_path is None:
            raise ValueError(
                "No config path provided and TESSERACT_TASK_COMPOSER_CONFIG_FILE "
                "or TESSERACT_TASK_COMPOSER_DIR not set"
            )

        factory = TaskComposerPluginFactory(
            FilesystemPath(str(config_path)),
            locator,
        )

        return cls(factory, locator)

    @property
    def executor(self):
        """Get or create task executor (lazy initialization)."""
        if self._executor is None:
            self._executor = self.factory.createTaskComposerExecutor("TaskflowExecutor")
        return self._executor

    def plan(
        self,
        robot: "Robot",
        program: Union["MotionProgram", CompositeInstruction],
        pipeline: str = "TrajOptPipeline",
        profiles: Optional[ProfileDictionary] = None,
    ) -> PlanningResult:
        """
        Plan a motion program using specified pipeline.

        Args:
            robot: Robot instance
            program: Motion program or CompositeInstruction
            pipeline: Pipeline name (e.g., "TrajOptPipeline", "OMPLPipeline")
            profiles: Custom profiles (uses defaults if None)

        Returns:
            PlanningResult with trajectory if successful
        """
        # Convert MotionProgram to CompositeInstruction if needed
        if hasattr(program, "to_composite_instruction"):
            joint_names = robot.get_joint_names(program.group_name)
            tcp_frame = program.tcp_frame
            if tcp_frame is None:
                manip_info = robot.get_manipulator_info(program.group_name)
                tcp_frame = manip_info.tcp_frame
            composite = program.to_composite_instruction(joint_names, tcp_frame)
        else:
            composite = program

        # Setup profiles
        if profiles is None:
            profiles = ProfileDictionary()

        # Create task
        task = self.factory.createTaskComposerNode(pipeline)
        if task is None:
            return PlanningResult(
                successful=False,
                message=f"Pipeline '{pipeline}' not found",
            )

        output_key = task.getOutputKeys().get("program")
        input_key = task.getInputKeys().get("planning_input")

        # Wrap data for task composer
        program_anypoly = AnyPoly_wrap_CompositeInstruction(composite)
        environment_anypoly = AnyPoly_wrap_EnvironmentConst(robot.env)
        profiles_anypoly = AnyPoly_wrap_ProfileDictionary(profiles)

        task_data = TaskComposerDataStorage()
        task_data.setData(input_key, program_anypoly)
        task_data.setData("environment", environment_anypoly)
        task_data.setData("profiles", profiles_anypoly)

        # Execute
        future = None
        try:
            future = self.executor.run(task, task_data)
            future.wait()

            if not future.context.isSuccessful():
                return PlanningResult(
                    successful=False,
                    message="Planning failed",
                )

            # Extract results
            output_composite = AnyPoly_as_CompositeInstruction(
                future.context.data_storage.getData(output_key)
            )

            trajectory = self._extract_trajectory(output_composite)

            return PlanningResult(
                successful=True,
                message="Planning successful",
                trajectory=trajectory,
                raw_results=output_composite,
            )

        except Exception as e:
            return PlanningResult(
                successful=False,
                message=str(e),
            )

        finally:
            # Cleanup to prevent memory issues
            del task_data
            del program_anypoly
            del environment_anypoly
            del profiles_anypoly
            if future is not None:
                del future

    def plan_freespace(
        self,
        robot: "Robot",
        program: Union["MotionProgram", CompositeInstruction],
        profiles: Optional[ProfileDictionary] = None,
    ) -> PlanningResult:
        """
        Plan freespace motion using TrajOpt pipeline.

        Convenience method that uses TrajOptPipeline.

        Args:
            robot: Robot instance
            program: Motion program
            profiles: Custom profiles

        Returns:
            PlanningResult
        """
        return self.plan(robot, program, pipeline="TrajOptPipeline", profiles=profiles)

    def plan_cartesian(
        self,
        robot: "Robot",
        program: Union["MotionProgram", CompositeInstruction],
        profiles: Optional[ProfileDictionary] = None,
    ) -> PlanningResult:
        """
        Plan Cartesian motion using Descartes pipeline.

        For motions requiring precise Cartesian paths.

        Args:
            robot: Robot instance
            program: Motion program
            profiles: Custom profiles

        Returns:
            PlanningResult
        """
        return self.plan(robot, program, pipeline="DescartesPipeline", profiles=profiles)

    def _extract_trajectory(
        self, composite: CompositeInstruction
    ) -> List[TrajectoryPoint]:
        """Extract trajectory points from CompositeInstruction."""
        trajectory = []

        for instr in composite:
            if not instr.isMoveInstruction():
                continue

            move_instr = InstructionPoly_as_MoveInstructionPoly(instr)
            wp = move_instr.getWaypoint()

            if not wp.isStateWaypoint():
                continue

            state_wp = WaypointPoly_as_StateWaypointPoly(wp)

            point = TrajectoryPoint(
                joint_names=list(state_wp.getNames()),
                positions=np.array(state_wp.getPosition()),
            )

            # Extract optional velocity/acceleration/time if available
            try:
                vel = state_wp.getVelocity()
                if vel is not None and len(vel) > 0:
                    point.velocities = np.array(vel)
            except (AttributeError, RuntimeError):
                pass

            try:
                acc = state_wp.getAcceleration()
                if acc is not None and len(acc) > 0:
                    point.accelerations = np.array(acc)
            except (AttributeError, RuntimeError):
                pass

            try:
                time = state_wp.getTime()
                if time is not None:
                    point.time = float(time)
            except (AttributeError, RuntimeError):
                pass

            trajectory.append(point)

        return trajectory

    def get_available_pipelines(self) -> List[str]:
        """
        Get list of available pipeline names.

        Returns:
            List of pipeline names that can be used with plan()
        """
        # Common pipelines - actual availability depends on config
        return [
            "TrajOptPipeline",
            "OMPLPipeline",
            "DescartesPipeline",
            "FreespaceMotionPipeline",
            "CartesianMotionPipeline",
        ]
