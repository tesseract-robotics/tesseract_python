"""
Motion program builder with automatic poly-type wrapping.

Provides a fluent builder API for constructing motion programs without
needing to manually wrap waypoints and instructions in poly types.

Example:
    from tesseract_robotics.planning import (
        MotionProgram,
        JointTarget,
        CartesianTarget,
        Pose,
    )

    # Build a program with fluent API
    program = (MotionProgram("manipulator")
        .start_at(JointTarget([0, 0, 0, 0, 0, 0]))
        .move_to(CartesianTarget(Pose.from_xyz(0.5, 0, 0.5)))
        .move_to(JointTarget([0.5, 0, 0, 0, 0, 0]))
    )

    # Access underlying CompositeInstruction for low-level API
    composite = program.to_composite_instruction()
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto
from typing import List, Optional, Sequence, Union

import numpy as np
from numpy.typing import ArrayLike

from tesseract_robotics.tesseract_common import ManipulatorInfo
from tesseract_robotics.tesseract_command_language import (
    CartesianWaypoint,
    JointWaypoint,
    StateWaypoint,
    MoveInstruction,
    CompositeInstruction,
    MoveInstructionType,
    MoveInstructionType_FREESPACE,
    MoveInstructionType_LINEAR,
    MoveInstructionPoly_wrap_MoveInstruction,
    CartesianWaypointPoly_wrap_CartesianWaypoint,
    JointWaypointPoly_wrap_JointWaypoint,
    StateWaypointPoly_wrap_StateWaypoint,
    DEFAULT_PROFILE_KEY,
)

from tesseract_robotics.planning.transforms import Pose


class MoveType(Enum):
    """Motion type for move instructions."""
    FREESPACE = auto()  # Point-to-point, collision-free
    LINEAR = auto()      # Linear interpolation in Cartesian space


@dataclass
class CartesianTarget:
    """
    Cartesian waypoint target (pose in Cartesian space).

    Attributes:
        pose: Target pose as Pose or position/quaternion
        move_type: Motion type (FREESPACE or LINEAR)
        profile: Motion profile name

    Example:
        # From Pose
        target = CartesianTarget(Pose.from_xyz(0.5, 0, 0.5))

        # From position and quaternion
        target = CartesianTarget(
            position=[0.5, 0, 0.5],
            quaternion=[0, 0, 0.707, 0.707],
        )

        # Linear motion
        target = CartesianTarget(
            Pose.from_xyz(0.5, 0, 0.5),
            move_type=MoveType.LINEAR,
        )
    """
    pose: Optional[Pose] = None
    position: Optional[ArrayLike] = None
    quaternion: Optional[ArrayLike] = None
    move_type: MoveType = MoveType.FREESPACE
    profile: str = DEFAULT_PROFILE_KEY

    def __post_init__(self):
        # Construct pose from position/quaternion if provided
        if self.pose is None:
            if self.position is not None:
                pos = np.asarray(self.position)
                if self.quaternion is not None:
                    quat = np.asarray(self.quaternion)
                    self.pose = Pose.from_position_quaternion(pos, quat)
                else:
                    self.pose = Pose.from_position(pos)
            else:
                raise ValueError("Must provide either pose or position")

    def to_waypoint(self) -> CartesianWaypoint:
        """Convert to low-level CartesianWaypoint."""
        return CartesianWaypoint(self.pose.to_isometry())

    def _get_move_type(self) -> MoveInstructionType:
        """Get low-level move instruction type."""
        if self.move_type == MoveType.LINEAR:
            return MoveInstructionType_LINEAR
        return MoveInstructionType_FREESPACE


@dataclass
class JointTarget:
    """
    Joint waypoint target (joint positions).

    Attributes:
        positions: Target joint positions (radians for revolute joints)
        names: Joint names (auto-populated from robot if None)
        move_type: Motion type (FREESPACE or LINEAR)
        profile: Motion profile name

    Example:
        # Just positions (names from robot)
        target = JointTarget([0, 0, 0, -1.57, 0, 0])

        # With explicit names
        target = JointTarget(
            positions=[0, 0, 0, -1.57, 0, 0],
            names=["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
        )
    """
    positions: ArrayLike
    names: Optional[List[str]] = None
    move_type: MoveType = MoveType.FREESPACE
    profile: str = DEFAULT_PROFILE_KEY

    def __post_init__(self):
        self.positions = np.asarray(self.positions, dtype=np.float64)

    def to_waypoint(self, joint_names: Optional[List[str]] = None) -> JointWaypoint:
        """
        Convert to low-level JointWaypoint.

        Args:
            joint_names: Joint names to use if not set on target
        """
        names = self.names or joint_names
        if names is None:
            raise ValueError("Joint names must be provided")
        return JointWaypoint(names, self.positions)

    def _get_move_type(self) -> MoveInstructionType:
        if self.move_type == MoveType.LINEAR:
            return MoveInstructionType_LINEAR
        return MoveInstructionType_FREESPACE


@dataclass
class StateTarget:
    """
    State waypoint target (joint positions with optional velocities/accelerations).

    Used for time-parameterized trajectories or when specifying full state.

    Attributes:
        positions: Joint positions
        names: Joint names
        velocities: Optional joint velocities
        accelerations: Optional joint accelerations
        time: Optional timestamp
        move_type: Motion type
        profile: Motion profile name
    """
    positions: ArrayLike
    names: Optional[List[str]] = None
    velocities: Optional[ArrayLike] = None
    accelerations: Optional[ArrayLike] = None
    time: Optional[float] = None
    move_type: MoveType = MoveType.FREESPACE
    profile: str = DEFAULT_PROFILE_KEY

    def __post_init__(self):
        self.positions = np.asarray(self.positions, dtype=np.float64)
        if self.velocities is not None:
            self.velocities = np.asarray(self.velocities, dtype=np.float64)
        if self.accelerations is not None:
            self.accelerations = np.asarray(self.accelerations, dtype=np.float64)

    def to_waypoint(self, joint_names: Optional[List[str]] = None) -> StateWaypoint:
        """Convert to low-level StateWaypoint."""
        names = self.names or joint_names
        if names is None:
            raise ValueError("Joint names must be provided")

        wp = StateWaypoint(names, self.positions)
        if self.velocities is not None:
            wp.setVelocity(self.velocities)
        if self.accelerations is not None:
            wp.setAcceleration(self.accelerations)
        if self.time is not None:
            wp.setTime(self.time)
        return wp

    def _get_move_type(self) -> MoveInstructionType:
        if self.move_type == MoveType.LINEAR:
            return MoveInstructionType_LINEAR
        return MoveInstructionType_FREESPACE


# Type alias for any target type
Target = Union[CartesianTarget, JointTarget, StateTarget]


class MotionProgram:
    """
    Fluent builder for motion programs.

    Provides a high-level API for building motion programs with automatic
    poly-type wrapping. Supports chaining with method calls.

    Attributes:
        group_name: Kinematic group name (e.g., "manipulator")
        profile: Default motion profile
        targets: List of motion targets

    Example:
        # Build program with chained calls
        program = (MotionProgram("manipulator")
            .start_at(JointTarget([0, 0, 0, 0, 0, 0]))
            .move_to(CartesianTarget(Pose.from_xyz(0.5, 0, 0.5)))
            .linear_to(CartesianTarget(Pose.from_xyz(0.5, 0.2, 0.5)))
            .move_to(JointTarget([0, 0, 0, 0, 0, 0]))
        )

        # Or add targets individually
        program = MotionProgram("manipulator")
        program.add_target(JointTarget([0, 0, 0, 0, 0, 0]))
        program.add_target(CartesianTarget(pose))
    """

    def __init__(
        self,
        group_name: str,
        tcp_frame: Optional[str] = None,
        working_frame: str = "base_link",
        profile: str = DEFAULT_PROFILE_KEY,
    ):
        """
        Initialize motion program builder.

        Args:
            group_name: Kinematic group name
            tcp_frame: TCP frame name (auto-detected if None)
            working_frame: Working/base frame name
            profile: Default motion profile name
        """
        self.group_name = group_name
        self.tcp_frame = tcp_frame
        self.working_frame = working_frame
        self.profile = profile
        self._targets: List[Target] = []
        self._joint_names: Optional[List[str]] = None

    def set_joint_names(self, names: List[str]) -> MotionProgram:
        """
        Set joint names for the program.

        Required if using JointTarget without explicit names.

        Args:
            names: List of joint names

        Returns:
            Self for chaining
        """
        self._joint_names = names
        return self

    def start_at(self, target: Target) -> MotionProgram:
        """
        Set the starting position (alias for add_target).

        Args:
            target: Starting position target

        Returns:
            Self for chaining
        """
        return self.add_target(target)

    def move_to(self, target: Target) -> MotionProgram:
        """
        Add a freespace motion to target.

        Sets target move_type to FREESPACE.

        Args:
            target: Motion target

        Returns:
            Self for chaining
        """
        target.move_type = MoveType.FREESPACE
        return self.add_target(target)

    def linear_to(self, target: Target) -> MotionProgram:
        """
        Add a linear motion to target.

        Sets target move_type to LINEAR.

        Args:
            target: Motion target

        Returns:
            Self for chaining
        """
        target.move_type = MoveType.LINEAR
        return self.add_target(target)

    def add_target(self, target: Target) -> MotionProgram:
        """
        Add a target to the program.

        Args:
            target: Motion target (CartesianTarget, JointTarget, or StateTarget)

        Returns:
            Self for chaining
        """
        self._targets.append(target)
        return self

    def add_targets(self, targets: Sequence[Target]) -> MotionProgram:
        """
        Add multiple targets to the program.

        Args:
            targets: Sequence of motion targets

        Returns:
            Self for chaining
        """
        self._targets.extend(targets)
        return self

    @property
    def targets(self) -> List[Target]:
        """Get list of motion targets."""
        return self._targets

    def __len__(self) -> int:
        """Return number of targets in program."""
        return len(self._targets)

    def to_composite_instruction(
        self,
        joint_names: Optional[List[str]] = None,
        tcp_frame: Optional[str] = None,
    ) -> CompositeInstruction:
        """
        Convert to low-level CompositeInstruction.

        Handles all poly-type wrapping automatically.

        Args:
            joint_names: Joint names (required for JointTarget without names)
            tcp_frame: Override TCP frame

        Returns:
            CompositeInstruction ready for planning
        """
        if not self._targets:
            raise ValueError("Program has no targets")

        joint_names = joint_names or self._joint_names
        tcp_frame = tcp_frame or self.tcp_frame

        # Create manipulator info
        manip_info = ManipulatorInfo()
        manip_info.manipulator = self.group_name
        manip_info.working_frame = self.working_frame
        if tcp_frame:
            manip_info.tcp_frame = tcp_frame

        # Create composite instruction
        program = CompositeInstruction(self.profile)
        program.setManipulatorInfo(manip_info)

        # Add each target as a move instruction
        for target in self._targets:
            instruction = self._target_to_instruction(target, joint_names)
            wrapped = MoveInstructionPoly_wrap_MoveInstruction(instruction)
            program.appendMoveInstruction(wrapped)

        return program

    def _target_to_instruction(
        self,
        target: Target,
        joint_names: Optional[List[str]],
    ) -> MoveInstruction:
        """Convert a target to a MoveInstruction with wrapped waypoint."""
        profile = target.profile or self.profile
        move_type = target._get_move_type()

        if isinstance(target, CartesianTarget):
            waypoint = target.to_waypoint()
            wrapped_wp = CartesianWaypointPoly_wrap_CartesianWaypoint(waypoint)
            return MoveInstruction(wrapped_wp, move_type, profile)

        elif isinstance(target, JointTarget):
            waypoint = target.to_waypoint(joint_names)
            wrapped_wp = JointWaypointPoly_wrap_JointWaypoint(waypoint)
            return MoveInstruction(wrapped_wp, move_type, profile)

        elif isinstance(target, StateTarget):
            waypoint = target.to_waypoint(joint_names)
            wrapped_wp = StateWaypointPoly_wrap_StateWaypoint(waypoint)
            return MoveInstruction(wrapped_wp, move_type, profile)

        else:
            raise TypeError(f"Unknown target type: {type(target)}")

    def __repr__(self) -> str:
        return f"MotionProgram(group={self.group_name!r}, targets={len(self._targets)})"
