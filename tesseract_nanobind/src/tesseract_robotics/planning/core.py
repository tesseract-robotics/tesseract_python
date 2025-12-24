"""
Core classes for robot environment management.

Provides high-level interfaces for loading and managing robot environments,
wrapping the lower-level Environment and related classes.

Example:
    from tesseract_robotics.planning import Robot

    # Load from package URLs
    robot = Robot.from_urdf(
        "package://tesseract_support/urdf/abb_irb2400.urdf",
        "package://tesseract_support/urdf/abb_irb2400.srdf"
    )

    # Or from file paths
    robot = Robot.from_files("/path/to/robot.urdf", "/path/to/robot.srdf")

    # Access robot state
    state = robot.get_state()
    print(state.joint_positions)

    # Set joint positions
    robot.set_joints({"joint_1": 0.5, "joint_2": -0.3})
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Union

import numpy as np
from numpy.typing import ArrayLike

from tesseract_robotics.tesseract_common import (
    GeneralResourceLocator,
    FilesystemPath,
    ManipulatorInfo,
    Isometry3d,
)
from tesseract_robotics.tesseract_environment import Environment
from tesseract_robotics.tesseract_kinematics import KinGroupIKInput
from tesseract_robotics.tesseract_scene_graph import (
    Joint,
    Link,
    SceneGraph,
)
from tesseract_robotics.tesseract_environment import (
    AddLinkCommand,
    RemoveLinkCommand,
    MoveLinkCommand,
    ModifyAllowedCollisionsCommand,
    ModifyAllowedCollisionsType,
    ChangeCollisionMarginsCommand,
)

from tesseract_robotics.planning.transforms import Pose


@dataclass
class RobotState:
    """
    Snapshot of robot joint state.

    Attributes:
        joint_names: List of joint names
        joint_positions: Array of joint positions (radians for revolute)
        joint_velocities: Array of joint velocities (optional)
        joint_accelerations: Array of joint accelerations (optional)
    """
    joint_names: List[str]
    joint_positions: np.ndarray
    joint_velocities: Optional[np.ndarray] = None
    joint_accelerations: Optional[np.ndarray] = None

    def __post_init__(self):
        self.joint_positions = np.asarray(self.joint_positions, dtype=np.float64)
        if self.joint_velocities is not None:
            self.joint_velocities = np.asarray(self.joint_velocities, dtype=np.float64)
        if self.joint_accelerations is not None:
            self.joint_accelerations = np.asarray(self.joint_accelerations, dtype=np.float64)

    def as_dict(self) -> Dict[str, float]:
        """Return joint positions as {name: position} dictionary."""
        return dict(zip(self.joint_names, self.joint_positions))

    def __getitem__(self, joint_name: str) -> float:
        """Get position of a specific joint by name."""
        try:
            idx = self.joint_names.index(joint_name)
            return float(self.joint_positions[idx])
        except ValueError:
            raise KeyError(f"Joint '{joint_name}' not found")

    def __repr__(self) -> str:
        joints_str = ", ".join(
            f"{n}={p:.4f}" for n, p in zip(self.joint_names, self.joint_positions)
        )
        return f"RobotState({joints_str})"


class Robot:
    """
    High-level interface for robot environment management.

    Wraps the Tesseract Environment with a Pythonic interface for loading
    robots, managing state, and accessing kinematic information.

    Attributes:
        env: Underlying Tesseract Environment
        locator: Resource locator for resolving package:// URLs

    Example:
        robot = Robot.from_urdf(
            "package://my_robot/urdf/robot.urdf",
            "package://my_robot/urdf/robot.srdf"
        )

        # Get joint names for a manipulator
        joints = robot.get_joint_names("manipulator")

        # Compute forward kinematics
        pose = robot.fk("manipulator", [0, 0, 0, 0, 0, 0])

        # Compute inverse kinematics
        solutions = robot.ik("manipulator", target_pose)
    """

    def __init__(
        self,
        env: Environment,
        locator: Optional[GeneralResourceLocator] = None,
    ):
        """
        Initialize Robot wrapper.

        Args:
            env: Initialized Tesseract Environment
            locator: Resource locator (created if not provided)
        """
        self.env = env
        self.locator = locator or GeneralResourceLocator()
        self._manipulator_cache: Dict[str, ManipulatorInfo] = {}

    @classmethod
    def from_urdf(
        cls,
        urdf_url: str,
        srdf_url: str,
        locator: Optional[GeneralResourceLocator] = None,
    ) -> Robot:
        """
        Load robot from URDF and SRDF URLs.

        Supports both package:// URLs and file:// URLs.

        Args:
            urdf_url: URL to URDF file (e.g., "package://my_robot/urdf/robot.urdf")
            srdf_url: URL to SRDF file (e.g., "package://my_robot/urdf/robot.srdf")
            locator: Resource locator (uses GeneralResourceLocator if not provided)

        Returns:
            Initialized Robot instance

        Raises:
            RuntimeError: If environment initialization fails
        """
        locator = locator or GeneralResourceLocator()

        urdf_path = FilesystemPath(locator.locateResource(urdf_url).getFilePath())
        srdf_path = FilesystemPath(locator.locateResource(srdf_url).getFilePath())

        env = Environment()
        if not env.init(urdf_path, srdf_path, locator):
            raise RuntimeError(f"Failed to initialize environment from {urdf_url}")

        return cls(env, locator)

    @classmethod
    def from_files(
        cls,
        urdf_path: Union[str, Path],
        srdf_path: Union[str, Path],
        locator: Optional[GeneralResourceLocator] = None,
    ) -> Robot:
        """
        Load robot from local URDF and SRDF files.

        Args:
            urdf_path: Path to URDF file
            srdf_path: Path to SRDF file
            locator: Resource locator for resolving mesh references

        Returns:
            Initialized Robot instance
        """
        locator = locator or GeneralResourceLocator()

        urdf_path = FilesystemPath(str(urdf_path))
        srdf_path = FilesystemPath(str(srdf_path))

        env = Environment()
        if not env.init(urdf_path, srdf_path, locator):
            raise RuntimeError(f"Failed to initialize environment from {urdf_path}")

        return cls(env, locator)

    @classmethod
    def from_tesseract_support(
        cls,
        robot_name: str,
        locator: Optional[GeneralResourceLocator] = None,
    ) -> Robot:
        """
        Load robot from tesseract_support package.

        Convenience method for loading standard test robots.

        Args:
            robot_name: Name of robot (e.g., "abb_irb2400", "lbr_iiwa_14_r820")

        Returns:
            Initialized Robot instance
        """
        urdf_url = f"package://tesseract_support/urdf/{robot_name}.urdf"
        srdf_url = f"package://tesseract_support/urdf/{robot_name}.srdf"
        return cls.from_urdf(urdf_url, srdf_url, locator)

    def get_state(self, joint_names: Optional[List[str]] = None) -> RobotState:
        """
        Get current robot state.

        Args:
            joint_names: Specific joints to query (all if None)

        Returns:
            Current robot state
        """
        env_state = self.env.getState()

        if joint_names is None:
            # Get all joint names from the state's joints dict
            joint_names = list(env_state.joints.keys())

        positions = env_state.getJointValues(joint_names)

        return RobotState(
            joint_names=joint_names,
            joint_positions=positions,
        )

    def set_joints(
        self,
        joint_values: Union[Dict[str, float], Sequence[float]],
        joint_names: Optional[List[str]] = None,
    ) -> None:
        """
        Set robot joint positions.

        Args:
            joint_values: Joint positions as dict or sequence
            joint_names: Joint names (required if joint_values is sequence)
        """
        if isinstance(joint_values, dict):
            names = list(joint_values.keys())
            values = np.array(list(joint_values.values()), dtype=np.float64)
        else:
            if joint_names is None:
                raise ValueError("joint_names required when joint_values is a sequence")
            names = joint_names
            values = np.asarray(joint_values, dtype=np.float64)

        self.env.setState(names, values)

    def get_joint_names(self, group_name: str) -> List[str]:
        """
        Get joint names for a kinematic group.

        Args:
            group_name: Name of the kinematic group (e.g., "manipulator")

        Returns:
            List of joint names
        """
        group = self.env.getJointGroup(group_name)
        return list(group.getJointNames())

    def get_link_names(self) -> List[str]:
        """Get all link names in the robot."""
        return list(self.env.getLinkNames())

    def get_joint_limits(self, group_name: str) -> Dict[str, Dict[str, float]]:
        """
        Get joint limits for a kinematic group.

        Args:
            group_name: Name of the kinematic group

        Returns:
            Dictionary mapping joint name to {lower, upper, velocity, acceleration}
        """
        group = self.env.getKinematicGroup(group_name)
        limits = group.getLimits()

        result = {}
        joint_names = list(group.getJointNames())
        for i, name in enumerate(joint_names):
            result[name] = {
                "lower": float(limits.joint_limits[i, 0]),
                "upper": float(limits.joint_limits[i, 1]),
                "velocity": float(limits.velocity_limits[i]),
                "acceleration": float(limits.acceleration_limits[i]),
            }
        return result

    def fk(
        self,
        group_name: str,
        joint_positions: ArrayLike,
        tip_link: Optional[str] = None,
    ) -> Pose:
        """
        Compute forward kinematics.

        Args:
            group_name: Kinematic group name
            joint_positions: Joint positions array
            tip_link: Target link (uses default TCP if None)

        Returns:
            Pose of the tip link
        """
        group = self.env.getKinematicGroup(group_name)
        joint_positions = np.asarray(joint_positions, dtype=np.float64)

        poses = group.calcFwdKin(joint_positions)

        if tip_link is None:
            # Use first tip link
            tip_link = list(group.getActiveLinkNames())[-1]

        return Pose.from_isometry(poses[tip_link])

    def ik(
        self,
        group_name: str,
        target_pose: Union[Pose, Isometry3d],
        seed: Optional[ArrayLike] = None,
        tip_link: Optional[str] = None,
    ) -> Optional[np.ndarray]:
        """
        Compute inverse kinematics.

        Args:
            group_name: Kinematic group name
            target_pose: Desired end-effector pose
            seed: Initial joint configuration (current state if None)
            tip_link: Target link name

        Returns:
            Joint positions array, or None if no solution found
        """
        group = self.env.getKinematicGroup(group_name)

        if isinstance(target_pose, Pose):
            target_pose = target_pose.to_isometry()

        if seed is None:
            seed = self.get_state(list(group.getJointNames())).joint_positions
        else:
            seed = np.asarray(seed, dtype=np.float64)

        if tip_link is None:
            tip_link = list(group.getActiveLinkNames())[-1]

        # Get working frame from group
        working_frame = group.getBaseLinkName()

        # Create proper IK input
        ik_input = KinGroupIKInput(target_pose, working_frame, tip_link)

        result = group.calcInvKin(ik_input, seed)

        if result is None or len(result) == 0:
            return None

        return result[0]

    def get_manipulator_info(
        self,
        group_name: str,
        tcp_frame: Optional[str] = None,
        working_frame: str = "base_link",
    ) -> ManipulatorInfo:
        """
        Get or create ManipulatorInfo for a kinematic group.

        Args:
            group_name: Kinematic group name
            tcp_frame: TCP frame name (auto-detected if None)
            working_frame: Working frame name

        Returns:
            Configured ManipulatorInfo
        """
        cache_key = f"{group_name}:{tcp_frame}:{working_frame}"

        if cache_key not in self._manipulator_cache:
            info = ManipulatorInfo()
            info.manipulator = group_name

            if tcp_frame is None:
                # Auto-detect from kinematic group
                group = self.env.getKinematicGroup(group_name)
                tcp_frame = list(group.getActiveLinkNames())[-1]

            info.tcp_frame = tcp_frame
            info.working_frame = working_frame

            self._manipulator_cache[cache_key] = info

        return self._manipulator_cache[cache_key]

    def add_link(
        self,
        link: Link,
        joint: Joint,
    ) -> bool:
        """
        Add a new link to the environment.

        Args:
            link: Link to add
            joint: Joint connecting link to parent

        Returns:
            True if successful
        """
        cmd = AddLinkCommand(link, joint)
        return self.env.applyCommand(cmd)

    def remove_link(self, link_name: str) -> bool:
        """
        Remove a link from the environment.

        Args:
            link_name: Name of link to remove

        Returns:
            True if successful
        """
        cmd = RemoveLinkCommand(link_name)
        return self.env.applyCommand(cmd)

    def move_link(
        self,
        joint: Joint,
    ) -> bool:
        """
        Move a link by providing a new joint configuration.

        This attaches the child link to a new parent link via the joint.

        Args:
            joint: Joint object specifying new parent-child relationship

        Returns:
            True if successful
        """
        cmd = MoveLinkCommand(joint)
        return self.env.applyCommand(cmd)

    def add_allowed_collision(
        self,
        link1: str,
        link2: str,
        reason: str = "Adjacent",
    ) -> bool:
        """
        Add an allowed collision pair.

        Args:
            link1: First link name
            link2: Second link name
            reason: Reason for allowing collision

        Returns:
            True if successful
        """
        from tesseract_robotics.tesseract_common import AllowedCollisionMatrix
        acm = AllowedCollisionMatrix()
        acm.addAllowedCollision(link1, link2, reason)
        cmd = ModifyAllowedCollisionsCommand(acm, ModifyAllowedCollisionsType.ADD)
        return self.env.applyCommand(cmd)

    def set_collision_margin(self, margin: float) -> bool:
        """
        Set default collision margin distance.

        Args:
            margin: Collision margin in meters

        Returns:
            True if successful
        """
        cmd = ChangeCollisionMarginsCommand(margin)
        return self.env.applyCommand(cmd)

    @property
    def scene_graph(self) -> SceneGraph:
        """Access the underlying scene graph."""
        return self.env.getSceneGraph()

    def __repr__(self) -> str:
        links = len(self.get_link_names())
        return f"Robot(links={links})"
