"""
Pose helpers for creating poses and transformations.

Provides a clean API for creating transformation matrices without needing
to manipulate numpy arrays directly or chain Isometry3d operations.

Example:
    from tesseract_robotics.planning import Pose, translation, rotation_z

    # Create pose from components
    pose = Pose.from_xyz_rpy(0.5, 0.0, 0.3, 0, 0, 1.57)

    # Or use helper functions
    pose = translation(0.5, 0.0, 0.3) @ rotation_z(1.57)

    # Convert to Isometry3d for low-level API
    isometry = pose.to_isometry()
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Tuple

import numpy as np
from numpy.typing import ArrayLike

from tesseract_robotics.tesseract_common import Isometry3d


@dataclass
class Pose:
    """
    A 3D pose (position + orientation).

    This class provides a Pythonic interface for working with 3D poses,
    wrapping numpy arrays and providing convenient factory methods.

    Attributes:
        matrix: 4x4 homogeneous transformation matrix

    Example:
        # From position and orientation
        p = Pose.from_xyz_quat(0.5, 0, 0.3, 0, 0, 0.707, 0.707)

        # From numpy matrix
        p = Pose(np.eye(4))

        # Chain poses (compose transformations)
        result = p1 @ p2

        # Access components
        pos = p.position  # [x, y, z]
        rot = p.rotation_matrix  # 3x3
    """

    matrix: np.ndarray

    def __post_init__(self):
        """Validate and convert matrix to numpy array."""
        self.matrix = np.asarray(self.matrix, dtype=np.float64)
        if self.matrix.shape != (4, 4):
            raise ValueError(f"Pose matrix must be 4x4, got {self.matrix.shape}")

    @classmethod
    def identity(cls) -> Pose:
        """Create identity pose (no rotation, no translation)."""
        return cls(np.eye(4))

    @classmethod
    def from_xyz(cls, x: float, y: float, z: float) -> Pose:
        """Create pure translation pose."""
        mat = np.eye(4)
        mat[:3, 3] = [x, y, z]
        return cls(mat)

    @classmethod
    def from_position(cls, position: ArrayLike) -> Pose:
        """Create pure translation from position array [x, y, z]."""
        pos = np.asarray(position)
        if pos.shape != (3,):
            raise ValueError(f"Position must have 3 elements, got {pos.shape}")
        return cls.from_xyz(pos[0], pos[1], pos[2])

    @classmethod
    def from_xyz_quat(
        cls,
        x: float,
        y: float,
        z: float,
        qx: float,
        qy: float,
        qz: float,
        qw: float,
    ) -> Pose:
        """
        Create pose from position and quaternion.

        Args:
            x, y, z: Position coordinates
            qx, qy, qz, qw: Quaternion components (scalar-last convention)
        """
        mat = np.eye(4)
        mat[:3, 3] = [x, y, z]
        mat[:3, :3] = _quaternion_to_rotation_matrix(qx, qy, qz, qw)
        return cls(mat)

    @classmethod
    def from_position_quaternion(
        cls,
        position: ArrayLike,
        quaternion: ArrayLike,
    ) -> Pose:
        """
        Create pose from position and quaternion arrays.

        Args:
            position: [x, y, z] position
            quaternion: [qx, qy, qz, qw] quaternion (scalar-last)
        """
        pos = np.asarray(position)
        quat = np.asarray(quaternion)
        return cls.from_xyz_quat(
            pos[0], pos[1], pos[2],
            quat[0], quat[1], quat[2], quat[3],
        )

    @classmethod
    def from_xyz_rpy(
        cls,
        x: float,
        y: float,
        z: float,
        roll: float,
        pitch: float,
        yaw: float,
    ) -> Pose:
        """
        Create pose from position and roll-pitch-yaw angles.

        Args:
            x, y, z: Position coordinates
            roll, pitch, yaw: Euler angles in radians (XYZ convention)
        """
        mat = np.eye(4)
        mat[:3, 3] = [x, y, z]
        mat[:3, :3] = _rpy_to_rotation_matrix(roll, pitch, yaw)
        return cls(mat)

    @classmethod
    def from_matrix(cls, matrix: ArrayLike) -> Pose:
        """Create pose from 4x4 homogeneous matrix."""
        return cls(np.asarray(matrix))

    @classmethod
    def from_matrix_position(cls, rotation: ArrayLike, position: ArrayLike) -> Pose:
        """
        Create pose from 3x3 rotation matrix and position.

        Args:
            rotation: 3x3 rotation matrix
            position: [x, y, z] position

        Example:
            R = np.array([[-1, 0, 0], [0, 1, 0], [0, 0, -1]])
            p = Pose.from_matrix_position(R, [0.5, 0, 0.3])
        """
        mat = np.eye(4)
        mat[:3, :3] = rotation
        mat[:3, 3] = position
        return cls(mat)

    @classmethod
    def from_isometry(cls, isometry: Isometry3d) -> Pose:
        """Create pose from Tesseract Isometry3d."""
        return cls(isometry.matrix())

    @property
    def position(self) -> np.ndarray:
        """Get translation component as [x, y, z]."""
        return self.matrix[:3, 3].copy()

    @property
    def x(self) -> float:
        """Get x position."""
        return float(self.matrix[0, 3])

    @property
    def y(self) -> float:
        """Get y position."""
        return float(self.matrix[1, 3])

    @property
    def z(self) -> float:
        """Get z position."""
        return float(self.matrix[2, 3])

    @property
    def rotation_matrix(self) -> np.ndarray:
        """Get 3x3 rotation matrix."""
        return self.matrix[:3, :3].copy()

    @property
    def quaternion(self) -> np.ndarray:
        """Get quaternion as [qx, qy, qz, qw] (scalar-last)."""
        return _rotation_matrix_to_quaternion(self.matrix[:3, :3])

    @property
    def rpy(self) -> Tuple[float, float, float]:
        """Get roll-pitch-yaw angles in radians."""
        return _rotation_matrix_to_rpy(self.matrix[:3, :3])

    def to_isometry(self) -> Isometry3d:
        """Convert to Tesseract Isometry3d."""
        return Isometry3d(self.matrix)

    def inverse(self) -> Pose:
        """Return the inverse pose."""
        return Pose(np.linalg.inv(self.matrix))

    def __matmul__(self, other: Pose) -> Pose:
        """Chain poses: result = self @ other."""
        if isinstance(other, Pose):
            return Pose(self.matrix @ other.matrix)
        raise TypeError(f"Cannot multiply Pose with {type(other)}")

    def __repr__(self) -> str:
        pos = self.position
        quat = self.quaternion
        return (
            f"Pose(position=[{pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f}], "
            f"quaternion=[{quat[0]:.4f}, {quat[1]:.4f}, {quat[2]:.4f}, {quat[3]:.4f}])"
        )


# Convenience factory functions

def translation(x: float, y: float, z: float) -> Pose:
    """Create pure translation pose."""
    return Pose.from_xyz(x, y, z)


def rotation_x(angle: float) -> Pose:
    """Create rotation around X axis (radians)."""
    c, s = math.cos(angle), math.sin(angle)
    mat = np.eye(4)
    mat[1, 1] = c
    mat[1, 2] = -s
    mat[2, 1] = s
    mat[2, 2] = c
    return Pose(mat)


def rotation_y(angle: float) -> Pose:
    """Create rotation around Y axis (radians)."""
    c, s = math.cos(angle), math.sin(angle)
    mat = np.eye(4)
    mat[0, 0] = c
    mat[0, 2] = s
    mat[2, 0] = -s
    mat[2, 2] = c
    return Pose(mat)


def rotation_z(angle: float) -> Pose:
    """Create rotation around Z axis (radians)."""
    c, s = math.cos(angle), math.sin(angle)
    mat = np.eye(4)
    mat[0, 0] = c
    mat[0, 1] = -s
    mat[1, 0] = s
    mat[1, 1] = c
    return Pose(mat)


def rotation_from_quaternion(
    qx: float, qy: float, qz: float, qw: float
) -> Pose:
    """Create pure rotation from quaternion (scalar-last: qx, qy, qz, qw)."""
    mat = np.eye(4)
    mat[:3, :3] = _quaternion_to_rotation_matrix(qx, qy, qz, qw)
    return Pose(mat)


def rotation_from_axis_angle(
    axis: ArrayLike, angle: float
) -> Pose:
    """
    Create pure rotation from axis-angle representation.

    Args:
        axis: [ax, ay, az] unit axis of rotation
        angle: rotation angle in radians
    """
    axis = np.asarray(axis, dtype=np.float64)
    axis = axis / np.linalg.norm(axis)  # Normalize

    c, s = math.cos(angle), math.sin(angle)
    t = 1 - c
    x, y, z = axis

    mat = np.eye(4)
    mat[0, 0] = t * x * x + c
    mat[0, 1] = t * x * y - s * z
    mat[0, 2] = t * x * z + s * y
    mat[1, 0] = t * x * y + s * z
    mat[1, 1] = t * y * y + c
    mat[1, 2] = t * y * z - s * x
    mat[2, 0] = t * x * z - s * y
    mat[2, 1] = t * y * z + s * x
    mat[2, 2] = t * z * z + c

    return Pose(mat)


# Backwards compatibility alias
Transform = Pose


# Internal helper functions

def _quaternion_to_rotation_matrix(
    qx: float, qy: float, qz: float, qw: float
) -> np.ndarray:
    """Convert quaternion to 3x3 rotation matrix."""
    # Normalize quaternion
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n

    return np.array([
        [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)],
    ])


def _rotation_matrix_to_quaternion(R: np.ndarray) -> np.ndarray:
    """Convert 3x3 rotation matrix to quaternion [qx, qy, qz, qw]."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]

    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        qw = 0.25 / s
        qx = (R[2, 1] - R[1, 2]) * s
        qy = (R[0, 2] - R[2, 0]) * s
        qz = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        qw = (R[2, 1] - R[1, 2]) / s
        qx = 0.25 * s
        qy = (R[0, 1] + R[1, 0]) / s
        qz = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        qw = (R[0, 2] - R[2, 0]) / s
        qx = (R[0, 1] + R[1, 0]) / s
        qy = 0.25 * s
        qz = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        qw = (R[1, 0] - R[0, 1]) / s
        qx = (R[0, 2] + R[2, 0]) / s
        qy = (R[1, 2] + R[2, 1]) / s
        qz = 0.25 * s

    return np.array([qx, qy, qz, qw])


def _rpy_to_rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Convert roll-pitch-yaw (XYZ) to rotation matrix."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def _rotation_matrix_to_rpy(R: np.ndarray) -> Tuple[float, float, float]:
    """Convert rotation matrix to roll-pitch-yaw (XYZ)."""
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)

    if sy > 1e-6:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0

    return (roll, pitch, yaw)
