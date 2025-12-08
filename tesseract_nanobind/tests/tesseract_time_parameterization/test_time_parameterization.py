"""Tests for tesseract_time_parameterization bindings.

Tests cover:
- TimeOptimalTrajectoryGeneration (TOTG)
- IterativeSplineParameterization (ISP)
- InstructionsTrajectory container
"""
import numpy as np
import pytest

from tesseract_robotics.tesseract_time_parameterization import (
    TrajectoryContainer,
    InstructionsTrajectory,
    TimeParameterization,
    TimeOptimalTrajectoryGeneration,
    IterativeSplineParameterization,
)
from tesseract_robotics.tesseract_command_language import (
    CompositeInstruction,
    MoveInstruction,
    MoveInstructionPoly_wrap_MoveInstruction,
    JointWaypoint,
    JointWaypointPoly_wrap_JointWaypoint,
    StateWaypoint,
    StateWaypointPoly_wrap_StateWaypoint,
    MoveInstructionType_FREESPACE,
)
from tesseract_robotics.tesseract_common import ManipulatorInfo


class TestTimeOptimalTrajectoryGeneration:
    """Test TOTG time parameterization."""

    def test_default_constructor(self):
        totg = TimeOptimalTrajectoryGeneration()
        assert totg is not None

    def test_custom_parameters(self):
        totg = TimeOptimalTrajectoryGeneration(path_tolerance=0.05, min_angle_change=0.002)
        assert totg is not None

    def test_path_tolerance_only(self):
        totg = TimeOptimalTrajectoryGeneration(path_tolerance=0.2)
        assert totg is not None


class TestIterativeSplineParameterization:
    """Test ISP time parameterization."""

    def test_default_constructor(self):
        isp = IterativeSplineParameterization()
        assert isp is not None

    def test_add_points_true(self):
        isp = IterativeSplineParameterization(add_points=True)
        assert isp is not None

    def test_add_points_false(self):
        isp = IterativeSplineParameterization(add_points=False)
        assert isp is not None


class TestInstructionsTrajectory:
    """Test InstructionsTrajectory container."""

    def test_create_from_composite(self):
        """Test creating InstructionsTrajectory from CompositeInstruction."""
        program = CompositeInstruction("DEFAULT")
        manip = ManipulatorInfo()
        manip.manipulator = "manipulator"
        program.setManipulatorInfo(manip)

        # Add joint waypoints
        joint_names = ["j1", "j2", "j3", "j4", "j5", "j6"]
        positions = [
            np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            np.array([0.1, 0.1, 0.1, 0.1, 0.1, 0.1]),
            np.array([0.2, 0.2, 0.2, 0.2, 0.2, 0.2]),
        ]

        for pos in positions:
            wp = StateWaypoint(joint_names, pos)
            mi = MoveInstruction(
                StateWaypointPoly_wrap_StateWaypoint(wp),
                MoveInstructionType_FREESPACE,
                "DEFAULT"
            )
            program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(mi))

        assert len(program) == 3

        # Create trajectory container
        trajectory = InstructionsTrajectory(program)
        assert trajectory.size() == 3
        assert trajectory.dof() == 6
        assert not trajectory.empty()


class TestTimeParameterizationWorkflow:
    """Integration tests for time parameterization workflow."""

    @pytest.fixture
    def sample_program(self):
        """Create a sample CompositeInstruction with joint waypoints."""
        program = CompositeInstruction("DEFAULT")
        manip = ManipulatorInfo()
        manip.manipulator = "manipulator"
        program.setManipulatorInfo(manip)

        joint_names = ["j1", "j2", "j3", "j4", "j5", "j6"]
        positions = [
            np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            np.array([0.5, 0.5, 0.5, 0.5, 0.5, 0.5]),
            np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0]),
        ]

        for pos in positions:
            wp = StateWaypoint(joint_names, pos)
            mi = MoveInstruction(
                StateWaypointPoly_wrap_StateWaypoint(wp),
                MoveInstructionType_FREESPACE,
                "DEFAULT"
            )
            program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(mi))

        return program

    def test_totg_compute(self, sample_program):
        """Test TOTG compute on trajectory."""
        trajectory = InstructionsTrajectory(sample_program)

        # Limits must be (n_joints, 2) with [min, max] columns
        max_vel = 1.0
        max_acc = 2.0
        max_jerk = 5.0
        vel_limits = np.column_stack((-np.ones(6) * max_vel, np.ones(6) * max_vel))
        acc_limits = np.column_stack((-np.ones(6) * max_acc, np.ones(6) * max_acc))
        jerk_limits = np.column_stack((-np.ones(6) * max_jerk, np.ones(6) * max_jerk))

        totg = TimeOptimalTrajectoryGeneration()
        result = totg.compute(trajectory, vel_limits, acc_limits, jerk_limits)
        assert result is True

        # Verify time was added
        assert trajectory.getTimeFromStart(0) == 0.0
        assert trajectory.getTimeFromStart(1) > 0.0
        assert trajectory.getTimeFromStart(2) > trajectory.getTimeFromStart(1)

    def test_isp_compute(self, sample_program):
        """Test ISP compute on trajectory."""
        trajectory = InstructionsTrajectory(sample_program)

        # Limits must be (n_joints, 2) with [min, max] columns
        max_vel = 1.0
        max_acc = 2.0
        max_jerk = 5.0
        vel_limits = np.column_stack((-np.ones(6) * max_vel, np.ones(6) * max_vel))
        acc_limits = np.column_stack((-np.ones(6) * max_acc, np.ones(6) * max_acc))
        jerk_limits = np.column_stack((-np.ones(6) * max_jerk, np.ones(6) * max_jerk))

        isp = IterativeSplineParameterization()
        result = isp.compute(trajectory, vel_limits, acc_limits, jerk_limits)
        assert result is True

        # Verify time was added
        assert trajectory.getTimeFromStart(0) == 0.0
        assert trajectory.getTimeFromStart(1) > 0.0
        assert trajectory.getTimeFromStart(2) > trajectory.getTimeFromStart(1)

    def test_compute_with_scaling_factors(self, sample_program):
        """Test compute with velocity/acceleration scaling."""
        trajectory = InstructionsTrajectory(sample_program)

        # Limits must be (n_joints, 2) with [min, max] columns
        max_vel = 2.0
        max_acc = 4.0
        max_jerk = 10.0
        vel_limits = np.column_stack((-np.ones(6) * max_vel, np.ones(6) * max_vel))
        acc_limits = np.column_stack((-np.ones(6) * max_acc, np.ones(6) * max_acc))
        jerk_limits = np.column_stack((-np.ones(6) * max_jerk, np.ones(6) * max_jerk))

        # Scale down to 50%
        vel_scale = np.array([0.5])
        acc_scale = np.array([0.5])
        jerk_scale = np.array([0.5])

        totg = TimeOptimalTrajectoryGeneration()
        result = totg.compute(
            trajectory,
            vel_limits,
            acc_limits,
            jerk_limits,
            vel_scale,
            acc_scale,
            jerk_scale
        )
        assert result is True
