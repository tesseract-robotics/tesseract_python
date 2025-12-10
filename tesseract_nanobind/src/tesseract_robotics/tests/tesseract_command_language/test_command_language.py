"""Tests for tesseract_command_language bindings.

These tests cover the command language types used in motion planning examples,
including waypoints, instructions, and composite instructions.
"""
import numpy as np
import pytest

from tesseract_robotics.tesseract_common import Isometry3d, Translation3d, Quaterniond, ManipulatorInfo
from tesseract_robotics.tesseract_command_language import (
    # Waypoint types
    JointWaypoint,
    CartesianWaypoint,
    StateWaypoint,
    # Poly wrappers
    CartesianWaypointPoly,
    JointWaypointPoly,
    StateWaypointPoly,
    CartesianWaypointPoly_wrap_CartesianWaypoint,
    JointWaypointPoly_wrap_JointWaypoint,
    StateWaypointPoly_wrap_StateWaypoint,
    # Instructions
    MoveInstruction,
    MoveInstructionPoly,
    MoveInstructionPoly_wrap_MoveInstruction,
    CompositeInstruction,
    # Enums
    MoveInstructionType,
    MoveInstructionType_FREESPACE,
    MoveInstructionType_LINEAR,
    # Other
    ProfileDictionary,
)


class TestJointWaypoint:
    """Test JointWaypoint creation and methods."""

    def test_default_constructor(self):
        wp = JointWaypoint()
        assert wp is not None

    def test_constructor_with_names_and_position(self):
        names = ["joint_1", "joint_2", "joint_3"]
        position = np.array([0.1, 0.2, 0.3])
        wp = JointWaypoint(names, position)
        assert wp.getNames() == names
        np.testing.assert_array_almost_equal(wp.getPosition(), position)

    def test_constructor_with_constrained_flag(self):
        names = ["j1", "j2"]
        position = np.array([1.0, 2.0])
        wp = JointWaypoint(names, position, False)
        assert not wp.isConstrained()

    def test_set_get_methods(self):
        wp = JointWaypoint()
        names = ["a", "b"]
        pos = np.array([0.5, 1.5])
        wp.setNames(names)
        wp.setPosition(pos)
        assert wp.getNames() == names
        np.testing.assert_array_almost_equal(wp.getPosition(), pos)


class TestCartesianWaypoint:
    """Test CartesianWaypoint creation and methods."""

    def test_default_constructor(self):
        wp = CartesianWaypoint()
        assert wp is not None

    def test_constructor_with_isometry(self):
        transform = Isometry3d.Identity() * Translation3d(1.0, 2.0, 3.0)
        wp = CartesianWaypoint(transform)
        result = wp.getTransform()
        np.testing.assert_array_almost_equal(
            result.translation(), np.array([1.0, 2.0, 3.0])
        )

    def test_constructor_with_quaternion(self):
        # Create transform with rotation
        transform = (
            Isometry3d.Identity()
            * Translation3d(0.8, 0.3, 1.5)
            * Quaterniond(0.707, 0, 0.707, 0)
        )
        wp = CartesianWaypoint(transform)
        assert wp is not None


class TestStateWaypoint:
    """Test StateWaypoint creation and methods."""

    def test_default_constructor(self):
        wp = StateWaypoint()
        assert wp is not None

    def test_constructor_with_names_and_position(self):
        names = ["j1", "j2", "j3", "j4", "j5", "j6"]
        position = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
        wp = StateWaypoint(names, position)
        assert wp.getNames() == names
        np.testing.assert_array_almost_equal(wp.getPosition(), position)


class TestWaypointPoly:
    """Test Poly wrapper types for waypoints."""

    def test_cartesian_waypoint_poly_wrap(self):
        transform = Isometry3d.Identity() * Translation3d(1.0, 0.0, 0.0)
        wp = CartesianWaypoint(transform)
        poly = CartesianWaypointPoly_wrap_CartesianWaypoint(wp)
        assert poly is not None
        assert not poly.isNull()

    def test_joint_waypoint_poly_wrap(self):
        names = ["j1", "j2"]
        position = np.array([0.5, 1.0])
        wp = JointWaypoint(names, position)
        poly = JointWaypointPoly_wrap_JointWaypoint(wp)
        assert poly is not None
        assert not poly.isNull()

    def test_state_waypoint_poly_wrap(self):
        names = ["j1", "j2"]
        position = np.array([0.5, 1.0])
        wp = StateWaypoint(names, position)
        poly = StateWaypointPoly_wrap_StateWaypoint(wp)
        assert poly is not None
        assert not poly.isNull()


class TestMoveInstruction:
    """Test MoveInstruction creation - the core of motion planning."""

    def test_constructor_cartesian_two_args(self):
        """Test MoveInstruction(waypoint, type) - 2-arg form."""
        transform = Isometry3d.Identity() * Translation3d(0.8, 0.3, 1.5)
        wp = CartesianWaypoint(transform)
        poly = CartesianWaypointPoly_wrap_CartesianWaypoint(wp)
        mi = MoveInstruction(poly, MoveInstructionType.FREESPACE)
        assert mi is not None
        assert mi.getMoveType() == MoveInstructionType.FREESPACE

    def test_constructor_cartesian_three_args_with_profile(self):
        """Test MoveInstruction(waypoint, type, profile) - 3-arg SWIG-compatible form."""
        transform = Isometry3d.Identity() * Translation3d(0.8, 0.3, 1.5)
        wp = CartesianWaypoint(transform)
        poly = CartesianWaypointPoly_wrap_CartesianWaypoint(wp)
        mi = MoveInstruction(poly, MoveInstructionType_FREESPACE, "DEFAULT")
        assert mi is not None
        assert mi.getMoveType() == MoveInstructionType.FREESPACE
        assert mi.getProfile() == "DEFAULT"

    def test_constructor_joint_three_args(self):
        """Test MoveInstruction with JointWaypoint and profile."""
        names = ["j1", "j2", "j3"]
        position = np.array([0.1, 0.2, 0.3])
        wp = JointWaypoint(names, position)
        poly = JointWaypointPoly_wrap_JointWaypoint(wp)
        mi = MoveInstruction(poly, MoveInstructionType_LINEAR, "MY_PROFILE")
        assert mi.getMoveType() == MoveInstructionType.LINEAR
        assert mi.getProfile() == "MY_PROFILE"

    def test_move_instruction_poly_wrap(self):
        """Test wrapping MoveInstruction in MoveInstructionPoly."""
        transform = Isometry3d.Identity() * Translation3d(0.5, 0.5, 0.5)
        wp = CartesianWaypoint(transform)
        poly_wp = CartesianWaypointPoly_wrap_CartesianWaypoint(wp)
        mi = MoveInstruction(poly_wp, MoveInstructionType_FREESPACE, "DEFAULT")
        mi_poly = MoveInstructionPoly_wrap_MoveInstruction(mi)
        assert mi_poly is not None
        assert not mi_poly.isNull()


class TestCompositeInstruction:
    """Test CompositeInstruction - container for motion planning programs."""

    def test_default_constructor(self):
        program = CompositeInstruction()
        assert program is not None
        assert program.empty()
        assert len(program) == 0

    def test_constructor_with_profile(self):
        """Test CompositeInstruction(profile) - SWIG-compatible constructor."""
        program = CompositeInstruction("DEFAULT")
        assert program is not None
        assert program.getProfile() == "DEFAULT"

    def test_set_manipulator_info(self):
        program = CompositeInstruction("DEFAULT")
        manip_info = ManipulatorInfo()
        manip_info.tcp_frame = "tool0"
        manip_info.manipulator = "manipulator"
        manip_info.working_frame = "base_link"
        program.setManipulatorInfo(manip_info)
        result = program.getManipulatorInfo()
        assert result.tcp_frame == "tool0"

    def test_append_move_instruction(self):
        """Test appending MoveInstructionPoly to CompositeInstruction."""
        program = CompositeInstruction("DEFAULT")

        # Create waypoints and instructions
        wp1 = CartesianWaypoint(Isometry3d.Identity() * Translation3d(0.8, -0.3, 1.5))
        wp2 = CartesianWaypoint(Isometry3d.Identity() * Translation3d(0.8, 0.3, 1.5))

        mi1 = MoveInstruction(
            CartesianWaypointPoly_wrap_CartesianWaypoint(wp1),
            MoveInstructionType_FREESPACE,
            "DEFAULT",
        )
        mi2 = MoveInstruction(
            CartesianWaypointPoly_wrap_CartesianWaypoint(wp2),
            MoveInstructionType_FREESPACE,
            "DEFAULT",
        )

        program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(mi1))
        program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(mi2))

        assert len(program) == 2
        assert not program.empty()


class TestProfileDictionary:
    """Test ProfileDictionary for motion planning profiles."""

    def test_default_constructor(self):
        profiles = ProfileDictionary()
        assert profiles is not None


class TestABBExampleWorkflow:
    """Integration test simulating the ABB IRB2400 viewer example workflow.

    This tests the exact sequence of operations used in abb_irb2400_viewer.py
    to ensure users can run examples without errors.
    """

    def test_complete_command_language_workflow(self):
        """Test the command_language portion of the ABB example."""
        # Setup ManipulatorInfo (as in example)
        manip_info = ManipulatorInfo()
        manip_info.tcp_frame = "tool0"
        manip_info.manipulator = "manipulator"
        manip_info.working_frame = "base_link"

        # Define waypoints (as in example)
        wp1 = CartesianWaypoint(
            Isometry3d.Identity()
            * Translation3d(0.8, -0.3, 1.455)
            * Quaterniond(0.70710678, 0, 0.70710678, 0)
        )
        wp2 = CartesianWaypoint(
            Isometry3d.Identity()
            * Translation3d(0.8, 0.3, 1.455)
            * Quaterniond(0.70710678, 0, 0.70710678, 0)
        )
        wp3 = CartesianWaypoint(
            Isometry3d.Identity()
            * Translation3d(0.8, 0.5, 1.455)
            * Quaterniond(0.70710678, 0, 0.70710678, 0)
        )

        # Create instructions (as in example - 3-arg form with profile)
        start_instruction = MoveInstruction(
            CartesianWaypointPoly_wrap_CartesianWaypoint(wp1),
            MoveInstructionType_FREESPACE,
            "DEFAULT",
        )
        plan_f1 = MoveInstruction(
            CartesianWaypointPoly_wrap_CartesianWaypoint(wp2),
            MoveInstructionType_FREESPACE,
            "DEFAULT",
        )
        plan_f2 = MoveInstruction(
            CartesianWaypointPoly_wrap_CartesianWaypoint(wp3),
            MoveInstructionType_FREESPACE,
            "DEFAULT",
        )

        # Create program (as in example)
        program = CompositeInstruction("DEFAULT")
        program.setManipulatorInfo(manip_info)
        program.appendMoveInstruction(
            MoveInstructionPoly_wrap_MoveInstruction(start_instruction)
        )
        program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(plan_f1))

        assert len(program) == 2
        assert program.getProfile() == "DEFAULT"
        result_info = program.getManipulatorInfo()
        assert result_info.tcp_frame == "tool0"


class TestMoveInstructionDescription:
    """Test MoveInstruction.setDescription and getDescription methods."""

    def test_set_get_description(self):
        """Test setDescription and getDescription on MoveInstruction."""
        transform = Isometry3d.Identity() * Translation3d(0.5, 0.5, 0.5)
        wp = CartesianWaypoint(transform)
        poly_wp = CartesianWaypointPoly_wrap_CartesianWaypoint(wp)
        mi = MoveInstruction(poly_wp, MoveInstructionType_FREESPACE, "DEFAULT")

        # Set description
        mi.setDescription("test_waypoint_1")
        assert mi.getDescription() == "test_waypoint_1"

        # Change description
        mi.setDescription("updated_waypoint")
        assert mi.getDescription() == "updated_waypoint"

    def test_description_on_move_instruction_poly(self):
        """Test setDescription on MoveInstructionPoly."""
        transform = Isometry3d.Identity() * Translation3d(0.5, 0.5, 0.5)
        wp = CartesianWaypoint(transform)
        poly_wp = CartesianWaypointPoly_wrap_CartesianWaypoint(wp)
        mi = MoveInstruction(poly_wp, MoveInstructionType_FREESPACE, "DEFAULT")
        mi_poly = MoveInstructionPoly_wrap_MoveInstruction(mi)

        # MoveInstructionPoly also has setDescription
        mi_poly.setDescription("poly_description")
        assert mi_poly.getDescription() == "poly_description"


class TestAnyPolyWrappers:
    """Test AnyPoly wrapper functions for TaskComposerDataStorage."""

    def test_anypoly_wrap_composite_instruction(self):
        """Test wrapping CompositeInstruction in AnyPoly."""
        from tesseract_robotics.tesseract_command_language import (
            AnyPoly_wrap_CompositeInstruction,
        )

        program = CompositeInstruction("DEFAULT")
        # Add some instructions
        wp = CartesianWaypoint(Isometry3d.Identity() * Translation3d(0.5, 0.5, 0.5))
        mi = MoveInstruction(
            CartesianWaypointPoly_wrap_CartesianWaypoint(wp),
            MoveInstructionType_FREESPACE,
            "DEFAULT",
        )
        program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(mi))

        # Wrap in AnyPoly
        any_poly = AnyPoly_wrap_CompositeInstruction(program)
        assert any_poly is not None
        assert not any_poly.isNull()

    def test_anypoly_wrap_profile_dictionary(self):
        """Test wrapping ProfileDictionary in AnyPoly."""
        from tesseract_robotics.tesseract_command_language import (
            AnyPoly_wrap_ProfileDictionary,
        )

        profiles = ProfileDictionary()
        any_poly = AnyPoly_wrap_ProfileDictionary(profiles)
        assert any_poly is not None
        assert not any_poly.isNull()

    def test_anypoly_roundtrip_composite_instruction(self):
        """Test wrapping and unwrapping CompositeInstruction via AnyPoly."""
        from tesseract_robotics.tesseract_command_language import (
            AnyPoly_wrap_CompositeInstruction,
            AnyPoly_as_CompositeInstruction,
        )

        # Create program with specific content
        program = CompositeInstruction("MY_PROFILE")
        wp1 = CartesianWaypoint(Isometry3d.Identity() * Translation3d(1.0, 0.0, 0.0))
        wp2 = CartesianWaypoint(Isometry3d.Identity() * Translation3d(0.0, 1.0, 0.0))
        mi1 = MoveInstruction(
            CartesianWaypointPoly_wrap_CartesianWaypoint(wp1),
            MoveInstructionType_FREESPACE,
            "DEFAULT",
        )
        mi2 = MoveInstruction(
            CartesianWaypointPoly_wrap_CartesianWaypoint(wp2),
            MoveInstructionType_LINEAR,
            "DEFAULT",
        )
        program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(mi1))
        program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(mi2))

        # Wrap
        any_poly = AnyPoly_wrap_CompositeInstruction(program)

        # Unwrap
        recovered = AnyPoly_as_CompositeInstruction(any_poly)
        assert recovered is not None
        assert len(recovered) == 2
        assert recovered.getProfile() == "MY_PROFILE"
