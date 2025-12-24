"""Tests for tesseract_motion_planners bindings."""
import numpy as np
import pytest

from tesseract_robotics.tesseract_common import (
    Isometry3d,
    Translation3d,
    Quaterniond,
    ManipulatorInfo,
    GeneralResourceLocator,
    FilesystemPath,
)
from tesseract_robotics.tesseract_environment import Environment
from tesseract_robotics.tesseract_command_language import (
    CartesianWaypoint,
    CartesianWaypointPoly_wrap_CartesianWaypoint,
    MoveInstruction,
    MoveInstructionPoly_wrap_MoveInstruction,
    CompositeInstruction,
    MoveInstructionType_FREESPACE,
    ProfileDictionary,
)
from tesseract_robotics.tesseract_motion_planners import PlannerRequest, PlannerResponse
from tesseract_robotics.tesseract_motion_planners_ompl import (
    OMPLMotionPlanner,
    OMPLRealVectorPlanProfile,
    ProfileDictionary_addOMPLProfile,
)
from tesseract_robotics.tesseract_motion_planners_simple import (
    generateInterpolatedProgram,
)


OMPL_DEFAULT_NAMESPACE = "OMPLMotionPlannerTask"


@pytest.fixture
def abb_irb2400_environment():
    """Load ABB IRB2400 robot environment for testing."""
    locator = GeneralResourceLocator()
    urdf_path = FilesystemPath(locator.locateResource("package://tesseract_support/urdf/abb_irb2400.urdf").getFilePath())
    srdf_path = FilesystemPath(locator.locateResource("package://tesseract_support/urdf/abb_irb2400.srdf").getFilePath())
    t_env = Environment()
    assert t_env.init(urdf_path, srdf_path, locator), "Failed to initialize ABB IRB2400"
    return t_env


class TestPlannerRequest:
    """Test PlannerRequest creation."""

    def test_default_constructor(self):
        request = PlannerRequest()
        assert request is not None


class TestOMPLMotionPlanner:
    """Test OMPL motion planner."""

    def test_constructor(self):
        planner = OMPLMotionPlanner(OMPL_DEFAULT_NAMESPACE)
        assert planner is not None

    def test_plan_profile(self):
        profile = OMPLRealVectorPlanProfile()
        assert profile is not None


class TestOMPLPlanning:
    """Integration test for OMPL motion planning with ABB robot."""

    def test_ompl_planning_workflow(self, abb_irb2400_environment):
        """Test complete OMPL planning workflow as in abb_irb2400_viewer.py."""
        t_env = abb_irb2400_environment

        # Setup manipulator info
        manip_info = ManipulatorInfo()
        manip_info.tcp_frame = "tool0"
        manip_info.manipulator = "manipulator"
        manip_info.working_frame = "base_link"

        # Set initial state
        joint_names = [f"joint_{i+1}" for i in range(6)]
        t_env.setState(joint_names, np.ones(6) * 0.1)

        # Define waypoints
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

        # Create instructions
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

        # Create program
        program = CompositeInstruction("DEFAULT")
        program.setManipulatorInfo(manip_info)
        program.appendMoveInstruction(
            MoveInstructionPoly_wrap_MoveInstruction(start_instruction)
        )
        program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(plan_f1))

        # Setup OMPL planner - use helper function for cross-module profile registration
        plan_profile = OMPLRealVectorPlanProfile()
        profiles = ProfileDictionary()
        ProfileDictionary_addOMPLProfile(profiles, OMPL_DEFAULT_NAMESPACE, "DEFAULT", plan_profile)

        # Create request
        request = PlannerRequest()
        request.instructions = program
        request.env = t_env
        request.profiles = profiles

        # Solve
        ompl_planner = OMPLMotionPlanner(OMPL_DEFAULT_NAMESPACE)
        response = ompl_planner.solve(request)

        assert response.successful, f"OMPL planning failed: {response.message}"
        assert response.results is not None

        # Interpolate results
        interpolated_results = generateInterpolatedProgram(
            response.results, t_env, 3.14, 1.0, 3.14, 10
        )
        assert interpolated_results is not None


class TestSimplePlanner:
    """Test simple motion planner utilities."""

    def test_generate_interpolated_program(self, abb_irb2400_environment):
        """Test generateInterpolatedProgram function."""
        t_env = abb_irb2400_environment

        # Setup manipulator info
        manip_info = ManipulatorInfo()
        manip_info.tcp_frame = "tool0"
        manip_info.manipulator = "manipulator"
        manip_info.working_frame = "base_link"

        # Set initial state
        joint_names = [f"joint_{i+1}" for i in range(6)]
        t_env.setState(joint_names, np.ones(6) * 0.1)

        # Create simple program with one move
        wp1 = CartesianWaypoint(
            Isometry3d.Identity()
            * Translation3d(0.8, 0.0, 1.455)
            * Quaterniond(0.70710678, 0, 0.70710678, 0)
        )

        mi = MoveInstruction(
            CartesianWaypointPoly_wrap_CartesianWaypoint(wp1),
            MoveInstructionType_FREESPACE,
            "DEFAULT",
        )

        program = CompositeInstruction("DEFAULT")
        program.setManipulatorInfo(manip_info)
        program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(mi))

        # First solve with OMPL to get a valid result - use helper function
        plan_profile = OMPLRealVectorPlanProfile()
        profiles = ProfileDictionary()
        ProfileDictionary_addOMPLProfile(profiles, OMPL_DEFAULT_NAMESPACE, "DEFAULT", plan_profile)

        request = PlannerRequest()
        request.instructions = program
        request.env = t_env
        request.profiles = profiles

        ompl_planner = OMPLMotionPlanner(OMPL_DEFAULT_NAMESPACE)
        response = ompl_planner.solve(request)

        if response.successful:
            # Now test interpolation
            interpolated = generateInterpolatedProgram(
                response.results, t_env, 3.14, 1.0, 3.14, 10
            )
            assert interpolated is not None
