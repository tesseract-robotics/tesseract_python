"""Tests for tesseract_motion_planners_trajopt bindings.

These tests cover TrajOpt motion planner and profiles.
"""
import os
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
    JointWaypoint,
    CartesianWaypoint,
    JointWaypointPoly_wrap_JointWaypoint,
    CartesianWaypointPoly_wrap_CartesianWaypoint,
    MoveInstruction,
    MoveInstructionPoly_wrap_MoveInstruction,
    InstructionPoly_as_MoveInstructionPoly,
    WaypointPoly_as_StateWaypointPoly,
    CompositeInstruction,
    MoveInstructionType_FREESPACE,
    ProfileDictionary,
)
from tesseract_robotics.tesseract_motion_planners import PlannerRequest
from tesseract_robotics.tesseract_motion_planners_simple import generateInterpolatedProgram

# TrajOpt imports - skip tests if not available
try:
    from tesseract_robotics.tesseract_motion_planners_trajopt import (
        TrajOptMotionPlanner,
        TrajOptDefaultPlanProfile,
        TrajOptDefaultCompositeProfile,
        CollisionCostConfig,
        CollisionConstraintConfig,
        CollisionEvaluatorType,
        ProfileDictionary_addTrajOptPlanProfile,
        ProfileDictionary_addTrajOptCompositeProfile,
    )
    TRAJOPT_AVAILABLE = True
except ImportError:
    TRAJOPT_AVAILABLE = False


TRAJOPT_DEFAULT_NAMESPACE = "TrajOptMotionPlannerTask"


@pytest.fixture
def lbr_iiwa_environment():
    """Load LBR IIWA robot environment for testing (same as SWIG tests)."""
    tesseract_support = os.environ.get("TESSERACT_SUPPORT_DIR")
    if not tesseract_support:
        pytest.skip("TESSERACT_SUPPORT_DIR not set")

    locator = GeneralResourceLocator()

    urdf_path = FilesystemPath(os.path.join(tesseract_support, "urdf/lbr_iiwa_14_r820.urdf"))
    srdf_path = FilesystemPath(os.path.join(tesseract_support, "urdf/lbr_iiwa_14_r820.srdf"))

    t_env = Environment()
    success = t_env.init(urdf_path, srdf_path, locator)
    if not success:
        pytest.skip("Failed to initialize LBR IIWA environment")

    manip_info = ManipulatorInfo()
    manip_info.tcp_frame = "tool0"
    manip_info.manipulator = "manipulator"
    manip_info.working_frame = "base_link"
    manip_info.manipulator_ik_solver = "KDLInvKinChainLMA"

    joint_names = list(t_env.getJointGroup("manipulator").getJointNames())

    return t_env, manip_info, joint_names


@pytest.mark.skipif(not TRAJOPT_AVAILABLE, reason="TrajOpt bindings not available")
class TestTrajOptProfiles:
    """Test TrajOpt profile types."""

    def test_default_plan_profile(self):
        profile = TrajOptDefaultPlanProfile()
        assert profile is not None

    def test_default_composite_profile(self):
        profile = TrajOptDefaultCompositeProfile()
        assert profile is not None
        # Test default attributes
        assert hasattr(profile, "smooth_velocities")
        assert hasattr(profile, "smooth_accelerations")
        assert hasattr(profile, "smooth_jerks")
        assert hasattr(profile, "collision_cost_config")

    def test_collision_cost_config(self):
        config = CollisionCostConfig()
        assert config is not None
        config.enabled = True
        config.safety_margin = 0.025
        config.type = CollisionEvaluatorType.DISCRETE_CONTINUOUS
        assert config.enabled is True
        assert config.safety_margin == 0.025
        assert config.type == CollisionEvaluatorType.DISCRETE_CONTINUOUS

    def test_collision_constraint_config(self):
        config = CollisionConstraintConfig()
        assert config is not None
        config.enabled = True
        config.safety_margin = 0.01
        assert config.enabled is True
        assert config.safety_margin == 0.01

    def test_collision_evaluator_type_enum(self):
        assert CollisionEvaluatorType.SINGLE_TIMESTEP is not None
        assert CollisionEvaluatorType.DISCRETE_CONTINUOUS is not None
        assert CollisionEvaluatorType.CAST_CONTINUOUS is not None


@pytest.mark.skipif(not TRAJOPT_AVAILABLE, reason="TrajOpt bindings not available")
class TestTrajOptMotionPlanner:
    """Test TrajOpt motion planner."""

    def test_constructor(self):
        planner = TrajOptMotionPlanner(TRAJOPT_DEFAULT_NAMESPACE)
        assert planner is not None
        assert planner.getName() == TRAJOPT_DEFAULT_NAMESPACE


@pytest.mark.skipif(not TRAJOPT_AVAILABLE, reason="TrajOpt bindings not available")
class TestTrajOptPlanning:
    """Integration test for TrajOpt motion planning."""

    def test_trajopt_freespace_joint_cart(self, lbr_iiwa_environment):
        """Test TrajOpt planning from joint to cartesian waypoint."""
        env, manip_info, joint_names = lbr_iiwa_environment

        # Start at joint position
        wp1 = JointWaypoint(joint_names, np.array([0, 0, 0, -1.57, 0, 0, 0], dtype=np.float64))
        # End at cartesian position
        wp2 = CartesianWaypoint(
            Isometry3d.Identity()
            * Translation3d(-0.2, 0.4, 0.2)
            * Quaterniond(0, 0, 1.0, 0)
        )

        start_instruction = MoveInstruction(
            JointWaypointPoly_wrap_JointWaypoint(wp1),
            MoveInstructionType_FREESPACE,
            "TEST_PROFILE",
        )
        plan_f1 = MoveInstruction(
            CartesianWaypointPoly_wrap_CartesianWaypoint(wp2),
            MoveInstructionType_FREESPACE,
            "TEST_PROFILE",
        )

        program = CompositeInstruction("TEST_PROFILE")
        program.setManipulatorInfo(manip_info)
        program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(start_instruction))
        program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(plan_f1))

        # Interpolate the program first (TrajOpt requires a seed trajectory)
        interpolated_program = generateInterpolatedProgram(program, env, 3.14, 1.0, 3.14, 10)

        # Setup TrajOpt profiles
        plan_profile = TrajOptDefaultPlanProfile()
        composite_profile = TrajOptDefaultCompositeProfile()

        profiles = ProfileDictionary()
        # Use helper functions for cross-module profile registration
        ProfileDictionary_addTrajOptPlanProfile(
            profiles, TRAJOPT_DEFAULT_NAMESPACE, "TEST_PROFILE", plan_profile
        )
        ProfileDictionary_addTrajOptCompositeProfile(
            profiles, TRAJOPT_DEFAULT_NAMESPACE, "TEST_PROFILE", composite_profile
        )

        # Create planner and request
        test_planner = TrajOptMotionPlanner(TRAJOPT_DEFAULT_NAMESPACE)

        request = PlannerRequest()
        request.instructions = interpolated_program
        request.env = env
        request.profiles = profiles

        # Solve
        response = test_planner.solve(request)
        assert response.successful, f"TrajOpt planning failed: {response.message}"

        # Verify results - iterate through instructions
        results = response.results
        count = 0
        for instr in results:
            if instr.isMoveInstruction():
                move_instr = InstructionPoly_as_MoveInstructionPoly(instr)
                wp = move_instr.getWaypoint()
                if wp.isStateWaypoint():
                    state_wp = WaypointPoly_as_StateWaypointPoly(wp)
                    assert len(state_wp.getNames()) == 7  # 7 joints for IIWA
                    assert isinstance(state_wp.getPosition(), np.ndarray)
                    assert len(state_wp.getPosition()) == 7
                count += 1
        assert count > 0, "Expected at least one move instruction in results"

    def test_trajopt_with_collision_config(self, lbr_iiwa_environment):
        """Test TrajOpt planning with collision configuration."""
        env, manip_info, joint_names = lbr_iiwa_environment

        # Simple motion
        wp1 = JointWaypoint(joint_names, np.array([0, 0, 0, -1.0, 0, 0, 0], dtype=np.float64))
        wp2 = JointWaypoint(joint_names, np.array([0.5, 0, 0, -1.0, 0, 0, 0], dtype=np.float64))

        start_instruction = MoveInstruction(
            JointWaypointPoly_wrap_JointWaypoint(wp1),
            MoveInstructionType_FREESPACE,
            "COLLISION_TEST",
        )
        plan_f1 = MoveInstruction(
            JointWaypointPoly_wrap_JointWaypoint(wp2),
            MoveInstructionType_FREESPACE,
            "COLLISION_TEST",
        )

        program = CompositeInstruction("COLLISION_TEST")
        program.setManipulatorInfo(manip_info)
        program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(start_instruction))
        program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(plan_f1))

        interpolated_program = generateInterpolatedProgram(program, env, 3.14, 1.0, 3.14, 10)

        # Setup profiles with collision config
        plan_profile = TrajOptDefaultPlanProfile()
        composite_profile = TrajOptDefaultCompositeProfile()

        # Configure collision cost
        collision_cost = CollisionCostConfig()
        collision_cost.enabled = True
        collision_cost.type = CollisionEvaluatorType.DISCRETE_CONTINUOUS
        collision_cost.safety_margin = 0.025
        collision_cost.coeff = 20.0
        composite_profile.collision_cost_config = collision_cost

        # Configure smoothing
        composite_profile.smooth_velocities = True
        composite_profile.smooth_accelerations = True
        composite_profile.smooth_jerks = False

        profiles = ProfileDictionary()
        ProfileDictionary_addTrajOptPlanProfile(
            profiles, TRAJOPT_DEFAULT_NAMESPACE, "COLLISION_TEST", plan_profile
        )
        ProfileDictionary_addTrajOptCompositeProfile(
            profiles, TRAJOPT_DEFAULT_NAMESPACE, "COLLISION_TEST", composite_profile
        )

        test_planner = TrajOptMotionPlanner(TRAJOPT_DEFAULT_NAMESPACE)

        request = PlannerRequest()
        request.instructions = interpolated_program
        request.env = env
        request.profiles = profiles

        response = test_planner.solve(request)
        assert response.successful, f"TrajOpt planning with collision config failed: {response.message}"
