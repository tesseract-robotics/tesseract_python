"""
Motion planner profile creation helpers.

These helpers create ProfileDictionary objects with planner profiles
matching C++ example defaults (collision margins, cost coefficients, etc.).
"""
from __future__ import annotations

from typing import List, Optional

from tesseract_robotics.tesseract_command_language import ProfileDictionary


TRAJOPT_DEFAULT_NAMESPACE = "TrajOptMotionPlannerTask"


def create_trajopt_default_profiles(
    profile_names: Optional[List[str]] = None,
) -> ProfileDictionary:
    """Create TrajOpt profiles matching C++ example defaults.

    This creates profiles with collision and cost/constraint settings that
    match the tesseract_examples C++ code (car_seat_example.cpp, etc.).

    Args:
        profile_names: Profile names to register. Default: ["DEFAULT", "FREESPACE"]

    Returns:
        ProfileDictionary with configured TrajOpt profiles
    """
    from tesseract_robotics.tesseract_motion_planners_trajopt import (
        TrajOptDefaultCompositeProfile,
        TrajOptDefaultPlanProfile,
        ProfileDictionary_addTrajOptPlanProfile,
        ProfileDictionary_addTrajOptCompositeProfile,
    )

    if profile_names is None:
        profile_names = ["DEFAULT", "FREESPACE"]

    profiles = ProfileDictionary()

    for name in profile_names:
        # Composite profile: collision settings matching C++ car_seat_example.cpp:323-331
        composite = TrajOptDefaultCompositeProfile()
        composite.collision_constraint_config.enabled = True
        composite.collision_constraint_config.safety_margin = 0.00
        composite.collision_constraint_config.safety_margin_buffer = 0.005
        composite.collision_constraint_config.coeff = 10
        composite.collision_cost_config.safety_margin = 0.005
        composite.collision_cost_config.safety_margin_buffer = 0.01
        composite.collision_cost_config.coeff = 50

        # Plan profile: cost/constraint toggles matching C++ car_seat_example.cpp:333-337
        plan = TrajOptDefaultPlanProfile()
        plan.cartesian_cost_config.enabled = False
        plan.cartesian_constraint_config.enabled = True
        plan.joint_cost_config.enabled = False
        plan.joint_constraint_config.enabled = True

        # Register profiles for this name
        ProfileDictionary_addTrajOptCompositeProfile(
            profiles, TRAJOPT_DEFAULT_NAMESPACE, name, composite
        )
        ProfileDictionary_addTrajOptPlanProfile(
            profiles, TRAJOPT_DEFAULT_NAMESPACE, name, plan
        )

    return profiles


OMPL_DEFAULT_NAMESPACE = "OMPLMotionPlannerTask"


def create_ompl_default_profiles(
    profile_names: Optional[List[str]] = None,
    planning_time: float = 5.0,
    max_solutions: int = 10,
    optimize: bool = True,
    simplify: bool = False,
) -> ProfileDictionary:
    """Create OMPL profiles with sensible defaults.

    Args:
        profile_names: Profile names to register. Default: ["DEFAULT"]
        planning_time: Max planning time in seconds (default: 5.0)
        max_solutions: Max solutions to find before exiting (default: 10)
        optimize: Use all planning time to optimize trajectory (default: True)
        simplify: Simplify trajectory after planning (default: False)

    Returns:
        ProfileDictionary with configured OMPL profiles
    """
    from tesseract_robotics.tesseract_motion_planners_ompl import (
        OMPLRealVectorPlanProfile,
        ProfileDictionary_addOMPLProfile,
    )

    if profile_names is None:
        profile_names = ["DEFAULT"]

    profiles = ProfileDictionary()

    for name in profile_names:
        profile = OMPLRealVectorPlanProfile()

        # Configure solver settings
        profile.solver_config.planning_time = planning_time
        profile.solver_config.max_solutions = max_solutions
        profile.solver_config.optimize = optimize
        profile.solver_config.simplify = simplify

        ProfileDictionary_addOMPLProfile(
            profiles, OMPL_DEFAULT_NAMESPACE, name, profile
        )

    return profiles


DESCARTES_DEFAULT_NAMESPACE = "DescartesMotionPlannerTask"


def create_descartes_default_profiles(
    profile_names: Optional[List[str]] = None,
    enable_collision: bool = True,
    enable_edge_collision: bool = False,
) -> ProfileDictionary:
    """Create Descartes profiles with sensible defaults.

    Args:
        profile_names: Profile names to register. Default: ["DEFAULT"]
        enable_collision: Enable vertex collision checking (default: True)
        enable_edge_collision: Enable edge collision checking (default: False)

    Returns:
        ProfileDictionary with configured Descartes profiles
    """
    from tesseract_robotics.tesseract_motion_planners_descartes import (
        DescartesDefaultPlanProfileD,
        cast_DescartesPlanProfileD,
    )

    if profile_names is None:
        profile_names = ["DEFAULT"]

    profiles = ProfileDictionary()

    for name in profile_names:
        profile = DescartesDefaultPlanProfileD()

        # Configure collision settings
        profile.enable_collision = enable_collision
        profile.enable_edge_collision = enable_edge_collision

        # Cast to base Profile type and add to dictionary
        base_profile = cast_DescartesPlanProfileD(profile)
        profiles.addProfile(DESCARTES_DEFAULT_NAMESPACE, name, base_profile)

    return profiles
