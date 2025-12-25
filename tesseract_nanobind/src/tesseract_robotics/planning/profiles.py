"""
Motion planner profile creation helpers.

These helpers create ProfileDictionary objects with planner profiles
matching C++ example defaults (collision margins, cost coefficients, etc.).
"""
from __future__ import annotations

import os
from typing import List, Optional

from tesseract_robotics.tesseract_command_language import ProfileDictionary


def _get_cpu_count() -> int:
    """Get number of available CPUs for parallel planning."""
    return os.cpu_count() or 1


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
    num_planners: Optional[int] = None,
) -> ProfileDictionary:
    """Create OMPL profiles with sensible defaults.

    Args:
        profile_names: Profile names to register. Default: ["DEFAULT"]
        planning_time: Max planning time in seconds (default: 5.0)
        max_solutions: Max solutions to find before exiting (default: 10)
        optimize: Use all planning time to optimize trajectory (default: True)
        simplify: Simplify trajectory after planning (default: False)
        num_planners: Number of parallel RRTConnect planners (default: all CPUs)

    Returns:
        ProfileDictionary with configured OMPL profiles
    """
    from tesseract_robotics.tesseract_motion_planners_ompl import (
        OMPLRealVectorPlanProfile,
        ProfileDictionary_addOMPLProfile,
        RRTConnectConfigurator,
    )

    if profile_names is None:
        profile_names = ["DEFAULT"]

    # Default to CPU count for maximum parallelism
    if num_planners is None:
        num_planners = _get_cpu_count()

    profiles = ProfileDictionary()

    for name in profile_names:
        profile = OMPLRealVectorPlanProfile()

        # Configure solver settings
        profile.solver_config.planning_time = planning_time
        profile.solver_config.max_solutions = max_solutions
        profile.solver_config.optimize = optimize
        profile.solver_config.simplify = simplify

        # Add parallel RRTConnect planners (C++ default is 2)
        profile.solver_config.clearPlanners()
        for _ in range(num_planners):
            profile.solver_config.addPlanner(RRTConnectConfigurator())

        ProfileDictionary_addOMPLProfile(
            profiles, OMPL_DEFAULT_NAMESPACE, name, profile
        )

    return profiles


DESCARTES_DEFAULT_NAMESPACE = "DescartesMotionPlannerTask"


def create_descartes_default_profiles(
    profile_names: Optional[List[str]] = None,
    enable_collision: bool = True,
    enable_edge_collision: bool = False,
    num_threads: Optional[int] = None,
) -> ProfileDictionary:
    """Create Descartes profiles with sensible defaults.

    Creates both plan profiles (waypoint sampling) and solver profiles
    (ladder graph solver configuration).

    Args:
        profile_names: Profile names to register. Default: ["DEFAULT"]
        enable_collision: Enable vertex collision checking (default: True)
        enable_edge_collision: Enable edge collision checking (default: False)
        num_threads: Number of threads for solver (default: all CPUs)

    Returns:
        ProfileDictionary with configured Descartes profiles
    """
    if num_threads is None:
        num_threads = _get_cpu_count()
    from tesseract_robotics.tesseract_motion_planners_descartes import (
        DescartesDefaultPlanProfileD,
        DescartesLadderGraphSolverProfileD,
        cast_DescartesPlanProfileD,
        cast_DescartesSolverProfileD,
    )

    if profile_names is None:
        profile_names = ["DEFAULT"]

    profiles = ProfileDictionary()

    for name in profile_names:
        # Plan profile (waypoint sampling, collision)
        plan_profile = DescartesDefaultPlanProfileD()
        plan_profile.enable_collision = enable_collision
        plan_profile.enable_edge_collision = enable_edge_collision
        base_plan = cast_DescartesPlanProfileD(plan_profile)
        profiles.addProfile(DESCARTES_DEFAULT_NAMESPACE, name, base_plan)

        # Solver profile (ladder graph solver)
        solver_profile = DescartesLadderGraphSolverProfileD()
        solver_profile.num_threads = num_threads
        base_solver = cast_DescartesSolverProfileD(solver_profile)
        profiles.addProfile(DESCARTES_DEFAULT_NAMESPACE, name, base_solver)

    return profiles


# =============================================================================
# Pipeline-aware profile helpers
# =============================================================================
# These create profiles for complete pipelines (which use multiple planners)
# Based on task_composer_plugins.yaml:
#   FreespacePipeline → FreespaceTask → OMPL + TrajOpt
#   CartesianPipeline → CartesianTask → Descartes + TrajOpt
#   TrajOptPipeline → TrajOptTask → TrajOpt only


def _add_trajopt_to_profiles(
    profiles: ProfileDictionary,
    profile_names: Optional[List[str]] = None,
) -> None:
    """Add TrajOpt profiles to existing ProfileDictionary (internal helper)."""
    from tesseract_robotics.tesseract_motion_planners_trajopt import (
        TrajOptDefaultCompositeProfile,
        TrajOptDefaultPlanProfile,
        ProfileDictionary_addTrajOptPlanProfile,
        ProfileDictionary_addTrajOptCompositeProfile,
    )

    if profile_names is None:
        profile_names = ["DEFAULT", "FREESPACE"]

    for name in profile_names:
        composite = TrajOptDefaultCompositeProfile()
        composite.collision_constraint_config.enabled = True
        composite.collision_constraint_config.safety_margin = 0.00
        composite.collision_constraint_config.safety_margin_buffer = 0.005
        composite.collision_constraint_config.coeff = 10
        composite.collision_cost_config.safety_margin = 0.005
        composite.collision_cost_config.safety_margin_buffer = 0.01
        composite.collision_cost_config.coeff = 50

        plan = TrajOptDefaultPlanProfile()
        plan.cartesian_cost_config.enabled = False
        plan.cartesian_constraint_config.enabled = True
        plan.joint_cost_config.enabled = False
        plan.joint_constraint_config.enabled = True

        ProfileDictionary_addTrajOptCompositeProfile(
            profiles, TRAJOPT_DEFAULT_NAMESPACE, name, composite
        )
        ProfileDictionary_addTrajOptPlanProfile(
            profiles, TRAJOPT_DEFAULT_NAMESPACE, name, plan
        )


def create_freespace_pipeline_profiles(
    profile_names: Optional[List[str]] = None,
    num_planners: Optional[int] = None,
    planning_time: float = 5.0,
) -> ProfileDictionary:
    """Create profiles for FreespacePipeline (OMPL + TrajOpt).

    FreespacePipeline uses:
    1. OMPL for global path planning (parallel RRTConnect)
    2. TrajOpt for trajectory optimization/smoothing

    Args:
        profile_names: Profile names to register. Default: ["DEFAULT"]
        num_planners: Number of parallel OMPL planners (default: all CPUs)
        planning_time: OMPL planning time in seconds (default: 5.0)

    Returns:
        ProfileDictionary with both OMPL and TrajOpt profiles
    """
    if profile_names is None:
        profile_names = ["DEFAULT"]

    # Start with OMPL profiles
    profiles = create_ompl_default_profiles(
        profile_names=profile_names,
        num_planners=num_planners,
        planning_time=planning_time,
    )

    # Add TrajOpt profiles for smoothing step
    _add_trajopt_to_profiles(profiles, profile_names)

    return profiles


def create_cartesian_pipeline_profiles(
    profile_names: Optional[List[str]] = None,
    num_threads: Optional[int] = None,
) -> ProfileDictionary:
    """Create profiles for CartesianPipeline (Descartes + TrajOpt).

    CartesianPipeline uses:
    1. Descartes for Cartesian path sampling (ladder graph)
    2. TrajOpt for trajectory optimization

    Args:
        profile_names: Profile names to register. Default: ["DEFAULT"]
        num_threads: Number of Descartes solver threads (default: all CPUs)

    Returns:
        ProfileDictionary with both Descartes and TrajOpt profiles
    """
    if profile_names is None:
        profile_names = ["DEFAULT"]

    # Start with Descartes profiles
    profiles = create_descartes_default_profiles(
        profile_names=profile_names,
        num_threads=num_threads,
    )

    # Add TrajOpt profiles for optimization step
    _add_trajopt_to_profiles(profiles, profile_names)

    return profiles
