"""
Motion planner profile creation helpers.

These helpers create ProfileDictionary objects with planner profiles
matching C++ example defaults (collision margins, cost coefficients, etc.).

Also provides helpers for:
- OMPL planner configurators (RRTConnect, RRTstar, SBL)
- Time parameterization (TOTG, ISP)
"""
from __future__ import annotations

import os
from typing import List, Optional, Union

from tesseract_robotics.tesseract_command_language import ProfileDictionary


def _get_cpu_count() -> int:
    """Get number of available CPUs for parallel planning."""
    return os.cpu_count() or 1


# Standard profile names used across examples and pipelines
STANDARD_PROFILE_NAMES = ["DEFAULT", "FREESPACE", "CARTESIAN", "RASTER", "UPRIGHT"]

# Planner-specific relevant profile names
# OMPL: only freespace planning, Cartesian/Raster/Upright don't apply
OMPL_PROFILE_NAMES = ["DEFAULT", "FREESPACE"]
# TrajOpt: can optimize any motion type
TRAJOPT_PROFILE_NAMES = STANDARD_PROFILE_NAMES
# Descartes: Cartesian sampling, freespace doesn't apply
DESCARTES_PROFILE_NAMES = ["DEFAULT", "CARTESIAN", "RASTER"]

TRAJOPT_DEFAULT_NAMESPACE = "TrajOptMotionPlannerTask"


def create_trajopt_default_profiles(
    profile_names: Optional[List[str]] = None,
) -> ProfileDictionary:
    """Create TrajOpt profiles matching C++ example defaults.

    This creates profiles with collision and cost/constraint settings that
    match the tesseract_examples C++ code (car_seat_example.cpp, etc.).

    TrajOpt uses Sequential Convex Optimization (SCO) to optimize trajectories
    by minimizing costs (smoothness, collision distance) while satisfying
    constraints (joint limits, collision avoidance, waypoint targets).

    Profile Configuration:
        Composite Profile (applies to entire trajectory):
        - collision_constraint: safety_margin=0.00, buffer=0.005, coeff=10
        - collision_cost: safety_margin=0.005, buffer=0.01, coeff=50

        Plan Profile (applies per waypoint):
        - cartesian_constraint: enabled (enforce Cartesian targets exactly)
        - joint_constraint: enabled (enforce joint targets exactly)
        - cartesian_cost: disabled (no soft Cartesian costs)
        - joint_cost: disabled (no soft joint costs)

    Args:
        profile_names: Profile names to register (default: TRAJOPT_PROFILE_NAMES)
            Standard names: ["DEFAULT", "FREESPACE", "CARTESIAN", "RASTER", "UPRIGHT"]

    Returns:
        ProfileDictionary with configured TrajOpt profiles

    Usage:
        from tesseract_robotics.planning.profiles import (
            create_trajopt_default_profiles
        )

        # Create profiles with standard names
        profiles = create_trajopt_default_profiles()

        # Or specify custom profile names
        profiles = create_trajopt_default_profiles(
            profile_names=["MY_PROFILE"]
        )
    """
    from tesseract_robotics.tesseract_motion_planners_trajopt import (
        TrajOptDefaultCompositeProfile,
        TrajOptDefaultPlanProfile,
        ProfileDictionary_addTrajOptPlanProfile,
        ProfileDictionary_addTrajOptCompositeProfile,
    )

    if profile_names is None:
        profile_names = TRAJOPT_PROFILE_NAMES

    # Create ONE set of profiles
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

    # Register under all requested names
    profiles = ProfileDictionary()
    for name in profile_names:
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

    OMPL (Open Motion Planning Library) provides sampling-based motion planners
    that search the configuration space for collision-free paths. This function
    creates profiles configured with parallel RRTConnect planners.

    Profile Configuration:
        - Parallel RRTConnect planners (one per CPU by default)
        - Planning runs until timeout OR max_solutions found
        - Optimization enabled to improve solution quality
        - Simplification disabled (preserves waypoints)

    Args:
        profile_names: Profile names to register (default: OMPL_PROFILE_NAMES)
            Relevant names: ["DEFAULT", "FREESPACE"]
            (OMPL doesn't apply to Cartesian/Raster/Upright tasks)
        planning_time: Max planning time in seconds (default: 5.0)
            Total time budget for all parallel planners
        max_solutions: Max solutions to find before exiting (default: 10)
            Stops early if this many solutions found
        optimize: Use all planning time to optimize trajectory (default: True)
            If True, continues improving solution until timeout
            If False, stops at first solution
        simplify: Simplify trajectory after planning (default: False)
            If True, removes waypoints (may violate constraints)
            If False, preserves all waypoints
        num_planners: Number of parallel RRTConnect planners (default: CPU count)
            More planners = higher chance of finding solution quickly

    Returns:
        ProfileDictionary with configured OMPL profiles

    Usage:
        from tesseract_robotics.planning.profiles import (
            create_ompl_default_profiles
        )

        # Quick planning with defaults
        profiles = create_ompl_default_profiles()

        # Custom planning time and parallelism
        profiles = create_ompl_default_profiles(
            planning_time=10.0,
            num_planners=8
        )

        # Fast first-solution mode
        profiles = create_ompl_default_profiles(
            planning_time=2.0,
            optimize=False,
            max_solutions=1
        )
    """
    from tesseract_robotics.tesseract_motion_planners_ompl import (
        OMPLRealVectorPlanProfile,
        ProfileDictionary_addOMPLProfile,
        RRTConnectConfigurator,
    )

    if profile_names is None:
        profile_names = OMPL_PROFILE_NAMES

    # Default to CPU count for maximum parallelism
    if num_planners is None:
        num_planners = _get_cpu_count()

    # Create ONE profile with the configured settings
    profile = OMPLRealVectorPlanProfile()
    profile.solver_config.planning_time = planning_time
    profile.solver_config.max_solutions = max_solutions
    profile.solver_config.optimize = optimize
    profile.solver_config.simplify = simplify

    # Add parallel RRTConnect planners (C++ default is 2)
    profile.solver_config.clearPlanners()
    for _ in range(num_planners):
        profile.solver_config.addPlanner(RRTConnectConfigurator())

    # Register under all requested names
    profiles = ProfileDictionary()
    for name in profile_names:
        ProfileDictionary_addOMPLProfile(
            profiles, OMPL_DEFAULT_NAMESPACE, name, profile
        )

    return profiles


def create_ompl_planner_configurators(
    planners: Optional[List[str]] = None,
    num_planners: Optional[int] = None,
    **kwargs
) -> List:
    """Create OMPL planner configurators with custom parameters.

    Supports multiple planner types with their specific parameters:
    - RRTConnect: range (default: 0.0 = auto)
    - RRTstar: range, goal_bias (0.05), delay_collision_checking (True)
    - SBL: range (default: 0.0 = auto)

    Args:
        planners: List of planner names (default: ["RRTConnect"])
            Supported: "RRTConnect", "RRTstar", "SBL"
        num_planners: Number of instances per planner type (default: 1)
            Use multiple instances for parallel planning
        **kwargs: Planner-specific parameters:
            range: Step size for tree extension (0.0 = auto)
            goal_bias: Probability of sampling goal (RRTstar only)
            delay_collision_checking: Delay collision checks (RRTstar only)

    Returns:
        List of configured OMPLPlannerConfigurator instances

    Example:
        # Single RRTConnect with default settings
        planners = create_ompl_planner_configurators()

        # Multiple parallel RRTConnect planners
        planners = create_ompl_planner_configurators(
            planners=["RRTConnect"],
            num_planners=4
        )

        # Mix of planner types with custom parameters
        planners = create_ompl_planner_configurators(
            planners=["RRTConnect", "RRTstar"],
            num_planners=2,
            range=0.1,
            goal_bias=0.1,
            delay_collision_checking=False
        )
    """
    from tesseract_robotics.tesseract_motion_planners_ompl import (
        RRTConnectConfigurator,
        RRTstarConfigurator,
        SBLConfigurator,
    )

    if planners is None:
        planners = ["RRTConnect"]

    if num_planners is None:
        num_planners = 1

    configurators = []

    for planner_name in planners:
        for _ in range(num_planners):
            if planner_name == "RRTConnect":
                cfg = RRTConnectConfigurator()
                if "range" in kwargs:
                    cfg.range = kwargs["range"]
                configurators.append(cfg)

            elif planner_name == "RRTstar":
                cfg = RRTstarConfigurator()
                if "range" in kwargs:
                    cfg.range = kwargs["range"]
                if "goal_bias" in kwargs:
                    cfg.goal_bias = kwargs["goal_bias"]
                if "delay_collision_checking" in kwargs:
                    cfg.delay_collision_checking = kwargs["delay_collision_checking"]
                configurators.append(cfg)

            elif planner_name == "SBL":
                cfg = SBLConfigurator()
                if "range" in kwargs:
                    cfg.range = kwargs["range"]
                configurators.append(cfg)

            else:
                raise ValueError(
                    f"Unknown planner: {planner_name}. "
                    f"Supported: RRTConnect, RRTstar, SBL"
                )

    return configurators


DESCARTES_DEFAULT_NAMESPACE = "DescartesMotionPlannerTask"


def create_descartes_default_profiles(
    profile_names: Optional[List[str]] = None,
    enable_collision: bool = True,
    enable_edge_collision: bool = False,
    num_threads: Optional[int] = None,
) -> ProfileDictionary:
    """Create Descartes profiles with sensible defaults.

    Descartes is a Cartesian path planner that uses a ladder graph approach
    to find joint configurations for Cartesian waypoints. It samples IK solutions
    at each waypoint and finds the minimum-cost path through the graph.

    Creates both plan profiles (waypoint sampling) and solver profiles
    (ladder graph solver configuration).

    Profile Configuration:
        Plan Profile:
        - enable_collision: Check vertex (waypoint) collisions
        - enable_edge_collision: Check edge (transition) collisions

        Solver Profile:
        - num_threads: Parallel graph solver threads (default: all CPUs)

    Args:
        profile_names: Profile names to register (default: DESCARTES_PROFILE_NAMES)
            Relevant names: ["DEFAULT", "CARTESIAN", "RASTER"]
            (Descartes doesn't apply to freespace tasks)
        enable_collision: Enable vertex collision checking (default: True)
            Checks each waypoint configuration for collisions
        enable_edge_collision: Enable edge collision checking (default: False)
            Checks transitions between waypoints for collisions
            Warning: Significantly slower, use only if needed
        num_threads: Number of threads for solver (default: CPU count)
            More threads = faster graph search

    Returns:
        ProfileDictionary with configured Descartes profiles

    Usage:
        from tesseract_robotics.planning.profiles import (
            create_descartes_default_profiles
        )

        # Standard configuration
        profiles = create_descartes_default_profiles()

        # With edge collision checking (slower but safer)
        profiles = create_descartes_default_profiles(
            enable_edge_collision=True
        )

        # Custom thread count
        profiles = create_descartes_default_profiles(
            num_threads=4
        )
    """
    from tesseract_robotics.tesseract_motion_planners_descartes import (
        DescartesDefaultPlanProfileD,
        DescartesLadderGraphSolverProfileD,
        cast_DescartesPlanProfileD,
        cast_DescartesSolverProfileD,
    )

    if profile_names is None:
        profile_names = DESCARTES_PROFILE_NAMES

    if num_threads is None:
        num_threads = _get_cpu_count()

    # Create ONE set of profiles
    plan_profile = DescartesDefaultPlanProfileD()
    plan_profile.enable_collision = enable_collision
    plan_profile.enable_edge_collision = enable_edge_collision
    base_plan = cast_DescartesPlanProfileD(plan_profile)

    solver_profile = DescartesLadderGraphSolverProfileD()
    solver_profile.num_threads = num_threads
    base_solver = cast_DescartesSolverProfileD(solver_profile)

    # Register under all requested names
    profiles = ProfileDictionary()
    for name in profile_names:
        profiles.addProfile(DESCARTES_DEFAULT_NAMESPACE, name, base_plan)
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
        profile_names = TRAJOPT_PROFILE_NAMES

    # Create ONE set of profiles
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

    # Register under all requested names
    for name in profile_names:
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

    The FreespacePipeline is a two-stage approach for collision-free motion:
    1. OMPL finds a feasible collision-free path (fast, approximate)
    2. TrajOpt optimizes the path for smoothness and exact collision margins

    This combines the strengths of sampling-based planning (global search)
    and optimization-based planning (solution quality).

    Pipeline Flow:
        Input: Start/goal joint states
        -> OMPL: Parallel RRTConnect planners find collision-free path
        -> TrajOpt: Optimize path for smoothness + collision margins
        -> Output: Smooth, collision-free trajectory

    Args:
        profile_names: Profile names to register (default: STANDARD_PROFILE_NAMES)
            Standard names: ["DEFAULT", "FREESPACE", "CARTESIAN", "RASTER", "UPRIGHT"]
        num_planners: Number of parallel OMPL planners (default: CPU count)
            More planners = higher success rate for difficult problems
        planning_time: OMPL planning time in seconds (default: 5.0)
            TrajOpt has no timeout (runs until convergence)

    Returns:
        ProfileDictionary with both OMPL and TrajOpt profiles

    Usage:
        from tesseract_robotics.planning.profiles import (
            create_freespace_pipeline_profiles
        )

        # Standard freespace planning
        profiles = create_freespace_pipeline_profiles()

        # Difficult environments: more time + parallelism
        profiles = create_freespace_pipeline_profiles(
            planning_time=10.0,
            num_planners=16
        )
    """
    if profile_names is None:
        profile_names = STANDARD_PROFILE_NAMES

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

    The CartesianPipeline handles motion with Cartesian (tool pose) constraints:
    1. Descartes samples IK solutions and finds minimum-cost joint path
    2. TrajOpt optimizes the path for smoothness and collision margins

    This is ideal for tasks with tool orientation constraints (welding, painting,
    milling, etc.) where the tool must follow a specific Cartesian path.

    Pipeline Flow:
        Input: Cartesian waypoints (tool poses)
        -> Descartes: Sample IK solutions at each waypoint
        -> Descartes: Find minimum-cost path through ladder graph
        -> TrajOpt: Optimize path for smoothness + collision margins
        -> Output: Smooth trajectory following Cartesian path

    Args:
        profile_names: Profile names to register (default: STANDARD_PROFILE_NAMES)
            Standard names: ["DEFAULT", "FREESPACE", "CARTESIAN", "RASTER", "UPRIGHT"]
        num_threads: Number of Descartes solver threads (default: CPU count)
            More threads = faster graph search for long paths

    Returns:
        ProfileDictionary with both Descartes and TrajOpt profiles

    Usage:
        from tesseract_robotics.planning.profiles import (
            create_cartesian_pipeline_profiles
        )

        # Standard Cartesian planning
        profiles = create_cartesian_pipeline_profiles()

        # Custom thread count for large raster paths
        profiles = create_cartesian_pipeline_profiles(
            num_threads=8
        )
    """
    if profile_names is None:
        profile_names = STANDARD_PROFILE_NAMES

    # Start with Descartes profiles
    profiles = create_descartes_default_profiles(
        profile_names=profile_names,
        num_threads=num_threads,
    )

    # Add TrajOpt profiles for optimization step
    _add_trajopt_to_profiles(profiles, profile_names)

    return profiles


# =============================================================================
# Time Parameterization Helpers
# =============================================================================


def create_time_optimal_parameterization(
    path_tolerance: float = 0.1,
    min_angle_change: float = 0.001,
):
    """Create Time-Optimal Trajectory Generation (TOTG) parameterization.

    TOTG finds the time-optimal trajectory that respects velocity, acceleration,
    and jerk limits. It uses a numerical integration approach to find the fastest
    trajectory along a given geometric path.

    Args:
        path_tolerance: Path deviation tolerance in radians (default: 0.1)
            Controls how closely the time-parameterized trajectory follows
            the original path. Smaller values = closer following but slower.
        min_angle_change: Minimum angle change in radians (default: 0.001)
            Used to detect linear segments vs curves. Smaller values = more
            sensitive curve detection.

    Returns:
        TimeOptimalTrajectoryGeneration instance

    Usage:
        from tesseract_robotics.planning.profiles import (
            create_time_optimal_parameterization
        )
        from tesseract_robotics.tesseract_time_parameterization import (
            InstructionsTrajectory
        )

        # Create time parameterization
        totg = create_time_optimal_parameterization(path_tolerance=0.05)

        # Apply to trajectory
        trajectory = InstructionsTrajectory(program)
        success = totg.compute(
            trajectory,
            velocity_limits,
            acceleration_limits,
            jerk_limits
        )

    Reference:
        Kunz & Stilman, "Time-Optimal Trajectory Generation for Path Following
        with Bounded Acceleration and Velocity", RSS 2012
    """
    from tesseract_robotics.tesseract_time_parameterization import (
        TimeOptimalTrajectoryGeneration,
    )

    return TimeOptimalTrajectoryGeneration(
        path_tolerance=path_tolerance,
        min_angle_change=min_angle_change
    )


def create_iterative_spline_parameterization(add_points: bool = True):
    """Create Iterative Spline Parameterization (ISP).

    ISP iteratively fits a spline to the trajectory while respecting velocity,
    acceleration, and jerk limits. Generally faster than TOTG but may produce
    slower trajectories.

    Args:
        add_points: Whether to add intermediate points (default: True)
            If True, adds waypoints between existing ones to better satisfy
            constraints. If False, only uses existing waypoints (faster but
            may violate limits).

    Returns:
        IterativeSplineParameterization instance

    Usage:
        from tesseract_robotics.planning.profiles import (
            create_iterative_spline_parameterization
        )
        from tesseract_robotics.tesseract_time_parameterization import (
            InstructionsTrajectory
        )

        # Create time parameterization
        isp = create_iterative_spline_parameterization(add_points=True)

        # Apply to trajectory
        trajectory = InstructionsTrajectory(program)
        success = isp.compute(
            trajectory,
            velocity_limits,
            acceleration_limits,
            jerk_limits
        )

    Note:
        ISP is typically faster to compute than TOTG, but TOTG produces
        time-optimal trajectories. Use ISP when computation speed matters
        more than trajectory time.
    """
    from tesseract_robotics.tesseract_time_parameterization import (
        IterativeSplineParameterization,
    )

    return IterativeSplineParameterization(add_points=add_points)
