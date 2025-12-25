"""
TrajOpt profile creation helpers.

These helpers create ProfileDictionary objects with TrajOpt profiles
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
