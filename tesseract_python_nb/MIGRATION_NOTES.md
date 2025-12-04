# Migration Notes: SWIG to nanobind

## Cross-Module Inheritance

### ProfileDictionary.addProfile - RESOLVED

The cross-module inheritance for Profile types now works. We use `nb::class_<DerivedProfile, BaseProfile>` in nanobind to register the base class relationship, and nanobind's module import mechanism loads the base type from `tesseract_command_language`.

**Solution:** Use helper functions like `ProfileDictionary_addOMPLProfile()` from the OMPL module:

```python
from tesseract_robotics.tesseract_motion_planners_ompl import ProfileDictionary_addOMPLProfile

plan_profile = OMPLRealVectorPlanProfile()
profiles = ProfileDictionary()
ProfileDictionary_addOMPLProfile(profiles, "OMPLMotionPlannerTask", "DEFAULT", plan_profile)
```

## InstructionsTrajectory and Time Parameterization

### Known Issue: Waypoint Type Mismatch

The `InstructionsTrajectory` class expects `StateWaypointPoly` waypoints but motion planners like OMPL return `JointWaypointPoly`. This causes a type erasure error:
```
RuntimeError: TypeErasureBase, tried to cast 'tesseract_planning::JointWaypointPoly' to 'tesseract_planning::StateWaypointPoly'
```

**Status:** This appears to be a limitation in the underlying C++ API design. The SWIG bindings may have used special handling for this.

**Workaround:** Skip time parameterization when using OMPL directly, or convert JointWaypointPoly to StateWaypointPoly manually before creating InstructionsTrajectory.

## Poly Types (Type Erasure)

The tesseract_command_language uses type-erased "Poly" types (WaypointPoly, InstructionPoly, etc.) for runtime polymorphism. These require careful handling:

- Use helper functions like `WaypointPoly_as_StateWaypointPoly()` to cast between types
- Check waypoint type with `isStateWaypoint()`, `isJointWaypoint()`, etc. before casting
- Type casts will throw RuntimeError if the underlying type doesn't match

### InstructionPoly.as<MoveInstructionPoly>() - RESOLVED

The C++ type erasure `as<T>()` method uses `typeid()` comparison which fails across shared library boundaries. Even when the underlying type IS `MoveInstructionPoly`, the cast fails:

```
RuntimeError: TypeErasureBase, tried to cast 'tesseract_planning::MoveInstructionPoly' to 'tesseract_planning::MoveInstructionPoly'
```

This is an RTTI issue where `typeid()` in Python bindings generates different `type_info` than `typeid()` in the tesseract C++ library.

**Root Cause:**
- `as<T>()` template instantiated in Python binding module creates different `typeid(T)` than the one stored in the type erasure
- BUT: `isMoveInstruction()` works because both `getType()` and `typeid(MoveInstructionPoly)` are generated within the C++ library

**Solution:** Use `getInterface().recover()` to get the underlying `void*` pointer, then cast:

```cpp
m.def("InstructionPoly_as_MoveInstructionPoly", [](tp::InstructionPoly& ip) -> tp::MoveInstructionPoly {
    if (!ip.isMoveInstruction())
        throw std::runtime_error("InstructionPoly is not a MoveInstruction");
    auto* ptr = static_cast<tp::MoveInstructionPoly*>(ip.getInterface().recover());
    return *ptr;  // Copy
}, "instruction"_a);
```

**Working Methods:**
- `InstructionPoly_as_MoveInstructionPoly(instr)` - helper function
- `instr.asMoveInstruction()` - method on InstructionPoly
- `WaypointPoly_as_JointWaypointPoly(wp)` - for JointWaypoint extraction
- `WaypointPoly_as_StateWaypointPoly(wp)` - for StateWaypoint extraction
- `WaypointPoly_as_CartesianWaypointPoly(wp)` - for CartesianWaypoint extraction

## CompositeInstruction.flatten() - Not Needed

The SWIG bindings had a `flatten()` method on `CompositeInstruction`. In nanobind, you can iterate directly over `CompositeInstruction` using `__getitem__` and `len()`.

## Cross-Module Type Resolution

When a method returns a type from another module (e.g., `Environment.getKinematicGroup()` returns `KinematicGroup` from `tesseract_kinematics`), the module containing that type must be imported first.

**Solution:** The `tesseract_environment/__init__.py` imports `tesseract_kinematics` to ensure `KinematicGroup` is registered.

## Viewer Trajectory Visualization - RESOLVED

The `tesseract_robotics_viewer` module now supports:
1. Both `StateWaypointPoly` and `JointWaypointPoly` waypoints (fixed in util.py)
2. Iterating over `CompositeInstruction` using `__getitem__` (SWIG's `flatten()` not needed)

**Status:** Trajectory visualization works with OMPL planning output.

## nanobind Reference Leaks

There are reference leaks warnings at exit. These are likely due to:
- Module-level singleton objects not being properly cleaned up
- Cross-module type references not being released properly

These don't affect functionality but should be investigated and fixed.

## TrajOpt Planner

TrajOpt bindings are optional and auto-detected at build time.

### Usage

```python
from tesseract_robotics.tesseract_motion_planners_trajopt import (
    TrajOptMotionPlanner,
    TrajOptDefaultPlanProfile,
    TrajOptDefaultCompositeProfile,
    CollisionCostConfig,
    CollisionEvaluatorType,
    ProfileDictionary_addTrajOptPlanProfile,
    ProfileDictionary_addTrajOptCompositeProfile,
)

# Create profiles
plan_profile = TrajOptDefaultPlanProfile()
composite_profile = TrajOptDefaultCompositeProfile()
composite_profile.smooth_velocities = True

# Configure collision avoidance
cost_config = CollisionCostConfig()
cost_config.enabled = True
cost_config.type = CollisionEvaluatorType.DISCRETE_CONTINUOUS
cost_config.safety_margin = 0.025
composite_profile.collision_cost_config = cost_config

# Register profiles
profiles = ProfileDictionary()
ProfileDictionary_addTrajOptPlanProfile(profiles, "TrajOptMotionPlannerTask", "DEFAULT", plan_profile)
ProfileDictionary_addTrajOptCompositeProfile(profiles, "TrajOptMotionPlannerTask", "DEFAULT", composite_profile)

# Solve
planner = TrajOptMotionPlanner("TrajOptMotionPlannerTask")
response = planner.solve(request)
```

### Time Parameterization

TrajOpt output uses `StateWaypointPoly`, which is compatible with time parameterization (unlike OMPL which returns `JointWaypointPoly`).
