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

## CompositeInstruction.flatten() - Not Available

The SWIG bindings had a `flatten()` method on `CompositeInstruction` that is not available in nanobind. This was a SWIG-specific convenience method. The viewer's trajectory visualization relies on this method.

**Status:** Trajectory visualization not yet working with nanobind bindings.

## Viewer Trajectory Visualization

The `tesseract_robotics_viewer` module expects:
1. `StateWaypointPoly` waypoints (not `JointWaypointPoly`)
2. `CompositeInstruction.flatten()` method (SWIG convenience)

Neither is available with the current nanobind bindings when using OMPL planning.

## nanobind Reference Leaks

There are reference leaks warnings at exit. These are likely due to:
- Module-level singleton objects not being properly cleaned up
- Cross-module type references not being released properly

These don't affect functionality but should be investigated and fixed.
