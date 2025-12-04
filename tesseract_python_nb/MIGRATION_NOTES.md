# Migration Notes: SWIG to nanobind

## Cross-Module Inheritance Limitation

nanobind has limitations with cross-module type inheritance. The `Profile` base class is defined in `tesseract_command_language` module, but derived profile classes like `OMPLPlanProfile` are defined in `tesseract_motion_planners_ompl`.

### Known Limitation: ProfileDictionary.addProfile

The `ProfileDictionary.addProfile(ns, profile_name, profile)` method expects a `Profile::ConstPtr` but nanobind cannot automatically convert derived profile types from other modules to the base `Profile` type.

**Workaround options:**

1. **TaskComposer approach**: Use `TaskComposerProblem` and configure profiles through YAML config files or TaskComposer infrastructure instead of manual `ProfileDictionary.addProfile` calls.

2. **Future fix**: Implement proper cross-module type registration using nanobind's `nb::type<>` template with `type_slots` for cross-module inheritance. This requires significant refactoring of the module structure.

### Affected APIs

- `ProfileDictionary.addProfile()` - cannot pass profile objects from other modules
- Motion planner profiles (OMPL, Simple, TrajOpt) cannot be added to ProfileDictionary from Python

### Migration from SWIG

In SWIG bindings, cross-module inheritance worked transparently. In nanobind, you may need to use TaskComposer YAML configuration instead of programmatic profile setup.

Example YAML-based configuration (preferred approach):
```yaml
task_composer:
  profiles:
    OMPLMotionPlannerTask:
      DEFAULT:
        # profile settings
```
