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

## TaskComposerPluginFactory Cross-Module Issue - RESOLVED

### Problem

`TaskComposerPluginFactory` takes a `ResourceLocator&` in C++, but nanobind cannot cast `GeneralResourceLocator` (from `tesseract_common` module) to `ResourceLocator` across module boundaries. Even with:
- `nb::module_::import_()` to import the base module
- The type displaying as identical in error messages
- Using `const T&` or `shared_ptr<T>` signatures

The cast fails because nanobind maintains separate type registries per module.

### Solution - WORKING

Use `nb::handle` to accept any Python object, manually verify the type using `nb::isinstance()` with the imported type, then use `nb::cast<>()`:

**C++ Binding:**
```cpp
// In tesseract_task_composer_bindings.cpp
// Import the module at module init
nb::module_::import_("tesseract_robotics.tesseract_common._tesseract_common");

// Use nb::handle + isinstance for cross-module type resolution
m.def("createTaskComposerPluginFactory", [](const std::string& config_str, nb::handle locator_handle) {
    tc::fs::path config(config_str);

    // Get the type from the imported module
    auto common_module = nb::module_::import_("tesseract_robotics.tesseract_common._tesseract_common");
    auto grl_type = common_module.attr("GeneralResourceLocator");

    // Verify type
    if (!nb::isinstance(locator_handle, grl_type)) {
        throw nb::type_error("locator must be a GeneralResourceLocator");
    }

    // Cast to C++ type
    auto* locator = nb::cast<tc::GeneralResourceLocator*>(locator_handle);
    return std::make_unique<tp::TaskComposerPluginFactory>(config, *locator);
}, "config"_a, "locator"_a);
```

**Python Usage:**
```python
from tesseract_robotics.tesseract_common import GeneralResourceLocator
from tesseract_robotics.tesseract_task_composer import createTaskComposerPluginFactory

locator = GeneralResourceLocator()
factory = createTaskComposerPluginFactory(task_composer_config_file, locator)
```

### Key Points

1. Accept `nb::handle` (not the concrete type) to avoid nanobind's automatic type checking
2. Import the module containing the type at runtime
3. Use `nb::isinstance(handle, type)` to check the type
4. Use `nb::cast<T*>(handle)` to get the C++ pointer
5. Accept `std::string` for path arguments instead of `tc::fs::path` to avoid additional cross-module issues

### General Pattern for Cross-Module Inheritance

When a function in module B accepts a type from module A:
1. Accept `nb::handle` in the binding
2. Import module A at runtime to get the type object
3. Use `nb::isinstance()` for type checking
4. Use `nb::cast<T*>()` for conversion

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

## C++ Build Issues

### Qt6 Cross-Compile Error - RESOLVED

When building tesseract_task_composer with planning components, CMake may fail with:
```
CMake Error: To use a cross-compiled Qt, please set the QT_HOST_PATH cache variable...
```

This happens because PCL/VTK pulls in Qt6 as a dependency, and the conda Qt6 package is misconfigured on macOS.

**Solution:** Set `QT_HOST_PATH` to point to the conda environment:

```bash
colcon build --merge-install --cmake-args \
    -DTESSERACT_BUILD_TASK_COMPOSER_PLANNING=ON \
    -DQT_HOST_PATH=/opt/miniconda3/envs/tesseract_nb
```

Or add to env.sh:
```bash
export QT_HOST_PATH=$CONDA_PREFIX
```

### Missing libode - RESOLVED

If build fails with `library 'ode' not found`, install the Open Dynamics Engine:

```bash
conda install -c conda-forge libode
```

### Task Composer Planning Component

By default, `TESSERACT_BUILD_TASK_COMPOSER_PLANNING` is OFF. To enable planning pipelines (FreespacePipeline, TrajOptPipeline, etc.):

```bash
colcon build --merge-install --cmake-args \
    -DTESSERACT_BUILD_TASK_COMPOSER_PLANNING=ON \
    -DQT_HOST_PATH=$CONDA_PREFIX
```

This builds `libtesseract_task_composer_planning_factories.dylib` which is required for:
- ProcessPlanningInputTaskFactory
- OMPLPipeline
- TrajOptPipeline
- FreespacePipeline

## Task Composer API Differences

### unique_ptr Returns - No .get() Needed

In nanobind, `createTaskComposerNode()` returns the `TaskComposerNode` directly (not a unique_ptr). Don't call `.get()` on the result:

```python
# nanobind (correct)
task = factory.createTaskComposerNode("TrajOptPipeline")
future = executor.run(task, task_data)

# SWIG (old style - don't use with nanobind)
# future = executor.run(task.get(), task_data)  # Wrong!
```

### Pipeline Input Keys

Different pipelines have different input key names:

```python
# TrajOptPipeline, FreespacePipeline
input_key = task.getInputKeys().get("planning_input")

# OMPLPipeline (direct, without TrajOpt refinement)
input_key = task.getInputKeys().get("program")
```

## TrajOpt Profile Configuration

### TrajOptDefaultPlanProfile

Configure waypoint-level TrajOpt behavior via config objects:

```python
from tesseract_robotics.tesseract_motion_planners_trajopt import (
    TrajOptDefaultPlanProfile,
    TrajOptDefaultCompositeProfile,
    CollisionEvaluatorType,
    ProfileDictionary_addTrajOptPlanProfile,
    ProfileDictionary_addTrajOptCompositeProfile,
)

# Plan profile - waypoint constraints
trajopt_plan_profile = TrajOptDefaultPlanProfile()
trajopt_plan_profile.joint_cost_config.enabled = False
trajopt_plan_profile.cartesian_cost_config.enabled = False
trajopt_plan_profile.cartesian_constraint_config.enabled = True
trajopt_plan_profile.cartesian_constraint_config.coeff = np.array([10.0, 10.0, 10.0, 10.0, 10.0, 0.0])

# Composite profile - collision/smoothing
trajopt_composite_profile = TrajOptDefaultCompositeProfile()
trajopt_composite_profile.collision_constraint_config.enabled = False
trajopt_composite_profile.collision_cost_config.enabled = True
trajopt_composite_profile.collision_cost_config.safety_margin = 0.025
trajopt_composite_profile.collision_cost_config.type = CollisionEvaluatorType.SINGLE_TIMESTEP
trajopt_composite_profile.collision_cost_config.coeff = 20.0

# Register profiles
ProfileDictionary_addTrajOptPlanProfile(profiles, "TrajOptMotionPlannerTask", "CARTESIAN", trajopt_plan_profile)
ProfileDictionary_addTrajOptCompositeProfile(profiles, "TrajOptMotionPlannerTask", "DEFAULT", trajopt_composite_profile)
```

Available config classes:
- `TrajOptCartesianWaypointConfig` - cartesian_cost_config, cartesian_constraint_config
- `TrajOptJointWaypointConfig` - joint_cost_config, joint_constraint_config
- `CollisionCostConfig`, `CollisionConstraintConfig` - collision_cost_config, collision_constraint_config

## Cartesian Path Planning with assignCurrentStateAsSeed

When planning Cartesian paths (CartesianWaypoint), TrajOpt needs seed joint states. Use `assignCurrentStateAsSeed()`:

```python
from tesseract_robotics.tesseract_motion_planners import assignCurrentStateAsSeed

program = CompositeInstruction("DEFAULT")
# ... add CartesianWaypoint instructions ...
assignCurrentStateAsSeed(program, env)
```

## Known Issues

### Reference Leaks at Exit

nanobind reference leak warnings may appear at program exit. These don't affect functionality but indicate cleanup issues with cross-module type references.

### OMPL Constrained Planning API Removed

The `OMPLPlannerConstrainedConfig` class and related constrained planning support has been removed from tesseract's OMPL planner implementation. The header `config/ompl_planner_constrained_config.h` no longer exists in the installed headers.

**Impact:** Examples like `glass_upright_ompl_example.cpp` (which kept a glass upright during motion using `ompl::base::Constraint`) cannot be ported. The C++ test file still exists in the tesseract source but uses deprecated API.

**Current Status:** Constrained OMPL planning (custom constraints on end-effector orientation, etc.) is not available through Python bindings. Use TrajOpt with Cartesian constraints as an alternative for orientation-constrained planning.

**Workaround:** For glass-upright style constraints, use TrajOpt with `cartesian_constraint_config.coeff` to penalize orientation deviations:
```python
trajopt_plan_profile.cartesian_constraint_config.enabled = True
trajopt_plan_profile.cartesian_constraint_config.coeff = np.array([1.0, 1.0, 1.0, 10.0, 10.0, 0.0])  # High weight on rx, ry
```
