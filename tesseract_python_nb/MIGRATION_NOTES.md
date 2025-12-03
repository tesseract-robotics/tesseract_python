# SWIG to nanobind Migration Notes

## Implementation Summary

Successfully created tesseract_common prototype with nanobind bindings.

### Files Created

1. **pyproject.toml** - scikit-build-core config, Python 3.9+, stable ABI
2. **CMakeLists.txt** - reusable `add_tesseract_nanobind_extension()` function
3. **src/tesseract_nb.h** - precompiled header with all nanobind includes
4. **src/tesseract_common/tesseract_common_bindings.cpp** - 250 lines of bindings
5. **Python package files** - __init__.py with exports
6. **Tests** - ported from SWIG version

### Migration Patterns Applied

#### 1. Eigen Types (HUGE WIN)
**SWIG:** 850 lines custom typemaps
**nanobind:** `#include <nanobind/eigen/dense.h>` - automatic!

```cpp
// Just works with NumPy arrays
.def_rw("position", &JointState::position)  // Eigen::VectorXd
```

#### 2. Directors → Trampolines
**SWIG:**
```swig
%feature("director") ResourceLocator;
```

**nanobind:**
```cpp
class PyResourceLocator : public tesseract_common::ResourceLocator {
public:
    NB_TRAMPOLINE(tesseract_common::ResourceLocator, 1);
    std::shared_ptr<Resource> locateResource(const std::string& url) override {
        NB_OVERRIDE_PURE(locateResource, url);
    }
};
```

Applied to:
- ResourceLocator (allow Python subclassing)
- OutputHandler (console_bridge)

#### 3. Smart Pointers
**SWIG:** `%shared_ptr(ClassName)`
**nanobind:** Template parameter

```cpp
nb::class_<Resource, std::shared_ptr<Resource>>(m, "Resource")
```

#### 4. Variant Types (tcp_offset challenge)
ManipulatorInfo::tcp_offset is `std::variant<std::string, Eigen::Isometry3d>`

**Solution:** Python property with dynamic type handling
```cpp
.def_property("tcp_offset",
    [](const ManipulatorInfo& self) -> nb::object {
        if (self.tcp_offset.index() == 0)
            return nb::cast(std::get<std::string>(self.tcp_offset));
        else
            return nb::cast(std::get<Eigen::Isometry3d>(self.tcp_offset));
    },
    [](ManipulatorInfo& self, nb::object value) {
        if (nb::isinstance<nb::str>(value))
            self.tcp_offset = nb::cast<std::string>(value);
        else
            self.tcp_offset = nb::cast<Eigen::Isometry3d>(value);
    })
```

Preserves SWIG API compatibility!

#### 5. STL Containers
**SWIG:** 50+ explicit `%template()` declarations
**nanobind:** Automatic for primitives, explicit for custom types

```cpp
#include <nanobind/stl/vector.h>  // std::vector<std::string> automatic
nb::bind_vector<AlignedVector<Eigen::Isometry3d>>(m, "VectorIsometry3d");
```

#### 6. Bytes/Binary Data
BytesResource uses `nb::bytes` for Python bytes objects:
```cpp
.def(nb::init([](const std::string& url, nb::bytes data) {
    std::vector<uint8_t> vec(data.size());
    std::memcpy(vec.data(), data.c_str(), data.size());
    return std::make_shared<BytesResource>(url, vec);
}))
```

### Classes Implemented

- [x] ResourceLocator (+ trampoline)
- [x] Resource, BytesResource, SimpleLocatedResource
- [x] SimpleResourceLocator, GeneralResourceLocator
- [x] ManipulatorInfo (with variant tcp_offset)
- [x] JointState
- [x] AllowedCollisionMatrix
- [x] CollisionMarginData, CollisionMarginOverrideType (enum)
- [x] KinematicLimits
- [x] PluginInfo
- [x] OutputHandler (+ trampoline)
- [x] Console bridge functions (log, setLogLevel, etc.)
- [x] Eigen types (Isometry3d, Translation3d, Quaterniond, AngleAxisd)

### Code Statistics

| Metric | SWIG | nanobind |
|--------|------|----------|
| Interface lines | 276 | 250 |
| Custom typemaps | 850 | 0 |
| Readability | Low | High |
| Type safety | Medium | High |

### Benefits Observed

1. **Native Eigen support** - massive simplification
2. **Type safety** - C++ compiler catches errors
3. **Better error messages** - clearer than SWIG
4. **Modern C++** - uses C++17 features
5. **Faster compilation** - precompiled headers
6. **Cleaner code** - no macro soup

### Challenges

1. **Variant types** - required custom property implementation
2. **Console bridge** - needed trampoline for virtual methods
3. **Binary data** - nb::bytes slightly different from SWIG buffer protocol
4. **Cross-module inheritance** - nanobind can't inherit from classes in different modules

#### Cross-Module Inheritance Issue

When a class inherits from a base class that's bound in a different module, nanobind throws a critical error at import time. For example, `SimpleMotionPlanner` inherits from `MotionPlanner`, but if they're in different .so files:

```cpp
// This FAILS at runtime:
nb::class_<SimpleMotionPlanner, MotionPlanner>(m, "SimpleMotionPlanner")

// Solution: Don't specify inheritance, manually re-expose base methods:
nb::class_<SimpleMotionPlanner>(m, "SimpleMotionPlanner")
    .def("getName", &SimpleMotionPlanner::getName)
    .def("solve", &SimpleMotionPlanner::solve, "request"_a)
    .def("terminate", &SimpleMotionPlanner::terminate)
    .def("clear", &SimpleMotionPlanner::clear);
```

Also ensure all types used in method signatures are complete (not forward-declared) by including the proper headers:
```cpp
#include <tesseract_motion_planners/core/types.h>  // For PlannerRequest/Response
```

This is a fundamental difference from pybind11 which supports cross-module inheritance via `pybind11::module_::import()`.

All challenges resolved with clean solutions!

## Next Steps

### Current: Refactoring SWIG Examples

Porting examples from original SWIG tesseract_python to nanobind version.

**Build/Run Setup (conda env: tesseract_nb at /opt/miniconda3/envs/tesseract_nb):**

```bash
conda activate tesseract_nb
export CMAKE_PREFIX_PATH="/Users/jelle/Code/CADCAM/tesseract_python_nanobind/ws/install:$CONDA_PREFIX"
export DYLD_LIBRARY_PATH=/Users/jelle/Code/CADCAM/tesseract_python_nanobind/ws/install/lib:$DYLD_LIBRARY_PATH
export TESSERACT_RESOURCE_PATH="/Users/jelle/Code/CADCAM/tesseract_python_nanobind/ws/src/tesseract/"
pip install -e .
```

**Examples Status:**

- [x] tesseract_collision_example.py - working
- [x] tesseract_kinematics_example.py - working
- [ ] tesseract_planning_example_composer.py - needs: tesseract_task_composer
- [ ] tesseract_planning_example_no_composer.py - needs: tesseract_motion_planners_trajopt (blocked, see below)

**Modules Bound:**
- tesseract_command_language (JointWaypoint, CartesianWaypoint, StateWaypoint, *Poly types, MoveInstruction, CompositeInstruction, ProfileDictionary)
- tesseract_motion_planners (PlannerRequest, PlannerResponse)
- tesseract_motion_planners_simple (generateInterpolatedProgram, SimpleMotionPlanner)
- tesseract_motion_planners_ompl (OMPLMotionPlanner, RRTConnectConfigurator, OMPLRealVectorPlanProfile)
- tesseract_time_parameterization (TimeOptimalTrajectoryGeneration, InstructionsTrajectory)
- tesseract_task_composer (TaskComposerKeys, TaskComposerDataStorage, TaskComposerContext, TaskComposerNode, TaskComposerFuture, TaskComposerExecutor, TaskflowTaskComposerExecutor, factory functions)

### TrajOpt Build Issues

Building `tesseract_motion_planners_trajopt` requires rebuilding the workspace due to:

1. **yaml-cpp target mismatch**: tesseract expects `yaml-cpp` target but conda exports `yaml-cpp::yaml-cpp`
2. **Qt6 cross-compilation detection**: VTK/PCL pulls in Qt6 which incorrectly detects cross-compilation
3. **CMake policy warnings**: Old packages need `-DCMAKE_POLICY_VERSION_MINIMUM=3.5`

#### yaml-cpp Target Mismatch Fix (Comprehensive Guide)

**The Problem:**

yaml-cpp has historically exported just `yaml-cpp` as the target name, but conda-forge's yaml-cpp now exports `yaml-cpp::yaml-cpp` (following modern CMake namespace conventions). Tesseract codebase uses the old-style `yaml-cpp` target name throughout.

**Error:**
```text
ld: library 'yaml-cpp' not found
clang++: error: linker command failed with exit code 1 (use -v to see invocation)
```

**Why ALIAS Doesn't Work:**

You might think this would fix it:
```cmake
find_package(yaml-cpp REQUIRED)
if(NOT TARGET yaml-cpp AND TARGET yaml-cpp::yaml-cpp)
  add_library(yaml-cpp ALIAS yaml-cpp::yaml-cpp)
endif()
```

This is in tesseract_srdf/CMakeLists.txt already. **BUT it fails because:**

1. With colcon's `--merge-install`, packages are built in isolated build directories
2. When package B depends on package A, it uses the *installed* CMake config from A
3. The ALIAS is only defined during A's build phase, not exported to the install
4. When B runs `find_package(A)`, the ALIAS doesn't exist
5. B fails with `yaml-cpp` not found

**The Correct Fix:**

Replace `yaml-cpp` with `yaml-cpp::yaml-cpp` **only in `target_link_libraries()` calls**, not in:
- `configure_component()` DEPENDENCIES (expects package name for `find_dependency()`)
- `configure_package()` DEPENDENCIES (same reason)
- `cpack_component()` DEPENDS lists (package names for system packages)

**All Files Requiring Patches:**

tesseract repository:
```
ws/src/tesseract/tesseract_collision/core/CMakeLists.txt    (line 20)
ws/src/tesseract/tesseract_srdf/CMakeLists.txt               (line 63)
ws/src/tesseract/tesseract_kinematics/core/CMakeLists.txt    (line 18)
```

tesseract_planning repository:
```
ws/src/tesseract_planning/tesseract_task_composer/core/CMakeLists.txt      (lines 27, 66)
ws/src/tesseract_planning/tesseract_task_composer/taskflow/CMakeLists.txt  (line 12)
ws/src/tesseract_planning/tesseract_task_composer/planning/CMakeLists.txt  (line 55)
```

**Patch Script:**
```bash
# tesseract repository
cd ws/src/tesseract
sed -i '' 's/^         yaml-cpp)$/         yaml-cpp::yaml-cpp)/' tesseract_collision/core/CMakeLists.txt
sed -i '' 's/^         yaml-cpp)$/         yaml-cpp::yaml-cpp)/' tesseract_srdf/CMakeLists.txt
sed -i '' 's/^         yaml-cpp)$/         yaml-cpp::yaml-cpp)/' tesseract_kinematics/core/CMakeLists.txt

# tesseract_planning repository
cd ../tesseract_planning/tesseract_task_composer
sed -i '' 's/^         yaml-cpp)$/         yaml-cpp::yaml-cpp)/' core/CMakeLists.txt
sed -i '' 's/^         yaml-cpp)$/         yaml-cpp::yaml-cpp)/' taskflow/CMakeLists.txt
sed -i '' 's/    yaml-cpp)$/    yaml-cpp::yaml-cpp)/' planning/CMakeLists.txt
```

**Why This Keeps Recurring:**

The workspace was built incrementally. If you run a clean build or rebuild affected packages, the cached cmake files are used which still reference `yaml-cpp`. To fully fix:

```bash
cd ws
rm -rf build/tesseract_collision build/tesseract_srdf build/tesseract_kinematics build/tesseract_task_composer
rm -rf install/lib/cmake/tesseract_collision install/lib/cmake/tesseract_srdf install/lib/cmake/tesseract_kinematics
colcon build --merge-install ...
```

**Upstream Fix Recommendation:**

The correct upstream fix is to use generator expressions in tesseract's CMakeLists:
```cmake
target_link_libraries(${PROJECT_NAME} PUBLIC
    $<IF:$<TARGET_EXISTS:yaml-cpp::yaml-cpp>,yaml-cpp::yaml-cpp,yaml-cpp>
)
```
This automatically uses `yaml-cpp::yaml-cpp` if available, falling back to `yaml-cpp` for older yaml-cpp versions.

#### Qt6 Cross-Compilation Detection Issue

On macOS with conda-forge's Qt6, the Qt6Config.cmake incorrectly detects "implicit cross-compilation" when the host arch doesn't match the build arch. Error:

```text
CMake Error at .../Qt6/QtPublicDependencyHelpers.cmake:288 (message):
  To use a cross-compiled Qt, please set the QT_HOST_PATH cache variable to
  the location of your host Qt installation.
```

This occurs because:
1. PCL depends on VTK
2. VTK depends on Qt6
3. Qt6 auto-detects cross-compilation via `CMAKE_OSX_ARCHITECTURES` comparison
4. Conda-forge Qt6 was built with different arch detection logic

**Solution:** Set QT_HOST_PATH as cache variable via CMAKE_PROJECT_INCLUDE:

```bash
# Create cmake_args.cmake in ws directory:
cat > ws/cmake_args.cmake << 'EOF'
set(QT_HOST_PATH "$ENV{CONDA_PREFIX}" CACHE PATH "Qt6 host path" FORCE)
EOF

# Add to colcon build:
colcon build --cmake-args "-DCMAKE_PROJECT_INCLUDE=$PWD/cmake_args.cmake" ...
```

#### OSQP 1.0 API Incompatibility

The trajopt repository (trajopt_sco, osqp_eigen in ws/src) uses the old OSQP 0.6.x API, but conda-forge has OSQP 1.0.0 with breaking API changes:

- Old API: `#include <constants.h>`, types `c_int`, `c_float`, `csc`
- New API: `#include <osqp/osqp_api_constants.h>`, different type names

**Options:**
1. Skip OSQP packages and use qpOASES only (add `--packages-skip osqp osqp_eigen`)
2. Port trajopt to OSQP 1.0 API (significant work)
3. Install osqp 0.6.x from source

#### vhacd Compilation Issues (macOS)

The vhacd package in trajopt_ext has macOS compatibility issues:
- Missing `#include <cassert>` for assert macro
- Uses `PTHREAD_MUTEX_RECURSIVE_NP` (Linux-specific, macOS uses `PTHREAD_MUTEX_RECURSIVE`)
- Uses `clCreateCommandQueueWithProperties` (requires OpenCL 2.0)

**Current Status:** TrajOpt planner blocked by OSQP 1.0 API incompatibility. Requires porting trajopt_sco and tesseract_motion_planners/trajopt to OSQP 1.0 API (significant work - type changes from `c_int`/`c_float`/`csc` to `OSQPInt`/`OSQPFloat`/`OSQPCscMatrix`, settings field name changes).

**Working planners:** SimpleMotionPlanner, OMPLMotionPlanner, TimeOptimalTrajectoryGeneration.

**To rebuild workspace WITHOUT trajopt:**

```bash
cd ws

# Create cmake args file for Qt6 workaround
cat > cmake_args.cmake << 'EOF'
set(QT_HOST_PATH "$ENV{CONDA_PREFIX}" CACHE PATH "Qt6 host path" FORCE)
EOF

rm -rf build install log
colcon build --merge-install --packages-skip trajopt trajopt_common trajopt_sco trajopt_ifopt vhacd osqp osqp_eigen --cmake-args \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_STANDARD=17 \
  "-DCMAKE_PROJECT_INCLUDE=/path/to/ws/cmake_args.cmake" \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  -DTESSERACT_ENABLE_TESTING=OFF
```

### Expansion (Remaining Modules)
- [ ] tesseract_motion_planners_trajopt - blocked on OSQP 1.0 API incompatibility
- [x] tesseract_task_composer - core + taskflow (no planning component - requires trajopt)
- ~~tesseract_process_managers~~ - deprecated, replaced by tesseract_task_composer

### Reusable Patterns
Create helper headers:
- `src/common/eigen_bindings.h` - common Eigen type bindings
- `src/common/container_bindings.h` - STL container templates
- `src/common/poly_bindings.h` - type erasure helpers (for command_language)

## Lessons Learned

1. **Start simple** - tesseract_common good choice for prototype
2. **nanobind is MUCH simpler** - especially for Eigen/NumPy
3. **Trampolines work well** - clean replacement for directors
4. **Property wrappers** - good solution for complex types (variants)
5. **Stable ABI** - broader Python version support

## API Compatibility

Tests should pass unchanged:
```python
from tesseract_robotics import tesseract_common

# Same API as SWIG version
info = tesseract_common.ManipulatorInfo()
info.tcp_offset = "tool0"
info.tcp_offset = tesseract_common.Isometry3d()
```

## Performance Expectations

nanobind typically 20-50% faster than SWIG:
- Lower call overhead
- Better memory layout
- Optimized type conversions

Will benchmark once tests pass.

## References

- nanobind docs: https://nanobind.readthedocs.io/
- Template: `../compas_nanobind_package_template`
- SWIG original: `../tesseract_python/swig/tesseract_common_python.i`
