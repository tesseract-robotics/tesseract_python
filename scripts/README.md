# Scripts

Development scripts for tesseract_python_nanobind.

## Setup

### `setup_conda_env.sh`
Creates conda environment `tesseract_nb` with all dependencies (cmake, ninja, eigen, boost, ompl, etc).

```bash
./scripts/setup_conda_env.sh
```

### `build_tesseract_cpp.sh`
Builds tesseract C++ libraries using colcon. Run from project root.

```bash
./scripts/build_tesseract_cpp.sh
```

## Development

### `env.sh`
Sets environment variables for development. Source before running Python:

```bash
source scripts/env.sh
```

Sets:
- `DYLD_LIBRARY_PATH` - shared library path (required for plugin loading on macOS)
- `TESSERACT_SUPPORT_DIR` - path to URDF/mesh resources
- `TESSERACT_RESOURCE_PATH` - resource locator base path
- `TESSERACT_TASK_COMPOSER_CONFIG_FILE` - task composer plugin config

### `run_benchmarks.sh`
Runs all planning benchmarks and shows summary report.

```bash
./scripts/run_benchmarks.sh
```

Tests all 5 examples:
- Default pipeline timing (5 tests)
- OMPL CPU scaling with 1/2/4/8 planners (skips if example needs TrajOpt)

### `run_tests.sh`
Runs pytest with correct environment. Preferred way to run tests:

```bash
./scripts/run_tests.sh                    # all tests
./scripts/run_tests.sh -k "freespace"     # filter tests
./scripts/run_tests.sh -x                 # stop on first failure
```

### `run_failing_tests.sh`
Runs specific failing tests for debugging.

## Building Wheels

### `build_wheel.sh`
Builds Python wheels for distribution. Two modes:

#### Dev mode (fast, for local testing)
```bash
./scripts/build_wheel.sh --dev
```
- Builds wheel in `dist/` without delocate
- Fast (~30s) - just runs pip wheel
- **Only works in the conda env where it was built** (bakes in absolute paths)
- Use for quick iteration when testing in the same env

#### Portable mode (slow, for distribution)
```bash
./scripts/build_wheel.sh
```
- Builds fully portable wheel in `wheelhouse/`
- Slow (~5min) - runs delocate to bundle all dylibs
- **Works across any conda env** - all dependencies bundled
- Creates temp conda env clone to avoid polluting your env

#### What portable mode does:
1. Clones `tesseract_nb` to temp env (isolation)
2. Builds wheel with `pip wheel`
3. Runs `delocate-wheel` to bundle linked dylibs and fix rpaths
4. Manually adds plugin factory dylibs (not caught by delocate since they're dlopen'd at runtime):
   - `libtesseract_collision_bullet_factories.dylib`
   - `libtesseract_collision_fcl_factories.dylib`
   - `libtesseract_kinematics_*_factory.dylib`
   - `libtesseract_task_composer_*_factories.dylib`
5. Runs `delocate-path` on plugins to fix their rpaths
6. Repacks wheel
7. Cleans up temp env

#### Why two modes?
Editable installs (`pip install -e .`) and dev wheels bake absolute conda env paths into the nanobind modules via `CMAKE_INSTALL_RPATH`. Example:
```
@rpath/libtesseract_environment.dylib
-> /opt/miniconda3/envs/tesseract_nb/lib/libtesseract_environment.dylib
```

Delocate converts these to `@loader_path` and bundles the dylibs:
```
@loader_path/.dylibs/libtesseract_environment.dylib
```

This makes the wheel self-contained and portable across environments.

#### Install portable wheel:
```bash
# In any conda env:
pip install wheelhouse/tesseract*.whl

# Test (no DYLD_LIBRARY_PATH needed):
python -c "from tesseract_robotics import tesseract_collision; print('OK')"
```
