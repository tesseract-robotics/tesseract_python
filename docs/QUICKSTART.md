# Quick Start Guide

## Option 1: Automated Setup (Recommended)

### Step 1: Setup Conda Environment

```bash
cd tesseract_nanobind
./setup_conda_env.sh
```

This will:
- Create `tesseract_nb` conda environment with Python 3.12
- Install all C++ dependencies (eigen, boost, bullet, ompl, etc.)
- Install Python build tools (nanobind, scikit-build-core, pytest)
- Install colcon and vcstool

**macOS users:** Also run `brew install libomp` for OpenMP support.

### Step 2: Activate Environment

```bash
conda activate tesseract_nb
```

### Step 3: Build Tesseract C++ Libraries

```bash
cd ..  # Back to tesseract_python_nanobind
./build_tesseract_cpp.sh
```

This will:
- Create `ws/` workspace
- Import tesseract dependencies via vcstool
- Build with colcon
- Install to `ws/install/`

### Step 4: Build Python Bindings

```bash
cd tesseract_nanobind
pip install -e .
```

### Step 5: Run Tests

```bash
export TESSERACT_SUPPORT_DIR=$(pwd)/../ws/src/tesseract/tesseract_support
export TESSERACT_TASK_COMPOSER_DIR=$(pwd)/../ws/src/tesseract_planning/tesseract_task_composer
pytest tests/
```

---

## Option 2: Manual Setup

### Using environment.yml

```bash
conda env create -f environment.yml
conda activate tesseract_nb

# macOS: Install brew packages
brew install libomp automake autoconf libtool
```

### From Scratch

See [DEPENDENCIES.md](DEPENDENCIES.md) for detailed instructions.

---

## Verify Installation

```python
from tesseract_robotics.tesseract_environment import Environment
from tesseract_robotics.tesseract_common import GeneralResourceLocator

env = Environment()
print("Success!")
```

---

## Troubleshooting

### CMake can't find tesseract_common

**Solution:** Ensure C++ libs are built and CMAKE_PREFIX_PATH is set:
```bash
export CMAKE_PREFIX_PATH=$(pwd)/../ws/install:$CONDA_PREFIX
pip install -e .
```

### ImportError: cannot import _tesseract_common

**Solution:** Rebuild bindings:
```bash
pip install -e . --force-reinstall --no-deps
```

### Tests fail with "TESSERACT_SUPPORT_DIR not found"

**Solution:** Export test data paths:
```bash
export TESSERACT_SUPPORT_DIR=$(pwd)/../ws/src/tesseract/tesseract_support
```

### macOS: Library not loaded errors

**Solution:** Set DYLD_LIBRARY_PATH:
```bash
export DYLD_LIBRARY_PATH=$(pwd)/../ws/install/lib:$DYLD_LIBRARY_PATH
```

---

## Next Steps

- See [MIGRATION_NOTES.md](MIGRATION_NOTES.md) for API differences from SWIG
- Check `examples/` for usage patterns
- Run `pytest tests/ -v` for full test suite
