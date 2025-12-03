# Quick Start Guide

## Option 1: Automated Setup (Recommended)

### Step 1: Setup Conda Environment

```bash
cd tesseract_python_nb
./setup_conda_env.sh
```

This will:
- Create `tesseract_nb` conda environment with Python 3.12
- Install all C++ dependencies (eigen, boost, bullet, ompl, etc.)
- Install Python build tools (nanobind, scikit-build-core, pytest)
- Install colcon and vcstool

**macOS users:** Also run `brew install libomp swig` for remaining dependencies.

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
- Build with colcon (takes 20-40 minutes)
- Install to `ws/install/`

### Step 4: Build Python Bindings

```bash
cd tesseract_python_nb
export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:$(pwd)/../ws/install
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
# Create environment
conda env create -f environment.yml
conda activate tesseract_nb

# macOS: Install brew packages
brew install libomp swig automake autoconf libtool

# Build tesseract C++ (follow Step 3 above)
# Build Python bindings (follow Step 4 above)
```

### From Scratch

See [DEPENDENCIES.md](DEPENDENCIES.md) for detailed instructions.

---

## Verify Installation

```python
python -c "from tesseract_robotics import tesseract_common; print('Success!')"
```

---

## Troubleshooting

### CMake can't find tesseract_common

**Solution:** Set CMAKE_PREFIX_PATH:
```bash
export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:$(pwd)/../ws/install
```

Or source the colcon workspace:
```bash
source ../ws/install/setup.bash
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
export TESSERACT_TASK_COMPOSER_DIR=$(pwd)/../ws/src/tesseract_planning/tesseract_task_composer
```

---

## Next Steps

Once prototype works:
1. Expand to tesseract_geometry module
2. Continue through remaining 15 modules
3. Create wheels for distribution

See [MIGRATION_NOTES.md](MIGRATION_NOTES.md) for migration patterns.
