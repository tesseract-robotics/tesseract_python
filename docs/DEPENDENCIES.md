# Dependencies

## C++ Libraries Required

The tesseract Python bindings require the tesseract C++ libraries to be installed.

### Install Tesseract C++

From the parent directory (tesseract_python_nanobind):

```bash
# Using colcon
source env.sh
cd ws && colcon build --merge-install
```

### Required Packages

Based on `dependencies.rosinstall`:
- tesseract 0.28.0
- tesseract_planning 0.28.1
- trajopt 0.28.0 (optional, for TrajOpt planner)
- descartes_light 0.4.5
- opw_kinematics 0.5.2
- ruckig 0.9.2
- ros_industrial_cmake_boilerplate 0.7.1

### System Dependencies

```bash
# macOS (via Homebrew + conda)
brew install libomp
conda install eigen boost bullet fcl ompl console-bridge

# Ubuntu
sudo apt install cmake libeigen3-dev libconsole-bridge-dev \
    libboost-all-dev libbullet-dev libfcl-dev libompl-dev
```

## Python Dependencies

```bash
pip install nanobind scikit-build-core numpy pytest
```

## Environment Setup

```bash
# Activate conda environment
conda activate tesseract_nb

# Set library paths (macOS)
export CMAKE_PREFIX_PATH=$PWD/ws/install:$CONDA_PREFIX
export DYLD_LIBRARY_PATH=$PWD/ws/install/lib:$DYLD_LIBRARY_PATH

# Build Python bindings
pip install -e tesseract_nanobind
```

## Optional: TrajOpt Support

TrajOpt requires OSQP 0.6.x (not 1.0.x). The bundled `trajopt_ext/osqp` builds 0.6.3 automatically if no compatible version is found.

```bash
# If OSQP 1.0.x is installed, remove it
conda remove libosqp osqp osqp-eigen

# Then rebuild with TrajOpt
colcon build --merge-install --cmake-args -DTESSERACT_BUILD_TRAJOPT=ON
```

## Task Composer Planning Component

To enable planning pipelines (FreespacePipeline, TrajOptPipeline, OMPLPipeline):

```bash
# Install ODE (required by Descartes)
conda install -c conda-forge libode

# Build with planning enabled
# Note: LIBRARY_PATH, QT_HOST_PATH and CMAKE_POLICY_VERSION_MINIMUM needed on macOS
LIBRARY_PATH=$CONDA_PREFIX/lib colcon build --merge-install --cmake-args \
    -DTESSERACT_BUILD_TASK_COMPOSER_PLANNING=ON \
    -DQT_HOST_PATH=$CONDA_PREFIX \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5
```

**Required flags on macOS:**
- `LIBRARY_PATH=$CONDA_PREFIX/lib` - linker needs to find libode
- `-DQT_HOST_PATH=$CONDA_PREFIX` - fixes Qt6 cross-compile error from PCL/VTK
- `-DCMAKE_POLICY_VERSION_MINIMUM=3.5` - fixes jsoncpp CMake policy error

Without this, you'll get errors like:
```
Failed to load symbol 'ProcessPlanningInputTaskFactory'
```
