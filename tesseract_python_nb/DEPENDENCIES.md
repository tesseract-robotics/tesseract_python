# Dependencies

## C++ Libraries Required

The tesseract Python bindings require the tesseract C++ libraries to be installed and findable by CMake.

### Install Tesseract C++

From the parent directory (tesseract_python_nanobind):

```bash
# Option 1: Using colcon (ROS workflow)
mkdir -p workspace/src
cd workspace/src
vcs import < ../../dependencies.rosinstall
cd ..
colcon build

# Source the workspace
source install/setup.bash
```

### Required Packages

Based on `dependencies.rosinstall`:
- tesseract 0.28.0
- tesseract_planning 0.28.1
- trajopt 0.28.0
- descartes_light 0.4.5
- opw_kinematics 0.5.2
- ifopt 2.1.3
- ruckig 0.9.2
- ros_industrial_cmake_boilerplate 0.7.1

### System Dependencies

```bash
# macOS
brew install cmake eigen console-bridge

# Ubuntu
sudo apt install cmake libeigen3-dev libconsole-bridge-dev
```

## Build Status

Current error:
```
CMake Error: Could not find a package configuration file provided by
"tesseract_common" with any of the following names:
  tesseract_commonConfig.cmake
  tesseract_common-config.cmake
```

**Solution:** Build tesseract C++ libraries first, then build Python bindings.

## Alternative: Development Mode

For development without full tesseract installation, you could:
1. Mock the tesseract_common library for testing
2. Use FetchContent to download tesseract during build
3. Point CMAKE_PREFIX_PATH to an existing tesseract installation

Example:
```bash
export CMAKE_PREFIX_PATH=/path/to/tesseract/install
pip install -e .
```
