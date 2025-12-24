#!/bin/zsh
# Rebuild C++ libs and test locally
set -e

cd /Users/jelle/Code/CADCAM/tesseract_python_nanobind

echo "=========================================="
echo "Rebuild and Test Script"
echo "=========================================="
echo ""

# Activate conda env
eval "$(conda shell.zsh hook)"
conda activate tesseract_nb

# Step 1: Rebuild C++ libraries
echo "Step 1: Rebuilding C++ libraries..."
./scripts/build_tesseract_cpp.sh

# Step 2: Verify rpath in libraries
echo ""
echo "=========================================="
echo "Step 2: Verifying @loader_path in dylibs"
echo "=========================================="
cd ws/install/lib
echo "Checking tesseract_common library:"
otool -l libtesseract_common.dylib | grep -A 2 LC_RPATH || echo "No LC_RPATH found"
echo ""
echo "Checking tesseract_environment library:"
otool -l libtesseract_environment.dylib | grep -A 2 LC_RPATH || echo "No LC_RPATH found"
cd -

# Step 3: Build dev wheel
echo ""
echo "=========================================="
echo "Step 3: Building dev wheel"
echo "=========================================="
./scripts/build_wheel.sh --dev

# Step 4: Install and test
echo ""
echo "=========================================="
echo "Step 4: Installing wheel and running tests"
echo "=========================================="
pip uninstall -y tesseract-robotics || true
pip install wheelhouse/tesseract_robotics-*.whl

# Run kinematics test
echo ""
echo "Testing kinematics plugin loading..."
python -c "
from tesseract_robotics.tesseract_environment import Environment
from tesseract_robotics.tesseract_common import FilesystemPath
env = Environment()
print('Environment created successfully')
srdf_path = FilesystemPath('tesseract_nanobind/tests/data/abb_irb2400.srdf')
urdf_path = FilesystemPath('tesseract_nanobind/tests/data/abb_irb2400.urdf')
env.init(urdf_path, srdf_path)
print('Environment initialized successfully')
kin_group = env.getKinematicGroup('manipulator')
if kin_group:
    print('✓ Kinematics group loaded successfully')
else:
    print('✗ Failed to load kinematics group')
"

echo ""
echo "=========================================="
echo "All steps complete!"
echo "=========================================="
