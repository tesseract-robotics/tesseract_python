#!/bin/zsh
# Setup conda environment for tesseract_nanobind development
# Based on .github/workflows/wheels.yml dependencies

set -e  # Exit on error

ENV_NAME="tesseract_nb"
PYTHON_VERSION="3.12"

echo "=========================================="
echo "Tesseract Python nanobind Setup"
echo "=========================================="
echo ""

# Check if conda is available
if ! command -v conda &> /dev/null; then
    echo "❌ conda not found. Please install miniconda or anaconda first."
    exit 1
fi

# Check if mamba is available (faster)
if command -v mamba &> /dev/null; then
    CONDA_CMD="mamba"
    echo "✓ Using mamba for faster installation"
else
    CONDA_CMD="conda"
    echo "✓ Using conda (consider installing mamba for faster installs)"
fi

echo ""
echo "Creating conda environment: $ENV_NAME"
echo "Python version: $PYTHON_VERSION"
echo ""

# Create environment if it doesn't exist
if conda env list | grep -q "^$ENV_NAME "; then
    echo "⚠️  Environment '$ENV_NAME' already exists."
    read -q "REPLY?Remove and recreate? (y/n) "
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        conda env remove -n $ENV_NAME -y
    else
        echo "Using existing environment."
        conda activate $ENV_NAME
        exit 0
    fi
fi

# Create environment with Python and base packages
$CONDA_CMD create -n $ENV_NAME python=$PYTHON_VERSION -y

# Activate environment
echo ""
echo "Activating environment..."
eval "$(conda shell.zsh hook)"
conda activate $ENV_NAME

echo ""
echo "=========================================="
echo "Installing Dependencies"
echo "=========================================="

# Install conda-forge packages (covers most C++ dependencies)
echo ""
echo "📦 Installing C++ libraries from conda-forge..."
$CONDA_CMD install -c conda-forge -y \
    cmake \
    ninja \
    eigen \
    boost-cpp \
    console_bridge \
    assimp \
    urdfdom \
    octomap \
    orocos-kdl \
    flann \
    jsoncpp \
    yaml-cpp \
    tinyxml2 \
    pcl \
    bullet \
    fcl \
    ompl \
    ipopt

# Install Python build tools
echo ""
echo "🐍 Installing Python build tools..."
$CONDA_CMD install -c conda-forge -y \
    pip \
    numpy \
    pytest \
    setuptools \
    wheel \
    scikit-build-core \
    nanobind \
    patchelf

# Install colcon and vcstool via pip (not available in conda)
echo ""
echo "🔧 Installing colcon and vcstool via pip..."
pip install \
    colcon-common-extensions \
    vcstool

# Install brew packages that aren't available in conda
echo ""
echo "🍺 Checking brew dependencies..."
if command -v brew &> /dev/null; then
    echo "Installing/updating brew packages..."
    brew install libomp swig automake autoconf libtool || true
else
    echo "⚠️  Homebrew not found. Some packages may be missing:"
    echo "   - libomp (required for OpenMP)"
    echo "   - swig (required for SWIG bindings if building old version)"
    echo ""
    echo "Install brew from: https://brew.sh"
fi

echo ""
echo "=========================================="
echo "Environment Setup Complete!"
echo "=========================================="
echo ""
echo "Environment: $ENV_NAME"
echo "Python: $(python --version)"
echo "CMake: $(cmake --version | head -1)"
echo "nanobind: $(python -c 'import nanobind; print(nanobind.__version__)')"
echo ""
echo "To activate this environment, run:"
echo "  conda activate $ENV_NAME"
echo ""
echo "Next steps:"
echo "  1. Build tesseract C++ libraries:"
echo "     ./build_tesseract_cpp.sh"
echo ""
echo "  2. Build Python bindings:"
echo "     cd .. && pip install -e tesseract_nanobind"
echo ""
echo "  3. Run tests:"
echo "     ./run_tests.sh"
echo ""
