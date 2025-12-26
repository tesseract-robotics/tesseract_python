#!/bin/zsh
# Build tesseract C++ libraries using colcon
# KEEP IN SYNC WITH: .github/workflows/wheels.yml (authoritative reference)

set -e  # Exit on error

# Get project root (parent of scripts/)
SCRIPT_DIR="${0:a:h}"  # zsh way to get script directory
PROJECT_ROOT="$SCRIPT_DIR/.."
cd "$PROJECT_ROOT"

WORKSPACE_DIR="$PROJECT_ROOT/ws"
PYTHON_VERSION=${PYTHON_VERSION:-"3.12"}

echo "=========================================="
echo "Tesseract C++ Build Script"
echo "=========================================="
echo ""

# Check for conda
if ! command -v conda &> /dev/null; then
    echo "❌ conda not found. Please install miniconda or anaconda."
    exit 1
fi

# Auto-activate tesseract_nb environment
ENV_NAME="tesseract_nb"

if [[ "$CONDA_DEFAULT_ENV" != "$ENV_NAME" ]]; then
    echo "Activating conda environment: $ENV_NAME"

    # Check if environment exists
    if ! conda env list | grep -q "^$ENV_NAME "; then
        echo "❌ Environment '$ENV_NAME' not found."
        echo ""
        echo "Please run the setup script first:"
        echo "  ./scripts/setup_conda_env.sh"
        exit 1
    fi

    # Activate the environment
    eval "$(conda shell.zsh hook)"
    conda activate $ENV_NAME

    echo "✓ Activated environment: $ENV_NAME"
else
    echo "✓ Already in environment: $ENV_NAME"
fi

echo "Python: $(which python)"
echo ""

# Check for required tools
if ! command -v colcon &> /dev/null; then
    echo "❌ colcon not found."
    echo ""
    echo "Please run the setup script first:"
    echo "   ./scripts/setup_conda_env.sh"
    echo ""
    echo "Or install manually:"
    echo "   pip install colcon-common-extensions"
    exit 1
fi

if ! command -v vcs &> /dev/null; then
    echo "❌ vcstool not found."
    echo ""
    echo "Please run the setup script first:"
    echo "   ./scripts/setup_conda_env.sh"
    echo ""
    echo "Or install manually:"
    echo "   pip install vcstool"
    exit 1
fi

# Create workspace
echo "Creating workspace at: $WORKSPACE_DIR"
mkdir -p "$WORKSPACE_DIR/src"

# Copy tesseract_python to workspace (optional - ignored by colcon anyway)
if [ -d "$PROJECT_ROOT/tesseract_python" ]; then
    echo "Copying tesseract_python to workspace..."
    if [ -d "$WORKSPACE_DIR/src/tesseract_python" ]; then
        echo "  (already exists, skipping)"
    else
        cp -r "$PROJECT_ROOT/tesseract_python" "$WORKSPACE_DIR/src/"
    fi
fi

# Copy dependencies.rosinstall to workspace
echo "Copying dependencies.rosinstall..."
if [ ! -f "$PROJECT_ROOT/dependencies.rosinstall" ]; then
    echo "❌ dependencies.rosinstall not found at: $PROJECT_ROOT/dependencies.rosinstall"
    exit 1
fi
cp "$PROJECT_ROOT/dependencies.rosinstall" "$WORKSPACE_DIR/src/"

# Import dependencies using vcstool
echo ""
echo "=========================================="
echo "Importing Dependencies (vcstool)"
echo "=========================================="
cd "$WORKSPACE_DIR/src"

echo "Running: vcs import --input dependencies.rosinstall"
vcs import --input dependencies.rosinstall

echo ""
echo "Workspace contents:"
ls -1

# Build with colcon
echo ""
echo "=========================================="
echo "Building with colcon"
echo "=========================================="
cd "$WORKSPACE_DIR"

# Detect architecture for macOS
if [[ "$OSTYPE" == "darwin"* ]]; then
    ARCH=$(uname -m)
    if [[ "$ARCH" == "arm64" ]]; then
        BREW_PREFIX="/opt/homebrew"
        ARCH_NAME="arm64"
    else
        BREW_PREFIX="/usr/local"
        ARCH_NAME="x86_64"
    fi

    echo "Detected macOS $ARCH_NAME"
    echo "Brew prefix: $BREW_PREFIX"
    echo ""

    # Set library paths for macOS
    export DYLD_LIBRARY_PATH=$DYLD_LIBRARY_PATH:$WORKSPACE_DIR/install/lib

    # Configure OpenMP paths - use conda's llvm-openmp (not Homebrew's)
    # This avoids duplicate library crashes when both are loaded at runtime
    if [ -f "$CONDA_PREFIX/lib/libomp.dylib" ]; then
        echo "✓ Found libomp at $CONDA_PREFIX/lib/libomp.dylib (conda)"
        OPENMP_CMAKE_ARGS=(
            -DOpenMP_CXX_INCLUDE_DIR=$CONDA_PREFIX/include
            -DOpenMP_C_INCLUDE_DIR=$CONDA_PREFIX/include
            -DOpenMP_CXX_LIB_NAMES=libomp
            -DOpenMP_CXX_FLAGS="-Xpreprocessor -fopenmp"
            -DOpenMP_C_LIB_NAMES=libomp
            -DOpenMP_C_FLAGS="-Xpreprocessor -fopenmp"
            -DOpenMP_libomp_LIBRARY=$CONDA_PREFIX/lib/libomp.dylib
        )
    else
        echo "⚠️  libomp not found in conda env"
        echo "   Install with: conda install llvm-openmp"
        OPENMP_CMAKE_ARGS=()
    fi
    # Set rpath to @loader_path so libs find deps in same directory
    RPATH_CMAKE_ARGS=(
        -DCMAKE_INSTALL_RPATH=@loader_path
        -DCMAKE_BUILD_WITH_INSTALL_RPATH=ON
    )
else
    OPENMP_CMAKE_ARGS=()
    # Set rpath to $ORIGIN so libs find deps in same directory
    RPATH_CMAKE_ARGS=(
        '-DCMAKE_INSTALL_RPATH=$ORIGIN'
        -DCMAKE_BUILD_WITH_INSTALL_RPATH=ON
    )
fi

# Build command
echo "Building tesseract C++ libraries..."
echo ""

# Set CMAKE_PREFIX_PATH to include conda environment
export CMAKE_PREFIX_PATH="$CONDA_PREFIX:$CMAKE_PREFIX_PATH"
echo "CMAKE_PREFIX_PATH: $CMAKE_PREFIX_PATH"

# Set LIBRARY_PATH for linker to find conda libraries (matches CI)
export LIBRARY_PATH=$CONDA_PREFIX/lib:$LIBRARY_PATH
echo "LIBRARY_PATH: $LIBRARY_PATH"
echo ""

colcon build \
    --merge-install \
    --packages-ignore tesseract_examples tesseract_python vhacd qpoases tesseract_nanobind tesseract_viewer_python \
    --event-handlers console_cohesion+ \
    --cmake-force-configure \
    --cmake-args \
        -GNinja \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_CXX_STANDARD=17 \
        -DBUILD_TESTING=OFF \
        -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
        -DINSTALL_OMPL=OFF \
        -DINSTALL_OMPL_TAG=master \
        -DBUILD_IPOPT=OFF \
        -DBUILD_SNOPT=OFF \
        -DBUILD_SHARED_LIBS=ON \
        -DTESSERACT_ENABLE_EXAMPLES=OFF \
        -DTESSERACT_BUILD_TRAJOPT_IFOPT=ON \
        -DVCPKG_APPLOCAL_DEPS=OFF \
        -DTESSERACT_ENABLE_TESTING=OFF \
        -DCMAKE_OSX_DEPLOYMENT_TARGET=12.0 \
        -DQT_HOST_PATH="$CONDA_PREFIX" \
        "${OPENMP_CMAKE_ARGS[@]}" \
        "${RPATH_CMAKE_ARGS[@]}"

echo ""
echo "=========================================="
echo "Build Complete!"
echo "=========================================="
echo ""

echo "Installation directory: $WORKSPACE_DIR/install"
echo ""
echo "To use these libraries:"
echo "  source $WORKSPACE_DIR/install/setup.bash"
echo "  export CMAKE_PREFIX_PATH=\$CMAKE_PREFIX_PATH:$WORKSPACE_DIR/install"
echo ""
echo "Next step: Build Python bindings"
echo "  cd tesseract_nanobind"
echo "  pip install -e ."
echo ""
