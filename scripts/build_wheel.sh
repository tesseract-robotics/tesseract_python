#!/bin/zsh
# Build wheel (macOS)
# Usage:
#   ./build_wheel.sh        # Full portable wheel with delocate (slow, for distribution)
#   ./build_wheel.sh --dev  # Fast dev build, no delocate (only works in current env)
set -e

SCRIPT_DIR="${0:a:h}"
PROJECT_ROOT="$SCRIPT_DIR/.."

DEV_MODE=false
if [[ "$1" == "--dev" ]]; then
    DEV_MODE=true
fi

cd "$PROJECT_ROOT/tesseract_nanobind"

# Set library paths
export DYLD_LIBRARY_PATH="$PROJECT_ROOT/ws/install/lib:$CONDA_PREFIX/lib"
export CMAKE_PREFIX_PATH="$CONDA_PREFIX:$PROJECT_ROOT/ws/install"

if $DEV_MODE; then
    echo "Building dev wheel (no delocate)..."
    rm -rf dist/

    # Fix plugin rpaths on macOS (so they work without DYLD_LIBRARY_PATH)
    if [[ "$OSTYPE" == "darwin"* ]]; then
        echo "Fixing plugin factory rpaths..."
        LIB_DIR="$PROJECT_ROOT/ws/install/lib"
        for factory in "$LIB_DIR"/*_factor*.dylib; do
            if [[ -f "$factory" ]]; then
                if ! otool -l "$factory" 2>/dev/null | grep -q LC_RPATH; then
                    install_name_tool -add_rpath "$LIB_DIR" "$factory" 2>/dev/null || true
                    echo "  Fixed: $(basename $factory)"
                fi
            fi
        done
    fi

    pip install setuptools-scm
    pip wheel . -w dist/ --no-build-isolation
    echo ""
    echo "Dev wheel: dist/"
    echo "Install: pip install dist/tesseract*.whl"
    echo "Note: only works in current conda env"
    exit 0
fi

# Full portable build with temp env
BUILD_ENV="tesseract_wheel_build_$$"
echo "Building portable wheel..."
echo "Using temporary env: $BUILD_ENV"

eval "$(conda shell.zsh hook)"
conda create -n "$BUILD_ENV" --clone tesseract_nb -y
conda activate "$BUILD_ENV"

pip install delocate setuptools-scm

rm -rf dist/ wheelhouse/

echo "Building wheel..."
pip wheel . -w dist/ --no-build-isolation

echo "Running delocate..."
mkdir -p wheelhouse
delocate-wheel -w wheelhouse -v dist/tesseract*.whl

# Add plugin factories (runtime-loaded, not caught by delocate)
echo "Adding plugin factories..."
WHEEL_FILE=$(ls wheelhouse/tesseract*.whl)
WHEEL_DIR=$(mktemp -d)
unzip -q "$WHEEL_FILE" -d "$WHEEL_DIR"

PLUGINS=(
    libtesseract_collision_bullet_factories.dylib
    libtesseract_collision_fcl_factories.dylib
    libtesseract_kinematics_core_factories.dylib
    libtesseract_kinematics_kdl_factories.dylib
    libtesseract_kinematics_opw_factory.dylib
    libtesseract_kinematics_ur_factory.dylib
    libtesseract_task_composer_factories.dylib
    libtesseract_task_composer_planning_factories.dylib
    libtesseract_task_composer_taskflow_factories.dylib
)

for plugin in "${PLUGINS[@]}"; do
    if [[ -f "$PROJECT_ROOT/ws/install/lib/$plugin" ]]; then
        cp "$PROJECT_ROOT/ws/install/lib/$plugin" "$WHEEL_DIR/tesseract_robotics/.dylibs/"
        echo "  Added: $plugin"
    fi
done

echo "Fixing plugin rpaths..."
delocate-path "$WHEEL_DIR/tesseract_robotics/.dylibs" -L "$PROJECT_ROOT/ws/install/lib:$CONDA_PREFIX/lib"

# Patch task_composer_config YAMLs (resolved at runtime by __init__.py)
echo "Patching task composer configs..."
for yaml_file in "$WHEEL_DIR/tesseract_robotics/data/task_composer_config"/*.yaml; do
    if [[ -f "$yaml_file" ]] && grep -q '/usr/local/lib' "$yaml_file"; then
        sed -i '' 's|/usr/local/lib|"@PLUGIN_PATH@"|g' "$yaml_file"
        echo "  Patched: $(basename $yaml_file)"
    fi
done

# Remove search_paths from robot YAMLs (forces use of env vars set by __init__.py)
echo "Removing hardcoded search_paths from robot YAMLs..."
find "$WHEEL_DIR/tesseract_robotics/data/tesseract_support" -name "*.yaml" -type f | while read yaml_file; do
    if grep -q 'search_paths:' "$yaml_file"; then
        # Remove search_paths: line and following lines with - /path
        sed -i '' '/search_paths:/d; /^[[:space:]]*- \/.*$/d' "$yaml_file"
        echo "  Removed search_paths from: $(basename $yaml_file)"
    fi
done

rm "$WHEEL_FILE"
cd "$WHEEL_DIR"
zip -rq "$PROJECT_ROOT/tesseract_nanobind/wheelhouse/$(basename $WHEEL_FILE)" .
cd "$PROJECT_ROOT/tesseract_nanobind"
rm -rf "$WHEEL_DIR"

conda deactivate
conda env remove -n "$BUILD_ENV" -y

echo ""
echo "Portable wheel: wheelhouse/"
echo "Install: pip install wheelhouse/tesseract*.whl"
