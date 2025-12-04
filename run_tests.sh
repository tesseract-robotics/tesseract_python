#!/bin/bash
# Run the full tesseract_python test suite
# Usage: ./run_tests.sh [pytest-args]
# Example: ./run_tests.sh -v
# Example: ./run_tests.sh tests/tesseract_collision/
# Example: ./run_tests.sh --lf    # run only last failed tests

set -e

# Get the directory containing this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Setup conda
source /opt/miniconda3/etc/profile.d/conda.sh
conda activate tesseract_nb

# Set environment variables
export DYLD_LIBRARY_PATH="$SCRIPT_DIR/ws/install/lib:$DYLD_LIBRARY_PATH"
export TESSERACT_SUPPORT_DIR="$SCRIPT_DIR/ws/src/tesseract/tesseract_support"
export TESSERACT_RESOURCE_PATH="$SCRIPT_DIR/ws/src/tesseract/"
export TESSERACT_HEADLESS=1
export TESSERACT_TASK_COMPOSER_CONFIG_FILE="$SCRIPT_DIR/ws/src/tesseract_planning/tesseract_task_composer/config/task_composer_plugins.yaml"
export QT_HOST_PATH=$CONDA_PREFIX

echo "============================================"
echo "Running tesseract_python test suite"
echo "============================================"
echo "DYLD_LIBRARY_PATH: $SCRIPT_DIR/ws/install/lib"
echo "TESSERACT_SUPPORT_DIR: $TESSERACT_SUPPORT_DIR"
echo "TESSERACT_RESOURCE_PATH: $TESSERACT_RESOURCE_PATH"
echo "TESSERACT_HEADLESS: $TESSERACT_HEADLESS"
echo "============================================"

cd "$SCRIPT_DIR/tesseract_python_nb"

# Run pytest with any additional arguments passed to the script
pytest -v "$@"
