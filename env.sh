#!/bin/bash
# Source this file to set up the tesseract_python environment
# Usage: source env.sh

# Get the directory containing this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Conda environment
source /opt/miniconda3/etc/profile.d/conda.sh
conda activate tesseract_nb

# Qt6 cross-compile fix (conda Qt6 needs this on macOS)
export QT_HOST_PATH=$CONDA_PREFIX

# Library paths
export DYLD_LIBRARY_PATH="$SCRIPT_DIR/ws/install/lib:$DYLD_LIBRARY_PATH"

# Tesseract resource paths
export TESSERACT_SUPPORT_DIR="$SCRIPT_DIR/ws/src/tesseract/tesseract_support"
export TESSERACT_RESOURCE_PATH="$SCRIPT_DIR/ws/src/tesseract/"

# Task composer config (required for planning examples)
export TESSERACT_TASK_COMPOSER_CONFIG_FILE="$SCRIPT_DIR/ws/src/tesseract_planning/tesseract_task_composer/config/task_composer_plugins.yaml"

# For headless testing (set to 1 to skip server startup in viewer examples)
export TESSERACT_HEADLESS=1

echo "Environment set up:"
echo "  DYLD_LIBRARY_PATH includes: $SCRIPT_DIR/ws/install/lib"
echo "  TESSERACT_SUPPORT_DIR: $TESSERACT_SUPPORT_DIR"
echo "  TESSERACT_RESOURCE_PATH: $TESSERACT_RESOURCE_PATH"
echo "  TESSERACT_HEADLESS: $TESSERACT_HEADLESS"
echo ""
echo "Run pytest:"
echo "  cd tesseract_python_nb && pytest"
echo ""
echo "Run examples:"
echo "  cd tesseract_viewer_python/examples && python shapes_viewer.py"

