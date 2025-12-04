#!/bin/bash
# Source this file to set up the tesseract_python environment
# Usage: source env.sh

# Get the directory containing this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Conda environment
source /opt/miniconda3/etc/profile.d/conda.sh
conda activate tesseract_nb

# Library paths
export DYLD_LIBRARY_PATH="$SCRIPT_DIR/ws/install/lib:$DYLD_LIBRARY_PATH"

# Tesseract resource paths
export TESSERACT_SUPPORT_DIR="$SCRIPT_DIR/ws/src/tesseract/tesseract_support"
export TESSERACT_RESOURCE_PATH="$SCRIPT_DIR/ws/src/tesseract/"

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
