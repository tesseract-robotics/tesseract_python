#!/bin/bash
# Run examples with correct environment
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

source env.sh

# Default: run all working examples
if [ $# -eq 0 ]; then
    echo "=== tesseract_collision_example ==="
    python examples/tesseract_collision_example.py

    echo ""
    echo "=== tesseract_kinematics_example ==="
    python examples/tesseract_kinematics_example.py

    echo ""
    echo "=== tesseract_planning_example_no_composer (OMPL direct) ==="
    echo "(press Ctrl+C to skip waiting for input)"
    timeout 60 python examples/tesseract_planning_example_no_composer.py || true
else
    # Run specific example
    exec python "examples/$1"
fi
