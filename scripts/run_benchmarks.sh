#!/bin/bash
# Run planning benchmarks with pytest-benchmark
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$SCRIPT_DIR/.."
cd "$PROJECT_ROOT"

source "$SCRIPT_DIR/env.sh"

# Group by param:num_planners to show CPU scaling impact clearly
exec pytest tesseract_nanobind/tests/benchmarks \
    --benchmark-enable \
    --benchmark-group-by=param:num_planners \
    --benchmark-sort=fullname \
    --benchmark-histogram=bench \
    -v "$@"
