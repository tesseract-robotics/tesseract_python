#!/bin/bash
# Run planning benchmarks with pytest-benchmark
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$SCRIPT_DIR/.."
cd "$PROJECT_ROOT"

source "$SCRIPT_DIR/env.sh"

exec pytest tesseract_nanobind/tests/benchmarks --benchmark-enable -n0 --benchmark-histogram=bench -v "$@"
