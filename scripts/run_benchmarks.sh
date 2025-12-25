#!/bin/bash
# Run planning benchmarks with pytest-benchmark
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$SCRIPT_DIR/.."
cd "$PROJECT_ROOT"

source "$SCRIPT_DIR/env.sh"

# xdist auto-disables benchmark timing, but tests still run in parallel
exec pytest tesseract_nanobind/tests/benchmarks --benchmark-disable -n auto -v "$@"
