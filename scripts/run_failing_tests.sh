#!/bin/bash
# Run the tests that currently fail (TrajOpt planning issues)
# Use this script to track progress on fixing planning bugs

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$SCRIPT_DIR/.."
cd "$PROJECT_ROOT"

source "$SCRIPT_DIR/env.sh"

echo "Running failing TrajOpt planning tests..."
echo ""

# These tests fail because TrajOpt planning returns nullptr/failure
pytest tesseract_nanobind/tests/tesseract_planning/test_planning_api.py::TestPlanningIntegration::test_plan_freespace \
       -v --tb=short "$@"
