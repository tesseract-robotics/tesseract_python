#!/bin/bash
# Run the tests that currently fail (TrajOpt planning issues)
# Use this script to track progress on fixing planning bugs

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

source env.sh
export TESSERACT_HEADLESS=1

echo "Running failing TrajOpt planning tests..."
echo ""

# These tests fail because TrajOpt planning returns nullptr/failure
pytest tesseract_nanobind/src/tesseract_robotics/tests/tesseract_planning/test_planning_api.py::TestPlanningIntegration::test_plan_freespace \
       tesseract_nanobind/src/tesseract_robotics/tests/tesseract_task_composer/test_tesseract_task_composer.py::TestTaskComposerTrajOptPipeline::test_trajopt_pipeline \
       -v --tb=short "$@"
