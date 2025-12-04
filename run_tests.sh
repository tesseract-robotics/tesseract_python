#!/bin/bash
# Run all pytests with correct environment
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

source env.sh

export TESSERACT_HEADLESS=1

exec pytest tesseract_python_nb/tests "$@"
