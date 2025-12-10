#!/bin/bash
# Run all pytests with correct environment
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

source env.sh

exec pytest tesseract_nanobind/tests "$@"
