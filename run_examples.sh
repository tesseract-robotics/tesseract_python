#!/bin/bash
# Run examples with correct environment
# Usage: ./run_examples.sh [example_name...]
# Without args: runs all examples

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

source env.sh
export TESSERACT_HEADLESS=1

EXAMPLES_DIR="$SCRIPT_DIR/examples"
TIMEOUT_CMD=$(command -v gtimeout 2>/dev/null || command -v timeout 2>/dev/null || echo "")
TIMEOUT_SECS=120
PASSED=0
FAILED=0
FAILED_LIST=()

run_example() {
    local example="$1"
    local name=$(basename "$example" .py)
    printf "%-45s " "$name"

    local cmd="python $example"
    [ -n "$TIMEOUT_CMD" ] && cmd="$TIMEOUT_CMD $TIMEOUT_SECS $cmd"
    if eval "$cmd" > /tmp/example_out_$$.txt 2>&1; then
        echo "✓ PASS"
        ((PASSED++))
    else
        echo "✗ FAIL"
        ((FAILED++))
        FAILED_LIST+=("$name")
    fi
}

if [ $# -gt 0 ]; then
    # Run specific examples
    for arg in "$@"; do
        example="$EXAMPLES_DIR/${arg%.py}.py"
        if [ -f "$example" ]; then
            run_example "$example"
        else
            echo "Example not found: $arg"
            ((FAILED++))
        fi
    done
else
    # Run all examples (skip __init__.py)
    for example in "$EXAMPLES_DIR"/*.py; do
        [ "$(basename "$example")" = "__init__.py" ] && continue
        run_example "$example"
    done
fi

echo ""
echo "Results: $PASSED passed, $FAILED failed"
if [ ${#FAILED_LIST[@]} -gt 0 ]; then
    echo "Failed: ${FAILED_LIST[*]}"
    exit 1
fi
