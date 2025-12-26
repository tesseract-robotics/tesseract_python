#!/bin/bash
# Run planning benchmarks with pytest-benchmark
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$SCRIPT_DIR/.."
cd "$PROJECT_ROOT"

source "$SCRIPT_DIR/env.sh"

JSON_DEFAULTS=$(mktemp /tmp/bench_defaults_XXXXXX.json)
JSON_SCALING=$(mktemp /tmp/bench_scaling_XXXXXX.json)

# Run default pipeline tests (all 5 examples)
echo "=== Default Pipeline Benchmarks ==="
pytest tesseract_nanobind/tests/benchmarks \
    -k "TestExampleBenchmarks" \
    --benchmark-enable \
    --benchmark-min-rounds=2 \
    --benchmark-json="$JSON_DEFAULTS" \
    -v "$@"

# Run scaling tests grouped by num_planners (generates histogram)
echo ""
echo "=== OMPL CPU Scaling Benchmarks ==="
pytest tesseract_nanobind/tests/benchmarks \
    -k "TestOMPLScaling" \
    --benchmark-enable \
    --benchmark-min-rounds=2 \
    --benchmark-json="$JSON_SCALING" \
    --benchmark-group-by=param:num_planners \
    --benchmark-histogram=bench \
    -v "$@"

# Generate summary report
python3 - "$JSON_DEFAULTS" "$JSON_SCALING" << 'PYTHON'
import json
import sys
from collections import defaultdict

defaults = {}
scaling = defaultdict(dict)

# Parse defaults
with open(sys.argv[1]) as f:
    for bench in json.load(f).get("benchmarks", []):
        example = bench.get("params", {}).get("example_name", bench["name"])
        defaults[example] = bench["stats"]["mean"]

# Parse scaling
with open(sys.argv[2]) as f:
    for bench in json.load(f).get("benchmarks", []):
        params = bench.get("params", {})
        example = params.get("example_name", bench["name"])
        if "num_planners" in params:
            scaling[example][params["num_planners"]] = bench["stats"]["mean"]

def fmt(t):
    return f"{t:.2f}s" if t and t < 60 else (f"{t/60:.1f}m" if t else "-")

print()
print("╔══════════════════════════════════════════════════════════════════════════════╗")
print("║                          BENCHMARK RESULTS                                   ║")
print("╠══════════════════════════════════════════════════════════════════════════════╣")

if defaults:
    print()
    print("  Default Pipeline:")
    print("  " + "─" * 45)
    for ex in sorted(defaults.keys()):
        print(f"  {ex:<35} {fmt(defaults[ex]):>8}")

if scaling:
    print()
    print("  OMPL CPU Scaling:")
    print(f"  {'Example':<20} │ {'1 CPU':>8} │ {'2 CPU':>8} │ {'4 CPU':>8} │ {'8 CPU':>8} │ {'Speedup':>8}")
    print("  " + "─" * 20 + "─┼─" + "─" * 8 + "─┼─" + "─" * 8 + "─┼─" + "─" * 8 + "─┼─" + "─" * 8 + "─┼─" + "─" * 8)
    for ex in sorted(scaling.keys()):
        t = scaling[ex]
        t1, t2, t4, t8 = t.get(1, 0), t.get(2, 0), t.get(4, 0), t.get(8, 0)
        best = min(v for v in [t1, t2, t4, t8] if v > 0) if any([t1, t2, t4, t8]) else 0
        speedup = f"{t1/best:.2f}x" if t1 and best else "-"
        print(f"  {ex:<20} │ {fmt(t1):>8} │ {fmt(t2):>8} │ {fmt(t4):>8} │ {fmt(t8):>8} │ {speedup:>8}")
    print()
    print("  Speedup = time(1 CPU) / time(best)")

print()
print("╚══════════════════════════════════════════════════════════════════════════════╝")
print()
print("  Histogram: bench.svg (grouped by num_planners)")
PYTHON

rm "$JSON_DEFAULTS" "$JSON_SCALING"
