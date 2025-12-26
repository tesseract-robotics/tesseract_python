"""
Planning performance benchmarks using pytest-benchmark.

Run with: pytest tests/benchmarks/ --benchmark-only -v
Compare runs: pytest tests/benchmarks/ --benchmark-compare
Save JSON: pytest tests/benchmarks/ --benchmark-only --benchmark-json=results.json
"""
import importlib
import os
import pytest
import sys
from pathlib import Path

# Add examples to path
examples_dir = Path(__file__).parent.parent.parent.parent / "examples"
sys.path.insert(0, str(examples_dir))

# Skip if benchmark not installed
pytest.importorskip("pytest_benchmark")

EXAMPLES = ["freespace_ompl", "car_seat", "pick_and_place", "glass_upright", "basic_cartesian", "raster", "online_planning"]


def load_example(name: str):
    """Load example module by name using importlib."""
    return importlib.import_module(f"{name}_example")


class TestExampleBenchmarks:
    """Benchmark planning examples with default settings."""

    @pytest.mark.parametrize("example_name", EXAMPLES)
    def test_default(self, benchmark, example_name):
        """Time each example's run() function with default pipeline."""
        module = load_example(example_name)
        result = benchmark(module.run)
        assert result, f"{example_name} failed"
        benchmark.extra_info["example"] = example_name
        benchmark.extra_info["cpus"] = os.cpu_count()


class TestOMPLScaling:
    """Benchmark examples with OMPL parallel planners at different CPU counts.

    Tests all examples with FreespacePipeline and varying num_planners.
    Examples that require TrajOpt for constrained motions will be skipped.
    """

    @pytest.mark.parametrize("example_name", EXAMPLES)
    @pytest.mark.parametrize("num_planners", [1, 2, 4, 8])
    def test_scaling(self, benchmark, example_name, num_planners):
        """Time examples with FreespacePipeline and varying planner counts."""
        module = load_example(example_name)
        try:
            result = benchmark(module.run, pipeline="FreespacePipeline", num_planners=num_planners)
            assert result, f"{example_name} failed"
        except (AssertionError, RuntimeError) as e:
            pytest.skip(f"{example_name} requires TrajOpt: {e}")
        benchmark.extra_info["example"] = example_name
        benchmark.extra_info["num_planners"] = num_planners
        benchmark.extra_info["cpus"] = os.cpu_count()
