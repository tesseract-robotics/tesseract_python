"""
Planning performance benchmarks using pytest-benchmark.

Run with: pytest tests/benchmarks/ --benchmark-only -v
Compare runs: pytest tests/benchmarks/ --benchmark-compare
Save JSON: pytest tests/benchmarks/ --benchmark-only --benchmark-json=results.json
"""
import os
import pytest
import sys
from pathlib import Path

# Add examples to path
examples_dir = Path(__file__).parent.parent.parent.parent / "examples"
sys.path.insert(0, str(examples_dir))

# Skip if benchmark not installed
pytest.importorskip("pytest_benchmark")


class TestExampleBenchmarks:
    """Benchmark planning examples (default parallelism)."""

    @pytest.mark.parametrize("example_name", [
        "freespace_ompl",
        "car_seat",
        "pick_and_place",
        "glass_upright",
        "basic_cartesian",
    ])
    def test_example_timing(self, benchmark, example_name):
        """Time each example's run() function with default settings."""
        if example_name == "freespace_ompl":
            from freespace_ompl_example import run
        elif example_name == "car_seat":
            from car_seat_example import run
        elif example_name == "pick_and_place":
            from pick_and_place_example import run
        elif example_name == "glass_upright":
            from glass_upright_example import run
        elif example_name == "basic_cartesian":
            from basic_cartesian_example import run

        result = benchmark(run)
        assert result, f"{example_name} failed"
        benchmark.extra_info["example"] = example_name
        benchmark.extra_info["cpus"] = os.cpu_count()


class TestOMPLScaling:
    """Benchmark OMPL planner CPU scaling."""

    @pytest.mark.parametrize("num_planners", [1, 2, 4, 8])
    def test_freespace_ompl_scaling(self, benchmark, num_planners):
        """Time freespace_ompl with varying planner counts.

        Tests how OMPL performance scales with parallel RRTConnect planners.
        """
        from freespace_ompl_example import run

        result = benchmark(run, num_planners=num_planners)
        assert result, "freespace_ompl failed"
        benchmark.extra_info["num_planners"] = num_planners
        benchmark.extra_info["cpus"] = os.cpu_count()
