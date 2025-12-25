#!/usr/bin/env python3
"""
Plot benchmark results from pytest-benchmark JSON output.

Usage:
    pytest tests/benchmarks/ --benchmark-only --benchmark-json=results.json
    python tests/benchmarks/plot_benchmarks.py results.json

Outputs:
    benchmark_scaling.png - CPU scaling chart
    benchmark_examples.png - Example timing comparison
"""
import json
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def load_benchmarks(json_path: Path) -> list:
    """Load benchmark results from JSON file."""
    data = json.loads(json_path.read_text())
    return data.get("benchmarks", [])


def plot_ompl_scaling(benchmarks: list, output_path: Path):
    """Plot OMPL planner scaling (time vs num_planners)."""
    # Filter OMPL scaling tests
    ompl_tests = [b for b in benchmarks if "freespace_ompl_scaling" in b["name"]]
    if not ompl_tests:
        print("No OMPL scaling tests found")
        return

    # Extract data
    num_planners = []
    mean_times = []
    std_times = []

    for b in sorted(ompl_tests, key=lambda x: x["extra_info"].get("num_planners", 0)):
        num_planners.append(b["extra_info"]["num_planners"])
        mean_times.append(b["stats"]["mean"] * 1000)  # Convert to ms
        std_times.append(b["stats"]["stddev"] * 1000)

    # Calculate speedup
    base_time = mean_times[0] if mean_times else 1
    speedups = [base_time / t for t in mean_times]

    # Create figure with two subplots
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

    # Left: Time vs num_planners
    ax1.errorbar(num_planners, mean_times, yerr=std_times, marker="o", capsize=5)
    ax1.set_xlabel("Number of Parallel Planners")
    ax1.set_ylabel("Time (ms)")
    ax1.set_title("OMPL Planning Time vs Parallelism")
    ax1.set_xticks(num_planners)
    ax1.grid(True, alpha=0.3)

    # Right: Speedup
    ax2.bar(range(len(num_planners)), speedups, tick_label=num_planners)
    ax2.axhline(y=1.0, color="r", linestyle="--", alpha=0.5, label="Baseline (1 planner)")
    ax2.set_xlabel("Number of Parallel Planners")
    ax2.set_ylabel("Speedup (T₁ / Tₙ)")
    ax2.set_title("OMPL Speedup vs Parallelism")
    ax2.grid(True, alpha=0.3, axis="y")

    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    print(f"Saved: {output_path}")


def plot_example_comparison(benchmarks: list, output_path: Path):
    """Plot timing comparison across examples."""
    # Filter example tests (not scaling tests)
    example_tests = [b for b in benchmarks if "example_timing" in b["name"]]
    if not example_tests:
        print("No example timing tests found")
        return

    # Extract data
    names = []
    mean_times = []
    std_times = []

    for b in sorted(example_tests, key=lambda x: x["stats"]["mean"]):
        names.append(b["extra_info"]["example"])
        mean_times.append(b["stats"]["mean"])
        std_times.append(b["stats"]["stddev"])

    # Create bar chart
    fig, ax = plt.subplots(figsize=(10, 6))

    bars = ax.barh(names, mean_times, xerr=std_times, capsize=5)

    # Color by planner type (inferred from example name)
    colors = []
    for name in names:
        if "ompl" in name:
            colors.append("#2ecc71")  # Green for OMPL
        elif "cartesian" in name or "upright" in name:
            colors.append("#3498db")  # Blue for TrajOpt
        else:
            colors.append("#e74c3c")  # Red for TrajOpt (complex)

    for bar, color in zip(bars, colors):
        bar.set_color(color)

    ax.set_xlabel("Time (seconds)")
    ax.set_title("Planning Example Benchmark Comparison")
    ax.grid(True, alpha=0.3, axis="x")

    # Add legend
    from matplotlib.patches import Patch
    legend_elements = [
        Patch(facecolor="#2ecc71", label="OMPL (freespace)"),
        Patch(facecolor="#3498db", label="TrajOpt (simple)"),
        Patch(facecolor="#e74c3c", label="TrajOpt (complex)"),
    ]
    ax.legend(handles=legend_elements, loc="lower right")

    plt.tight_layout()
    plt.savefig(output_path, dpi=150)
    print(f"Saved: {output_path}")


def main():
    if len(sys.argv) < 2:
        print("Usage: python plot_benchmarks.py <results.json>")
        sys.exit(1)

    json_path = Path(sys.argv[1])
    if not json_path.exists():
        print(f"File not found: {json_path}")
        sys.exit(1)

    benchmarks = load_benchmarks(json_path)
    print(f"Loaded {len(benchmarks)} benchmark results")

    output_dir = json_path.parent
    plot_ompl_scaling(benchmarks, output_dir / "benchmark_scaling.png")
    plot_example_comparison(benchmarks, output_dir / "benchmark_examples.png")


if __name__ == "__main__":
    main()
