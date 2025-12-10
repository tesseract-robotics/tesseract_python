"""CLI selftest runner for tesseract_robotics.

Runs tests module-by-module and generates a report with GitHub issue template.

Usage:
    tesseract_nanobind_selftest           # normal mode
    tesseract_nanobind_selftest -v        # verbose - show full pytest output
    tesseract_nanobind_selftest --verbose # same as -v
"""
import argparse
import io
import os
import re
import sys
import platform
import time
from contextlib import redirect_stdout, redirect_stderr
from datetime import datetime
from pathlib import Path


def get_test_modules(test_dir: Path) -> list[Path]:
    """Get all test module directories."""
    modules = []
    for p in sorted(test_dir.iterdir()):
        if p.is_dir() and p.name.startswith("tesseract_"):
            modules.append(p)
    examples = test_dir / "examples"
    if examples.is_dir():
        modules.append(examples)
    return modules


def parse_failures(output: str) -> list[tuple[str, str]]:
    """Extract failed test names and error messages from pytest output."""
    failures = []
    # match FAILED lines like: FAILED test_foo.py::test_bar - ErrorType: message
    pattern = r"FAILED\s+([^\s]+)\s+-\s+(.+)"
    for match in re.finditer(pattern, output):
        test_id = match.group(1)
        # shorten test_id to just file::test
        if "::" in test_id:
            parts = test_id.split("::")
            test_id = f"{Path(parts[0]).name}::{parts[-1]}"
        error_msg = match.group(2)[:80]  # truncate long errors
        failures.append((test_id, error_msg))

    # also catch ERROR during collection
    error_pattern = r"ERROR\s+([^\s]+)"
    for match in re.finditer(error_pattern, output):
        test_id = match.group(1)
        if "::" in test_id:
            parts = test_id.split("::")
            test_id = f"{Path(parts[0]).name}::{parts[-1]}"
        else:
            test_id = Path(test_id).name
        failures.append((test_id, "collection error"))

    return failures


def parse_counts(output: str) -> tuple[int, int, int]:
    """Parse passed/failed/error counts from pytest output."""
    # patterns like "3 passed", "1 failed", "2 errors"
    passed = failed = errors = 0
    m = re.search(r"(\d+) passed", output)
    if m:
        passed = int(m.group(1))
    m = re.search(r"(\d+) failed", output)
    if m:
        failed = int(m.group(1))
    m = re.search(r"(\d+) error", output)
    if m:
        errors = int(m.group(1))
    return passed, failed, errors


def get_numpy_version() -> str:
    """Get numpy version if installed."""
    try:
        import numpy
        return numpy.__version__
    except ImportError:
        return "not installed"


def get_env_info() -> dict:
    """Collect environment information for verbose output."""
    env_vars = [
        "TESSERACT_SUPPORT_DIR",
        "TESSERACT_RESOURCE_PATH",
        "TESSERACT_TASK_COMPOSER_CONFIG_FILE",
        "DYLD_LIBRARY_PATH",
        "LD_LIBRARY_PATH",
    ]
    return {k: os.environ.get(k, "not set") for k in env_vars}


def main():
    parser = argparse.ArgumentParser(
        description="Run tesseract_robotics self-tests and generate a report."
    )
    parser.add_argument(
        "-v", "--verbose",
        action="store_true",
        help="Show detailed pytest output for each module"
    )
    args = parser.parse_args()
    verbose = args.verbose

    import pytest
    import tesseract_robotics

    test_dir = Path(__file__).parent / "tests"
    modules = get_test_modules(test_dir)

    report_path = Path.cwd() / "tesseract_selftest_report.txt"

    results = []  # list of (module_name, status, passed, failed, duration, failures, output)
    overall_exit = 0

    version = tesseract_robotics.__version__
    py_version = f"{sys.version_info.major}.{sys.version_info.minor}.{sys.version_info.micro}"
    plat = platform.platform()
    numpy_ver = get_numpy_version()
    env_info = get_env_info()

    print(f"tesseract_robotics selftest v{version}")
    print(f"Python {py_version} | NumPy {numpy_ver}")
    print(f"Platform: {plat}")

    if verbose:
        print("\nEnvironment:")
        for k, v in env_info.items():
            # truncate long paths
            if len(v) > 60:
                v = "..." + v[-57:]
            print(f"  {k}: {v}")

    print(f"\nRunning {len(modules)} test modules...\n")

    for module in modules:
        module_name = module.name

        if verbose:
            print(f"{'=' * 60}")
            print(f"MODULE: {module_name}")
            print(f"{'=' * 60}")
            # run pytest with full output
            start = time.time()
            exit_code = pytest.main([str(module), "-v", "--tb=short"])
            duration = time.time() - start
            output = ""  # not captured in verbose mode
        else:
            print(f"  [{module_name}] ", end="", flush=True)
            # capture pytest output
            stdout_capture = io.StringIO()
            stderr_capture = io.StringIO()

            start = time.time()
            with redirect_stdout(stdout_capture), redirect_stderr(stderr_capture):
                exit_code = pytest.main([str(module), "-q", "--tb=line"])
            duration = time.time() - start

            output = stdout_capture.getvalue() + stderr_capture.getvalue()

        passed, failed, errors = parse_counts(output) if output else (0, 0, 0)
        failures = parse_failures(output) if exit_code != 0 and output else []

        if exit_code == 0:
            status = "PASS"
            if not verbose:
                print(f"PASS ({passed} tests, {duration:.1f}s)")
        elif exit_code == 5:
            status = "SKIP"
            if not verbose:
                print("SKIP (no tests)")
        else:
            status = "FAIL"
            overall_exit = 1
            if not verbose:
                total = passed + failed + errors
                print(f"FAIL ({failed + errors}/{total} failed, {duration:.1f}s)")

        results.append((module_name, status, passed, failed + errors, duration, failures, output))

    # build report
    total_passed = sum(r[2] for r in results)
    total_failed = sum(r[3] for r in results)
    modules_passed = sum(1 for r in results if r[1] == "PASS")
    modules_failed = sum(1 for r in results if r[1] == "FAIL")
    modules_skipped = sum(1 for r in results if r[1] == "SKIP")

    lines = [
        "=" * 60,
        "TESSERACT ROBOTICS SELFTEST REPORT",
        "=" * 60,
        f"Date: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
        f"Version: {version}",
        f"Python: {py_version}",
        f"Platform: {plat}",
        f"NumPy: {numpy_ver}",
        "",
        "ENVIRONMENT",
        "-" * 40,
    ]
    for k, v in env_info.items():
        lines.append(f"  {k}: {v}")

    lines.extend([
        "",
        "RESULTS",
        "-" * 40,
    ])

    all_failures = []
    for module_name, status, passed, failed, duration, failures, output in results:
        if status == "PASS":
            lines.append(f"  {module_name}: PASS ({passed} tests, {duration:.1f}s)")
        elif status == "SKIP":
            lines.append(f"  {module_name}: SKIP (no tests)")
        else:
            lines.append(f"  {module_name}: FAIL ({failed}/{passed + failed} failed)")
            for test_id, error_msg in failures:
                lines.append(f"    - {test_id}: {error_msg}")
                all_failures.append((module_name, test_id, error_msg))

    lines.extend([
        "-" * 40,
        f"Modules: {modules_passed} passed, {modules_failed} failed, {modules_skipped} skipped",
        f"Tests: {total_passed} passed, {total_failed} failed",
        "=" * 60,
    ])

    # GitHub issue template (only if failures)
    if all_failures:
        failed_modules_summary = []
        for module_name, status, passed, failed, _, _, _ in results:
            if status == "FAIL":
                failed_modules_summary.append(f"- {module_name} ({failed}/{passed + failed} failed)")

        error_details = []
        for module_name, test_id, error_msg in all_failures[:10]:  # limit to 10
            error_details.append(f"{test_id}\n  {error_msg}")

        lines.extend([
            "",
            "=" * 60,
            "GITHUB ISSUE TEMPLATE (copy below this line)",
            "=" * 60,
            "",
            "## Selftest Failure Report",
            "",
            "**Environment:**",
            f"- tesseract_robotics version: {version}",
            f"- Python: {py_version}",
            f"- Platform: {plat}",
            f"- NumPy: {numpy_ver}",
            "",
            "**Failed modules:**",
            *failed_modules_summary,
            "",
            "**Error details:**",
            "```",
            *error_details,
            "```",
            "",
            "**Steps to reproduce:**",
            "```bash",
            "pip install tesseract-robotics-nanobind",
            "tesseract_nanobind_selftest",
            "```",
        ])

    report_content = "\n".join(lines)
    report_path.write_text(report_content)

    print(f"\nReport saved to: {report_path}")
    print(f"\nSummary: {modules_passed} modules passed, {modules_failed} failed, {modules_skipped} skipped")
    if all_failures:
        print("         (report contains GitHub issue template)")

    sys.exit(overall_exit)


if __name__ == "__main__":
    main()
