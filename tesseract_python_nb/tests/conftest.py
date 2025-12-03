"""Pytest configuration for tesseract_robotics tests.

Handles cleanup to avoid segfaults from Python's non-deterministic
garbage collection order at interpreter shutdown.
"""

import gc
import pytest


@pytest.fixture(autouse=True)
def cleanup_after_test():
    """Force garbage collection after each test to ensure proper cleanup order."""
    yield
    # Force garbage collection to clean up C++ objects before interpreter shutdown
    gc.collect()
    gc.collect()  # Run twice to handle cyclic references


def pytest_sessionfinish(session, exitstatus):
    """Clean up at end of test session."""
    gc.collect()
    gc.collect()
