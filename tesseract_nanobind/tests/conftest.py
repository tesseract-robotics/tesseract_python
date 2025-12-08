"""Pytest configuration for tesseract_robotics tests.

Handles cleanup to avoid segfaults from Python's non-deterministic
garbage collection order at interpreter shutdown.

Custom markers:
  - viewer: Viewer/visualization examples
  - planning: Motion planning examples (require TESSERACT_TASK_COMPOSER_CONFIG_FILE)
  - basic: Basic examples (collision, kinematics, scene_graph)
"""

import gc
import pytest


def pytest_configure(config):
    """Register custom markers."""
    config.addinivalue_line("markers", "viewer: marks tests as viewer examples")
    config.addinivalue_line("markers", "planning: marks tests as planning examples")
    config.addinivalue_line("markers", "basic: marks tests as basic examples")


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
