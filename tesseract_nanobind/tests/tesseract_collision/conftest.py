"""Pytest configuration for tesseract_collision tests."""
import gc
import pytest


@pytest.fixture(autouse=True)
def cleanup_collision_objects():
    """Force garbage collection after each test to avoid cleanup order issues.

    The collision module holds shared_ptr references to geometry objects.
    If Python's garbage collector destroys objects in the wrong order,
    it can cause segfaults during cleanup. This fixture ensures proper
    cleanup order.
    """
    yield
    # Force garbage collection after each test
    gc.collect()
