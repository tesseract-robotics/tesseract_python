"""Tests for tesseract_task_composer bindings.

Tests task composer factory creation and pipeline configuration loading.
Note: Pipeline execution tests are in tesseract_planning module which tests
the direct planner APIs that work correctly.
"""
import os
import gc
import pytest

import tesseract_robotics  # triggers env var setup
from tesseract_robotics.tesseract_common import (
    FilesystemPath,
    GeneralResourceLocator,
)
from tesseract_robotics.tesseract_task_composer import TaskComposerPluginFactory


TESSERACT_SUPPORT_DIR = os.environ.get("TESSERACT_SUPPORT_DIR", "")
TESSERACT_TASK_COMPOSER_CONFIG = os.environ.get("TESSERACT_TASK_COMPOSER_CONFIG_FILE", "")


class TestTaskComposerPluginFactory:
    """Test TaskComposerPluginFactory."""

    def test_create_factory_and_nodes(self):
        """Test factory creation and pipeline node creation."""
        if not TESSERACT_TASK_COMPOSER_CONFIG:
            pytest.skip("TESSERACT_TASK_COMPOSER_CONFIG_FILE not set")

        config_path = FilesystemPath(TESSERACT_TASK_COMPOSER_CONFIG)
        locator = GeneralResourceLocator()
        factory = TaskComposerPluginFactory(config_path, locator)
        assert factory is not None

        # Test pipeline node creation - keep references to avoid GC issues
        task1 = factory.createTaskComposerNode("TrajOptPipeline")
        assert task1 is not None
        assert task1.getName() == "TrajOptPipeline"

        task2 = factory.createTaskComposerNode("FreespacePipeline")
        assert task2 is not None
        assert task2.getName() == "FreespacePipeline"

        # Test executor creation
        executor = factory.createTaskComposerExecutor("TaskflowExecutor")
        assert executor is not None

        # Cleanup - delete in reverse order to avoid use-after-free
        del executor
        del task2
        del task1
        del factory
        gc.collect()
