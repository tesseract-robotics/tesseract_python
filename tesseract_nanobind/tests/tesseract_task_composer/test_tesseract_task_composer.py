"""Tests for tesseract_task_composer bindings."""
import os
import gc
import pytest

import tesseract_robotics
from tesseract_robotics.tesseract_common import FilesystemPath, GeneralResourceLocator
from tesseract_robotics.tesseract_task_composer import TaskComposerPluginFactory


class TestTaskComposerPluginFactory:
    """Test TaskComposerPluginFactory."""

    def test_create_factory_and_nodes(self):
        """Test factory creation and pipeline node creation."""
        # Try env var first (set during test runs), then package config
        config_file = os.environ.get("TESSERACT_TASK_COMPOSER_CONFIG_FILE")
        if not config_file or not os.path.isfile(config_file):
            pkg_config = tesseract_robotics.get_task_composer_config_path()
            if pkg_config and pkg_config.is_file():
                config_file = str(pkg_config)
        assert config_file and os.path.isfile(config_file), "No task composer config found"
        config_path = FilesystemPath(config_file)
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
