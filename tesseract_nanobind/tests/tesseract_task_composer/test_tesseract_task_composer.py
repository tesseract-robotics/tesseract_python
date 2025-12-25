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
        from pathlib import Path

        # Try multiple fallbacks to find config
        config_file = None

        # 1. Try env var first (may be set by test runner)
        env_cfg = os.environ.get("TESSERACT_TASK_COMPOSER_CONFIG_FILE")
        if env_cfg and Path(env_cfg).is_file():
            config_file = env_cfg

        # 2. Try package bundled config (installed wheels)
        if not config_file:
            pkg_config = tesseract_robotics.get_task_composer_config_path()
            if pkg_config.is_file():
                config_file = str(pkg_config)

        # 3. Try workspace config (editable install)
        if not config_file:
            # Navigate from test file to repo root
            test_dir = Path(__file__).parent.resolve()
            repo_root = test_dir.parent.parent.parent
            ws_config = repo_root / "ws/src/tesseract_planning/tesseract_task_composer/config/task_composer_plugins_no_trajopt_ifopt.yaml"
            if ws_config.is_file():
                config_file = str(ws_config)

        assert config_file and Path(config_file).is_file(), f"No task composer config found. Tried env var, package config, and workspace"
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
