__version__ = "0.6.0"

import os
from pathlib import Path


def _configure_environment():
    """Set env vars using bundled data paths (if not already set)."""
    pkg_dir = Path(__file__).parent.resolve()
    data_dir = pkg_dir / "data"
    support_dir = data_dir / "tesseract_support"
    config_dir = data_dir / "task_composer_config"

    if "TESSERACT_SUPPORT_DIR" not in os.environ and support_dir.is_dir():
        os.environ["TESSERACT_SUPPORT_DIR"] = str(support_dir)

    if "TESSERACT_RESOURCE_PATH" not in os.environ and support_dir.is_dir():
        os.environ["TESSERACT_RESOURCE_PATH"] = str(data_dir)

    if "TESSERACT_TASK_COMPOSER_CONFIG_FILE" not in os.environ:
        cfg = config_dir / "task_composer_plugins_no_trajopt_ifopt.yaml"
        if cfg.is_file():
            os.environ["TESSERACT_TASK_COMPOSER_CONFIG_FILE"] = str(cfg)


def get_data_path() -> Path:
    """Get path to bundled data directory."""
    return Path(__file__).parent / "data"


def get_tesseract_support_path() -> Path:
    """Get path to bundled tesseract_support directory."""
    return Path(__file__).parent / "data" / "tesseract_support"


def get_task_composer_config_path() -> Path:
    """Get path to bundled task composer config file."""
    return Path(__file__).parent / "data" / "task_composer_config" / "task_composer_plugins_no_trajopt_ifopt.yaml"


_configure_environment()
