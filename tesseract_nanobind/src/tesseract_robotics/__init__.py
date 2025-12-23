from importlib.metadata import version, PackageNotFoundError

try:
    __version__ = version("tesseract-robotics-nanobind")
except PackageNotFoundError:
    __version__ = "0.0.0.dev"  # fallback for editable installs without build

import os
from pathlib import Path


def _configure_environment():
    """Set env vars for bundled data or editable install paths."""
    pkg_dir = Path(__file__).parent.resolve()

    # Try bundled data first (installed wheel)
    data_dir = pkg_dir / "data"
    support_dir = data_dir / "tesseract_support"
    config_file = data_dir / "task_composer_config" / "task_composer_plugins_no_trajopt_ifopt.yaml"

    # Fallback to editable install layout (ws/src/...)
    if not support_dir.is_dir():
        # pkg_dir = .../tesseract_nanobind/src/tesseract_robotics
        # ws_dir = .../ws/src
        ws_src = pkg_dir.parent.parent.parent / "ws" / "src"
        support_dir = ws_src / "tesseract" / "tesseract_support"
        data_dir = ws_src / "tesseract"
        config_file = ws_src / "tesseract_planning" / "tesseract_task_composer" / "config" / "task_composer_plugins_no_trajopt_ifopt.yaml"

    if "TESSERACT_SUPPORT_DIR" not in os.environ and support_dir.is_dir():
        os.environ["TESSERACT_SUPPORT_DIR"] = str(support_dir)

    if "TESSERACT_RESOURCE_PATH" not in os.environ and data_dir.is_dir():
        os.environ["TESSERACT_RESOURCE_PATH"] = str(data_dir)

    if "TESSERACT_TASK_COMPOSER_CONFIG_FILE" not in os.environ and config_file.is_file():
        os.environ["TESSERACT_TASK_COMPOSER_CONFIG_FILE"] = str(config_file)


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
