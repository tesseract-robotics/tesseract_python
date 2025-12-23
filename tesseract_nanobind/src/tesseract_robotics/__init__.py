from importlib.metadata import version, PackageNotFoundError

try:
    __version__ = version("tesseract-robotics-nanobind")
except PackageNotFoundError:
    __version__ = "0.0.0.dev"  # fallback for editable installs without build

import os
from pathlib import Path


def _configure_environment():
    """Set env vars using bundled data paths (if not already set).

    Supports both:
    - Installed package: data in pkg/data/
    - Editable install: data in ws/src/tesseract/
    """
    pkg_dir = Path(__file__).parent.resolve()

    # Try bundled data first (installed package)
    data_dir = pkg_dir / "data"
    support_dir = data_dir / "tesseract_support"
    config_dir = data_dir / "task_composer_config"

    # Fallback to workspace layout (editable install)
    if not support_dir.is_dir():
        # Look for ws/src/tesseract relative to package root
        ws_root = pkg_dir.parent.parent.parent  # tesseract_nanobind/src/tesseract_robotics -> tesseract_nanobind -> root
        ws_support = ws_root / "ws" / "src" / "tesseract" / "tesseract_support"
        ws_config = ws_root / "tesseract_nanobind" / "config"
        ws_planning = ws_root / "ws" / "src" / "tesseract_planning" / "tesseract_task_composer"

        if ws_support.is_dir():
            support_dir = ws_support
            data_dir = ws_root / "ws" / "src" / "tesseract"
        if ws_config.is_dir():
            config_dir = ws_config
        elif ws_planning.is_dir():
            config_dir = ws_planning / "config"

    if "TESSERACT_SUPPORT_DIR" not in os.environ and support_dir.is_dir():
        os.environ["TESSERACT_SUPPORT_DIR"] = str(support_dir)

    if "TESSERACT_RESOURCE_PATH" not in os.environ and support_dir.is_dir():
        os.environ["TESSERACT_RESOURCE_PATH"] = str(data_dir)

    if "TESSERACT_TASK_COMPOSER_CONFIG_FILE" not in os.environ:
        cfg = config_dir / "task_composer_plugins_no_trajopt_ifopt.yaml"
        if not cfg.is_file():
            cfg = config_dir / "task_composer_plugins.yaml"
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
