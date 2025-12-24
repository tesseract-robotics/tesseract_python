from importlib.metadata import version, PackageNotFoundError
import json
import os
from pathlib import Path

try:
    __version__ = version("tesseract-robotics-nanobind")
except PackageNotFoundError:
    __version__ = "0.0.0.dev"  # fallback for editable installs without build


def _is_editable_install() -> bool:
    """Check if package was installed in editable mode (pip install -e .)."""
    try:
        from importlib.metadata import distribution
        dist = distribution("tesseract-robotics-nanobind")
        # Check for direct_url.json which indicates editable install
        direct_url = dist.read_text("direct_url.json")
        if direct_url:
            data = json.loads(direct_url)
            return data.get("dir_info", {}).get("editable", False)
    except Exception:
        pass
    # Fallback: check if __file__ is outside site-packages
    pkg_path = Path(__file__).parent
    return "site-packages" not in str(pkg_path)


def _resolve_config_paths(config_path: Path, plugin_path) -> Path:
    """
    Resolve plugin path placeholders in task composer YAML configs.

    Handles both:
    - @PLUGIN_PATH@ placeholder (bundled wheels)
    - /usr/local/lib (dev mode with ws/install)
    """
    if plugin_path is None:
        return config_path

    content = config_path.read_text()

    # Check if config needs patching
    needs_patch = "@PLUGIN_PATH@" in content or "/usr/local/lib" in content
    if not needs_patch:
        return config_path

    # Generate resolved config in cache dir
    import hashlib
    cache_dir = Path(__file__).parent / ".cache"
    cache_dir.mkdir(exist_ok=True)

    # Use hash of plugin_path to allow different installs
    path_hash = hashlib.md5(plugin_path.encode()).hexdigest()[:8]
    resolved_path = cache_dir / f"{config_path.stem}_{path_hash}.yaml"

    # Only regenerate if source changed or cache missing
    if not resolved_path.exists() or resolved_path.stat().st_mtime < config_path.stat().st_mtime:
        resolved_content = content.replace("@PLUGIN_PATH@", plugin_path)
        resolved_content = resolved_content.replace("/usr/local/lib", plugin_path)
        resolved_path.write_text(resolved_content)

    return resolved_path


def _configure_environment():
    """Set env vars using bundled data paths or dev workspace (if not already set)."""
    pkg_dir = Path(__file__).parent.resolve()

    # Try bundled data first (installed package)
    data_dir = pkg_dir / "data"
    support_dir = data_dir / "tesseract_support"
    config_dir = data_dir / "task_composer_config"

    # Fallback: dev workspace layout (editable install)
    # pkg_dir = tesseract_nanobind/src/tesseract_robotics -> project root is 3 levels up
    project_root = pkg_dir.parent.parent.parent
    ws_support = project_root / "ws" / "src" / "tesseract" / "tesseract_support"
    ws_resource = project_root / "ws" / "src" / "tesseract"
    ws_composer = project_root / "ws" / "src" / "tesseract_planning" / "tesseract_task_composer"
    ws_config = ws_composer / "config" / "task_composer_plugins_no_trajopt_ifopt.yaml"

    # TESSERACT_SUPPORT_DIR
    if "TESSERACT_SUPPORT_DIR" not in os.environ:
        if support_dir.is_dir():
            os.environ["TESSERACT_SUPPORT_DIR"] = str(support_dir)
        elif ws_support.is_dir():
            os.environ["TESSERACT_SUPPORT_DIR"] = str(ws_support)

    # TESSERACT_RESOURCE_PATH
    if "TESSERACT_RESOURCE_PATH" not in os.environ:
        if support_dir.is_dir():
            os.environ["TESSERACT_RESOURCE_PATH"] = str(data_dir)
        elif ws_resource.is_dir():
            os.environ["TESSERACT_RESOURCE_PATH"] = str(ws_resource)

    # Plugin search paths - env var, bundled plugins, or ws/install/lib
    # Linux: pkg_dir (all deps bundled in package root with $ORIGIN rpath)
    # macOS: .dylibs (delocate-repaired)
    plugin_path = os.environ.get("TESSERACT_PLUGIN_PATH")  # explicit override
    editable = _is_editable_install()

    if not plugin_path:
        # Check for bundled plugins (Linux wheel - plugins in package root)
        bundled_plugin = pkg_dir / "libtesseract_collision_bullet_factories.so"
        dylibs_dir = pkg_dir / ".dylibs"  # macOS bundled (delocate-repaired)
        ws_install_lib = project_root / "ws" / "install" / "lib"

        if bundled_plugin.exists():
            plugin_path = str(pkg_dir)
        elif dylibs_dir.is_dir():
            plugin_path = str(dylibs_dir)
        elif editable and ws_install_lib.is_dir():
            plugin_path = str(ws_install_lib)

    # TESSERACT_TASK_COMPOSER_CONFIG_FILE
    # Check if patching is needed (for bundled wheels or if env.sh set a path)
    env_cfg = os.environ.get("TESSERACT_TASK_COMPOSER_CONFIG_FILE")
    if env_cfg and plugin_path:
        env_cfg_path = Path(env_cfg)
        if env_cfg_path.is_file():
            cfg_resolved = _resolve_config_paths(env_cfg_path, plugin_path)
            os.environ["TESSERACT_TASK_COMPOSER_CONFIG_FILE"] = str(cfg_resolved)
    elif "TESSERACT_TASK_COMPOSER_CONFIG_FILE" not in os.environ:
        cfg = config_dir / "task_composer_plugins_no_trajopt_ifopt.yaml"
        if cfg.is_file():
            cfg_resolved = _resolve_config_paths(cfg, plugin_path) if plugin_path else cfg
            os.environ["TESSERACT_TASK_COMPOSER_CONFIG_FILE"] = str(cfg_resolved)
        elif ws_config.is_file():
            cfg_resolved = _resolve_config_paths(ws_config, plugin_path) if plugin_path else ws_config
            os.environ["TESSERACT_TASK_COMPOSER_CONFIG_FILE"] = str(cfg_resolved)

    # TESSERACT_TASK_COMPOSER_DIR (needed by some code paths)
    if "TESSERACT_TASK_COMPOSER_DIR" not in os.environ:
        if config_dir.is_dir():
            os.environ["TESSERACT_TASK_COMPOSER_DIR"] = str(config_dir)
        elif ws_composer.is_dir():
            os.environ["TESSERACT_TASK_COMPOSER_DIR"] = str(ws_composer)

    # Set plugin directories for all plugin factories (if not overridden)
    if plugin_path:
        for env_var in [
            "TESSERACT_CONTACT_MANAGERS_PLUGIN_DIRECTORIES",
            "TESSERACT_KINEMATICS_PLUGIN_DIRECTORIES",
            "TESSERACT_TASK_COMPOSER_PLUGIN_DIRECTORIES",
        ]:
            if env_var not in os.environ:
                os.environ[env_var] = plugin_path


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
