"""Tests that run the example scripts to verify API coverage.

These tests import and run each example's main() function in headless mode.
The goal is to verify all API methods can be invoked without error.
"""
import os
import importlib.util
import pytest

# Path: tesseract_python_nb/tests/examples/test_examples.py
# ROOT_DIR should be tesseract_python_nanobind (4 levels up)
ROOT_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
VIEWER_EXAMPLES = os.path.join(ROOT_DIR, "tesseract_viewer_python", "examples")
CORE_EXAMPLES = os.path.join(ROOT_DIR, "examples")


def _load_module(name, path):
    """Load a module from file path."""
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_shapes_viewer():
    """Test shapes_viewer example runs without error."""
    module = _load_module("shapes_viewer", os.path.join(VIEWER_EXAMPLES, "shapes_viewer.py"))
    module.main()


def test_material_mesh_viewer():
    """Test material mesh viewer example."""
    module = _load_module("tesseract_material_mesh_viewer", os.path.join(VIEWER_EXAMPLES, "tesseract_material_mesh_viewer.py"))
    module.main()


def test_abb_irb2400_viewer():
    """Test ABB IRB2400 OMPL planning example."""
    module = _load_module("abb_irb2400_viewer", os.path.join(VIEWER_EXAMPLES, "abb_irb2400_viewer.py"))
    module.main()


def test_collision_example():
    """Test collision checking example."""
    module = _load_module("tesseract_collision_example", os.path.join(CORE_EXAMPLES, "tesseract_collision_example.py"))
    module.main()


def test_kinematics_example():
    """Test kinematics example."""
    module = _load_module("tesseract_kinematics_example", os.path.join(CORE_EXAMPLES, "tesseract_kinematics_example.py"))
    module.main()


def test_scene_graph_example():
    """Test scene graph example."""
    module = _load_module("scene_graph_example", os.path.join(CORE_EXAMPLES, "scene_graph_example.py"))
    module.main()


def test_freespace_ompl_example():
    """Test freespace OMPL example."""
    if not os.environ.get("TESSERACT_TASK_COMPOSER_CONFIG_FILE"):
        pytest.skip("TESSERACT_TASK_COMPOSER_CONFIG_FILE not set")
    module = _load_module("freespace_ompl_example", os.path.join(CORE_EXAMPLES, "freespace_ompl_example.py"))
    module.main()


def test_basic_cartesian_example():
    """Test basic cartesian example."""
    if not os.environ.get("TESSERACT_TASK_COMPOSER_CONFIG_FILE"):
        pytest.skip("TESSERACT_TASK_COMPOSER_CONFIG_FILE not set")
    module = _load_module("basic_cartesian_example", os.path.join(CORE_EXAMPLES, "basic_cartesian_example.py"))
    module.main()


def test_glass_upright_example():
    """Test glass upright example."""
    if not os.environ.get("TESSERACT_TASK_COMPOSER_CONFIG_FILE"):
        pytest.skip("TESSERACT_TASK_COMPOSER_CONFIG_FILE not set")
    module = _load_module("glass_upright_example", os.path.join(CORE_EXAMPLES, "glass_upright_example.py"))
    module.main()


def test_puzzle_piece_example():
    """Test puzzle piece example."""
    if not os.environ.get("TESSERACT_TASK_COMPOSER_CONFIG_FILE"):
        pytest.skip("TESSERACT_TASK_COMPOSER_CONFIG_FILE not set")
    module = _load_module("puzzle_piece_example", os.path.join(CORE_EXAMPLES, "puzzle_piece_example.py"))
    module.main()
