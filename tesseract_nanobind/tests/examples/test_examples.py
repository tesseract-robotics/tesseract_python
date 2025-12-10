"""Tests that run the example scripts to verify API coverage.

These tests import and run each example's main() function.
pytest in sys.modules = headless mode (no viewer).

Markers:
  - @pytest.mark.viewer: Viewer/visualization examples
  - @pytest.mark.planning: Motion planning examples (require TESSERACT_TASK_COMPOSER_CONFIG_FILE)
  - @pytest.mark.basic: Basic examples (collision, kinematics, scene_graph)

Usage:
  pytest -m viewer              # Run only viewer examples
  pytest -m planning            # Run only planning examples
  pytest -m basic               # Run only basic examples
  pytest -m "not planning"      # Run all except planning examples
"""
import os
import importlib.util
import pytest

# Path: tesseract_nanobind/tests/examples/test_examples.py
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


@pytest.mark.viewer
def test_shapes_viewer():
    """Test shapes_viewer example runs without error."""
    module = _load_module("shapes_viewer", os.path.join(VIEWER_EXAMPLES, "shapes_viewer.py"))
    module.main()


@pytest.mark.viewer
def test_material_mesh_viewer():
    """Test material mesh viewer example."""
    module = _load_module("tesseract_material_mesh_viewer", os.path.join(VIEWER_EXAMPLES, "tesseract_material_mesh_viewer.py"))
    module.main()


@pytest.mark.viewer
@pytest.mark.planning
def test_abb_irb2400_viewer():
    """Test ABB IRB2400 OMPL planning example."""
    module = _load_module("abb_irb2400_viewer", os.path.join(VIEWER_EXAMPLES, "abb_irb2400_viewer.py"))
    module.main()


@pytest.mark.basic
def test_collision_example():
    """Test collision checking example."""
    module = _load_module("tesseract_collision_example", os.path.join(CORE_EXAMPLES, "tesseract_collision_example.py"))
    module.main()


@pytest.mark.basic
def test_kinematics_example():
    """Test kinematics example."""
    module = _load_module("tesseract_kinematics_example", os.path.join(CORE_EXAMPLES, "tesseract_kinematics_example.py"))
    module.main()


@pytest.mark.basic
def test_scene_graph_example():
    """Test scene graph example."""
    module = _load_module("scene_graph_example", os.path.join(CORE_EXAMPLES, "scene_graph_example.py"))
    module.main()


@pytest.mark.planning
def test_freespace_ompl_example():
    """Test freespace OMPL example."""
    if not os.environ.get("TESSERACT_TASK_COMPOSER_CONFIG_FILE"):
        pytest.skip("TESSERACT_TASK_COMPOSER_CONFIG_FILE not set")
    module = _load_module("freespace_ompl_example", os.path.join(CORE_EXAMPLES, "freespace_ompl_example.py"))
    module.main()


@pytest.mark.planning
def test_basic_cartesian_example():
    """Test basic cartesian example."""
    if not os.environ.get("TESSERACT_TASK_COMPOSER_CONFIG_FILE"):
        pytest.skip("TESSERACT_TASK_COMPOSER_CONFIG_FILE not set")
    module = _load_module("basic_cartesian_example", os.path.join(CORE_EXAMPLES, "basic_cartesian_example.py"))
    module.main()


@pytest.mark.planning
def test_glass_upright_example():
    """Test glass upright example."""
    if not os.environ.get("TESSERACT_TASK_COMPOSER_CONFIG_FILE"):
        pytest.skip("TESSERACT_TASK_COMPOSER_CONFIG_FILE not set")
    module = _load_module("glass_upright_example", os.path.join(CORE_EXAMPLES, "glass_upright_example.py"))
    module.main()


@pytest.mark.planning
def test_puzzle_piece_example():
    """Test puzzle piece example."""
    if not os.environ.get("TESSERACT_TASK_COMPOSER_CONFIG_FILE"):
        pytest.skip("TESSERACT_TASK_COMPOSER_CONFIG_FILE not set")
    module = _load_module("puzzle_piece_example", os.path.join(CORE_EXAMPLES, "puzzle_piece_example.py"))
    module.main()


@pytest.mark.planning
def test_pick_and_place_example():
    """Test pick and place example."""
    if not os.environ.get("TESSERACT_TASK_COMPOSER_CONFIG_FILE"):
        pytest.skip("TESSERACT_TASK_COMPOSER_CONFIG_FILE not set")
    module = _load_module("pick_and_place_example", os.path.join(CORE_EXAMPLES, "pick_and_place_example.py"))
    module.main()


@pytest.mark.planning
def test_car_seat_example():
    """Test car seat example - demonstrates dynamic mesh loading and convex hulls."""
    if not os.environ.get("TESSERACT_TASK_COMPOSER_CONFIG_FILE"):
        pytest.skip("TESSERACT_TASK_COMPOSER_CONFIG_FILE not set")
    module = _load_module("car_seat_example", os.path.join(CORE_EXAMPLES, "car_seat_example.py"))
    module.main()


@pytest.mark.planning
def test_puzzle_piece_auxillary_axes_example():
    """Test puzzle piece auxiliary axes - demonstrates 9-DOF planning (7 arm + 2 positioner)."""
    if not os.environ.get("TESSERACT_TASK_COMPOSER_CONFIG_FILE"):
        pytest.skip("TESSERACT_TASK_COMPOSER_CONFIG_FILE not set")
    module = _load_module("puzzle_piece_auxillary_axes_example", os.path.join(CORE_EXAMPLES, "puzzle_piece_auxillary_axes_example.py"))
    module.main()
