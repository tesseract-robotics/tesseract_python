"""Tests that run the example scripts to verify API coverage.

Markers:
  - @pytest.mark.viewer: Viewer examples
  - @pytest.mark.planning: Motion planning examples
  - @pytest.mark.basic: Basic examples (collision, kinematics, scene_graph)
  - @pytest.mark.lowlevel: Low-level API examples
"""
from pathlib import Path
import importlib.util
import pytest

ROOT_DIR = Path(__file__).parent.parent.parent.parent
VIEWER_EXAMPLES = ROOT_DIR / "tesseract_viewer_python" / "examples"
EXAMPLES = ROOT_DIR / "examples"
LOWLEVEL_EXAMPLES = EXAMPLES / "lowlevel"


def _load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


# === Viewer Examples ===

@pytest.mark.viewer
def test_shapes_viewer():
    _load_module("shapes_viewer", VIEWER_EXAMPLES / "shapes_viewer.py").main()


@pytest.mark.viewer
def test_material_mesh_viewer():
    _load_module("tesseract_material_mesh_viewer", VIEWER_EXAMPLES / "tesseract_material_mesh_viewer.py").main()


@pytest.mark.viewer
@pytest.mark.planning
def test_abb_irb2400_viewer():
    _load_module("abb_irb2400_viewer", VIEWER_EXAMPLES / "abb_irb2400_viewer.py").main()


# === High-Level API Examples ===

@pytest.mark.basic
def test_collision_example():
    _load_module("tesseract_collision_example", EXAMPLES / "tesseract_collision_example.py").main()


@pytest.mark.basic
def test_kinematics_example():
    _load_module("tesseract_kinematics_example", EXAMPLES / "tesseract_kinematics_example.py").main()


@pytest.mark.planning
def test_freespace_ompl_example():
    _load_module("freespace_ompl_example", EXAMPLES / "freespace_ompl_example.py").main()


@pytest.mark.planning
def test_basic_cartesian_example():
    _load_module("basic_cartesian_example", EXAMPLES / "basic_cartesian_example.py").main()


@pytest.mark.planning
def test_glass_upright_example():
    _load_module("glass_upright_example", EXAMPLES / "glass_upright_example.py").main()


@pytest.mark.planning
def test_pick_and_place_example():
    _load_module("pick_and_place_example", EXAMPLES / "pick_and_place_example.py").main()


@pytest.mark.planning
def test_car_seat_example():
    _load_module("car_seat_example", EXAMPLES / "car_seat_example.py").main()


@pytest.mark.planning
def test_puzzle_piece_auxillary_axes_example():
    _load_module("puzzle_piece_auxillary_axes_example", EXAMPLES / "puzzle_piece_auxillary_axes_example.py").main()


# === Low-Level API Examples ===

@pytest.mark.lowlevel
@pytest.mark.basic
def test_lowlevel_collision_example():
    _load_module("tesseract_collision_c_api_example", LOWLEVEL_EXAMPLES / "tesseract_collision_c_api_example.py").main()


@pytest.mark.lowlevel
@pytest.mark.basic
def test_lowlevel_kinematics_example():
    _load_module("tesseract_kinematics_c_api_example", LOWLEVEL_EXAMPLES / "tesseract_kinematics_c_api_example.py").main()


@pytest.mark.lowlevel
@pytest.mark.basic
def test_lowlevel_scene_graph_example():
    _load_module("scene_graph_c_api_example", LOWLEVEL_EXAMPLES / "scene_graph_c_api_example.py").main()


@pytest.mark.lowlevel
@pytest.mark.planning
def test_lowlevel_freespace_ompl_example():
    _load_module("freespace_ompl_c_api_example", LOWLEVEL_EXAMPLES / "freespace_ompl_c_api_example.py").main()


@pytest.mark.lowlevel
@pytest.mark.planning
def test_lowlevel_basic_cartesian_example():
    _load_module("basic_cartesian_c_api_example", LOWLEVEL_EXAMPLES / "basic_cartesian_c_api_example.py").main()


@pytest.mark.lowlevel
@pytest.mark.planning
def test_lowlevel_glass_upright_example():
    _load_module("glass_upright_c_api_example", LOWLEVEL_EXAMPLES / "glass_upright_c_api_example.py").main()


@pytest.mark.lowlevel
@pytest.mark.planning
def test_lowlevel_puzzle_piece_example():
    _load_module("puzzle_piece_c_api_example", LOWLEVEL_EXAMPLES / "puzzle_piece_c_api_example.py").main()


@pytest.mark.lowlevel
@pytest.mark.planning
def test_lowlevel_pick_and_place_example():
    _load_module("pick_and_place_c_api_example", LOWLEVEL_EXAMPLES / "pick_and_place_c_api_example.py").main()


@pytest.mark.lowlevel
@pytest.mark.planning
def test_lowlevel_car_seat_example():
    _load_module("car_seat_c_api_example", LOWLEVEL_EXAMPLES / "car_seat_c_api_example.py").main()


@pytest.mark.lowlevel
@pytest.mark.planning
def test_lowlevel_puzzle_piece_auxillary_axes_example():
    _load_module("puzzle_piece_auxillary_axes_c_api_example", LOWLEVEL_EXAMPLES / "puzzle_piece_auxillary_axes_c_api_example.py").main()
