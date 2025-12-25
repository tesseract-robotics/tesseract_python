"""Tests verifying high-level and low-level examples produce similar results.

This ensures the tesseract_robotics.planning high-level API produces
trajectories comparable to the verbose low-level examples (which more
closely match the C++ tesseract_examples).
"""
from pathlib import Path
import importlib.util
import numpy as np
import pytest

ROOT_DIR = Path(__file__).parent.parent.parent.parent
EXAMPLES = ROOT_DIR / "examples"
LOWLEVEL = EXAMPLES / "lowlevel"


def _load_module(name, path):
    """Load a Python module from path."""
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _get_final_positions(result):
    """Extract final joint positions from a PlanningResult."""
    if hasattr(result, 'trajectory') and result.trajectory:
        return result.trajectory[-1].positions
    return None


# Single-result examples: (name, waypoint_tolerance, decimal_precision)
SINGLE_RESULT_EXAMPLES = [
    ("freespace_ompl", 5, 2),
    ("basic_cartesian", 5, 2),
    ("glass_upright", 5, 2),
]


@pytest.mark.planning
@pytest.mark.parametrize("name,waypoint_tol,decimal", SINGLE_RESULT_EXAMPLES)
def test_single_result_parity(name, waypoint_tol, decimal):
    """Verify single-result examples match between high-level and low-level."""
    highlevel = _load_module(f"{name}_example", EXAMPLES / f"{name}_example.py")
    lowlevel = _load_module(f"{name}_c_api_example", LOWLEVEL / f"{name}_c_api_example.py")

    hl = highlevel.run()
    ll = lowlevel.run()

    assert hl["result"].successful
    assert ll["result"].successful

    diff = abs(len(hl["result"]) - len(ll["result"]))
    assert diff <= waypoint_tol, f"Waypoint count differs: {len(hl['result'])} vs {len(ll['result'])}"

    hl_final = _get_final_positions(hl["result"])
    ll_final = _get_final_positions(ll["result"])
    if hl_final is not None and ll_final is not None:
        np.testing.assert_array_almost_equal(hl_final, ll_final, decimal=decimal)


@pytest.mark.planning
def test_car_seat_parity():
    """Verify car_seat_example.py matches car_seat_c_api_example.py."""
    highlevel_mod = _load_module("car_seat_example", EXAMPLES / "car_seat_example.py")
    lowlevel_mod = _load_module("car_seat_c_api_example", LOWLEVEL / "car_seat_c_api_example.py")

    highlevel = highlevel_mod.run()
    lowlevel = lowlevel_mod.run()

    # Both should succeed
    assert highlevel["pick_result"].successful
    assert lowlevel["pick_result"].successful
    assert highlevel["place_result"].successful
    assert lowlevel["place_result"].successful

    # Waypoint counts should be similar
    pick_diff = abs(len(highlevel["pick_result"]) - len(lowlevel["pick_result"]))
    place_diff = abs(len(highlevel["place_result"]) - len(lowlevel["place_result"]))
    assert pick_diff <= 2, f"PICK: {len(highlevel['pick_result'])} vs {len(lowlevel['pick_result'])}"
    assert place_diff <= 2, f"PLACE: {len(highlevel['place_result'])} vs {len(lowlevel['place_result'])}"

    # Final positions should match
    for phase in ["pick_result", "place_result"]:
        hl_final = _get_final_positions(highlevel[phase])
        ll_final = _get_final_positions(lowlevel[phase])
        if hl_final is not None and ll_final is not None:
            np.testing.assert_array_almost_equal(hl_final, ll_final, decimal=2)


@pytest.mark.planning
def test_pick_and_place_parity():
    """Verify pick_and_place_example.py matches pick_and_place_c_api_example.py."""
    highlevel_mod = _load_module("pick_and_place_example", EXAMPLES / "pick_and_place_example.py")
    lowlevel_mod = _load_module("pick_and_place_c_api_example", LOWLEVEL / "pick_and_place_c_api_example.py")

    highlevel = highlevel_mod.run()
    lowlevel = lowlevel_mod.run()

    # Both should succeed
    assert highlevel["pick_result"].successful
    assert lowlevel["pick_result"].successful
    assert highlevel["place_result"].successful
    assert lowlevel["place_result"].successful

    # Waypoint counts should be similar (TrajOpt may vary)
    pick_diff = abs(len(highlevel["pick_result"]) - len(lowlevel["pick_result"]))
    place_diff = abs(len(highlevel["place_result"]) - len(lowlevel["place_result"]))
    assert pick_diff <= 5, f"PICK: {len(highlevel['pick_result'])} vs {len(lowlevel['pick_result'])}"
    assert place_diff <= 5, f"PLACE: {len(highlevel['place_result'])} vs {len(lowlevel['place_result'])}"

    # Final positions should match (looser tolerance for pick_and_place)
    for phase in ["pick_result", "place_result"]:
        hl_final = _get_final_positions(highlevel[phase])
        ll_final = _get_final_positions(lowlevel[phase])
        if hl_final is not None and ll_final is not None:
            np.testing.assert_array_almost_equal(hl_final, ll_final, decimal=1)
