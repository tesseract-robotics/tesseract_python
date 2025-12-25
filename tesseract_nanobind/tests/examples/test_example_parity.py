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


@pytest.mark.planning
def test_car_seat_parity():
    """Verify car_seat_example.py matches car_seat_c_api_example.py."""
    # Load and run both examples
    highlevel_mod = _load_module("car_seat_example", EXAMPLES / "car_seat_example.py")
    lowlevel_mod = _load_module("car_seat_c_api_example", LOWLEVEL / "car_seat_c_api_example.py")

    highlevel = highlevel_mod.run()
    lowlevel = lowlevel_mod.run()

    # Both should succeed
    assert highlevel["pick_result"].successful
    assert lowlevel["pick_result"].successful
    assert highlevel["place_result"].successful
    assert lowlevel["place_result"].successful

    # Waypoint counts should be similar (allow small variance due to optimization)
    pick_diff = abs(len(highlevel["pick_result"]) - len(lowlevel["pick_result"]))
    place_diff = abs(len(highlevel["place_result"]) - len(lowlevel["place_result"]))
    assert pick_diff <= 2, f"PICK waypoint count differs: {len(highlevel['pick_result'])} vs {len(lowlevel['pick_result'])}"
    assert place_diff <= 2, f"PLACE waypoint count differs: {len(highlevel['place_result'])} vs {len(lowlevel['place_result'])}"

    # Final positions should match (both reach same goal)
    highlevel_pick_final = _get_final_positions(highlevel["pick_result"])
    lowlevel_pick_final = _get_final_positions(lowlevel["pick_result"])
    if highlevel_pick_final is not None and lowlevel_pick_final is not None:
        np.testing.assert_array_almost_equal(
            highlevel_pick_final, lowlevel_pick_final, decimal=2,
            err_msg="PICK final positions differ"
        )

    highlevel_place_final = _get_final_positions(highlevel["place_result"])
    lowlevel_place_final = _get_final_positions(lowlevel["place_result"])
    if highlevel_place_final is not None and lowlevel_place_final is not None:
        np.testing.assert_array_almost_equal(
            highlevel_place_final, lowlevel_place_final, decimal=2,
            err_msg="PLACE final positions differ"
        )


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

    # Waypoint counts should be similar (TrajOpt may vary slightly)
    pick_diff = abs(len(highlevel["pick_result"]) - len(lowlevel["pick_result"]))
    place_diff = abs(len(highlevel["place_result"]) - len(lowlevel["place_result"]))
    assert pick_diff <= 5, f"PICK waypoint count differs: {len(highlevel['pick_result'])} vs {len(lowlevel['pick_result'])}"
    assert place_diff <= 5, f"PLACE waypoint count differs: {len(highlevel['place_result'])} vs {len(lowlevel['place_result'])}"

    # Final positions should match (both reach same goal pose)
    highlevel_pick_final = _get_final_positions(highlevel["pick_result"])
    lowlevel_pick_final = _get_final_positions(lowlevel["pick_result"])
    if highlevel_pick_final is not None and lowlevel_pick_final is not None:
        np.testing.assert_array_almost_equal(
            highlevel_pick_final, lowlevel_pick_final, decimal=1,
            err_msg="PICK final positions differ"
        )

    highlevel_place_final = _get_final_positions(highlevel["place_result"])
    lowlevel_place_final = _get_final_positions(lowlevel["place_result"])
    if highlevel_place_final is not None and lowlevel_place_final is not None:
        np.testing.assert_array_almost_equal(
            highlevel_place_final, lowlevel_place_final, decimal=1,
            err_msg="PLACE final positions differ"
        )
