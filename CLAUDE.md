# Known Issues and Notes

## TaskComposer Planning

TaskComposer pipeline (used by `plan_trajopt`, `plan_ompl`, etc.) aborts consistently.
- Direct planners work (TrajOpt, OMPL via low-level API)
- TaskComposer wrapper silently aborts - needs config/profile investigation
- NOT an RTTI issue (verified by running isolated tests)

Examples affected:
- `freespace_ompl_example.py` - completes, planning fails
- `basic_cartesian_example.py` - completes, planning fails
- `glass_upright_example.py` - completes, planning fails
- `puzzle_piece_auxillary_axes_example.py` - completes, planning fails

Examples that work:
- `tesseract_collision_example.py` - works
- `tesseract_kinematics_example.py` - works
- `scene_graph_example.py` - works but has nanobind reference leaks

## nanobind Reference Leaks

`scene_graph_example.py` triggers nanobind reference leak warnings on exit:
```
nanobind: leaked function "getOrder"
nanobind: leaked function "reset"
...
nanobind: this is likely caused by a reference counting issue in the binding code.
```
See: https://nanobind.readthedocs.io/en/latest/refleaks.html

## Environment Setup

Tests require `source env.sh` to set:
- `TESSERACT_RESOURCE_PATH`
- `TESSERACT_SUPPORT_DIR`
- `TESSERACT_TASK_COMPOSER_CONFIG_FILE`
- `DYLD_LIBRARY_PATH`

Without env.sh, tests fail with resource resolution errors (not RTTI).

## Test Status

189 tests pass, 5 xfailed:
- 1 xfail: CHANGE_LINK_ORIGIN not implemented in C++
- 4 xfail: TaskComposer pipeline aborts (needs investigation)
