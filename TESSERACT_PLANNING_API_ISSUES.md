# Issues Found in `tesseract_robotics.planning`

Report for: https://github.com/tesseract-robotics/tesseract_python_nanobind


---

- expose `num_threads`
    - set a useful default
- make sure that FCL not bullet is the default collision checker
- address installation for `ws/install/share/tesseract_support/*`

---

## 1. TaskComposer Constructor Doesn't Accept Environment

**Location:** `tesseract_robotics/planning/composer.py`

**Problem:** Constructor signature is `TaskComposer(factory, locator)` but the error message when passing `robot.env` is unhelpful:

```python
# This fails with confusing error:
composer = TaskComposer(robot.env)
# AttributeError: 'Environment' object has no attribute 'createTaskComposerNode'
```

**Suggestion:** Add type checking in `__init__` with helpful error message, or add `TaskComposer.from_environment(env)` factory method.

---

## 2. Config File Discovery is Fragile

**Location:** `composer.py:158-209` (`from_config()`)

**Problem:** Multiple fallback paths with silent failures:
1. `TESSERACT_TASK_COMPOSER_CONFIG_FILE` env var
2. `TESSERACT_TASK_COMPOSER_DIR` + hardcoded path
3. `get_task_composer_config_path()` bundled config

If none exist, error message doesn't list what was tried.

**Suggestion:** Log attempted paths or include them in error message.

---

## 3. `plan_freespace()` Uses TrajOpt, Not OMPL

**Location:** `composer.py:336`

```python
def plan_freespace(self, ...):
    return self.plan(robot, program, pipeline="TrajOptPipeline", ...)
```

**Problem:** Name suggests OMPL-style freespace planning but uses TrajOpt. For actual OMPL, user must call `plan(..., pipeline="OMPLPipeline")`.

**Suggestion:** Either rename to `plan_trajopt()` or add `plan_ompl()` convenience method.

---

## 4. MotionProgram Requires Group Name

**Location:** `tesseract_robotics/planning/program.py`

**Problem:** `MotionProgram.__init__()` requires `group_name` as first positional arg, but this isn't obvious from usage examples.

```python
# Fails with confusing error:
program = MotionProgram()
# TypeError: missing 1 required positional argument: 'group_name'
```

**Suggestion:** Better error message or docstring examples.

---

## 5. No Direct OMPL Low-Level Example

The high-level API abstracts away OMPL configuration. Users wanting custom OMPL settings (specific planner like RRT*, custom parameters) must use low-level API anyway.

**Suggestion:** Add examples showing how to configure specific OMPL planners via the high-level API, or document when to use low-level API instead.

---

## 6. tesseract_support Path Not Auto-Discovered

**Problem:** When loading URDF/SRDF files that reference `package://` URIs or relative plugin configs, the resource locator fails with cryptic "new_url" errors unless environment variables are manually set.

Example error output:
```
Error: new_url: /path/to/ur5_plugins.yaml
       at line 287 in .../tesseract_common/src/resource_locator.cpp
```

Users must manually set:
```bash
TESSERACT_RESOURCE_PATH=/path/to/ws/install/share
TESSERACT_SUPPORT_DIR=/path/to/ws/install/share/tesseract_support
```

**Suggestion:**
- Auto-detect `ws/install/share` relative to `tesseract_robotics.__file__`
- Add `resource_path` parameter to `Robot.from_files()`
- Improve error message to explain required environment variables
