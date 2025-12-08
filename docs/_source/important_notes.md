# Important Usage Notes

This page contains important notes for using the Tesseract Python bindings.

## Nanobind Bindings

The Python bindings are generated using [nanobind](https://github.com/wjakob/nanobind), a modern
C++17 binding library. Almost the entire Tesseract API is available in Python. The bindings provide
native NumPy integration for Eigen types and automatic handling of C++ smart pointers.

## C++ Storage and Pointer Types

The Tesseract library uses several common C++ storage and pointer patterns:

- **pass-by-copy**: These objects are copied into Python and managed by Python's garbage collector.
- **shared pointers**: Automatically reference-counted. Destroyed when no references remain.
- **unique pointers**: Nanobind automatically extracts unique_ptr contents, so `result.release()`
  is not needed (unlike SWIG bindings).

## Template Containers

The Tesseract library uses C++ container templates extensively. These are wrapped by nanobind.
Typical examples include `std::vector`, `std::map`, and `std::unordered_map`. Each specific
implementation of the template is wrapped, for example `std::vector<std::string>` is `StringVector`.
Most of these templates are contained in `tesseract_common`, however they can be found in many modules.
These template types roughly correspond to lists and dictionaries in Python. They can normally be
implicitly created using lists and dictionaries, and when returned have roughly the same accessor
methods.

## Eigen Types

Eigen matrices and vectors are used extensively in Tesseract. These are automatically converted
to NumPy arrays by nanobind. This conversion works for `float64` types (`double` in C++) and
`int32` types (`int` in C++). The conversion is done using efficient buffer protocols with
minimal copying where possible.

Some Eigen geometry types are wrapped as classes in the `tesseract_common` module. See the
`tesseract_common` documentation for more information and examples.

## Pythonic High-Level API

In addition to the low-level C++ API bindings, a Pythonic high-level API is available in
`tesseract_robotics.planning`. This provides a more user-friendly interface for common
motion planning tasks:

```python
from tesseract_robotics.planning import (
    Robot, MotionProgram, JointTarget, CartesianTarget,
    Pose, box, create_obstacle, TaskComposer,
)

# Load robot
robot = Robot.from_urdf(
    "package://tesseract_support/urdf/abb_irb2400.urdf",
    "package://tesseract_support/urdf/abb_irb2400.srdf"
)

# Add obstacle
create_obstacle(robot, "box", box(0.5, 0.5, 0.5), Pose.from_xyz(0.5, 0, 0.3))

# Build motion program
program = (MotionProgram("manipulator", tcp_frame="tool0")
    .set_joint_names(robot.get_joint_names("manipulator"))
    .move_to(JointTarget([0, 0, 0, 0, 0, 0]))
    .move_to(CartesianTarget(Pose.from_xyz(0.5, 0.3, 0.8)))
)

# Plan
composer = TaskComposer.from_config()
result = composer.plan(robot, program)
```

## Documentation Generation

The Python documentation is generated using Sphinx autodoc and nanobind's docstring support.
If you find errors, please submit a pull request to fix them.
