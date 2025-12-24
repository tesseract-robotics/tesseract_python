# Low-Level Examples

Direct C++ binding examples for tesseract_robotics.

These examples demonstrate the raw nanobind API - a 1:1 mapping of the C++ Tesseract libraries. Use these as reference for understanding the underlying mechanics or when you need fine-grained control not exposed by the high-level API.

**For most use cases, prefer the [high-level examples](../) instead.**

## Examples

| Example | Description |
|---------|-------------|
| `freespace_ompl_example.py` | OMPL planning with manual environment setup |
| `basic_cartesian_example.py` | TrajOpt Cartesian planning |
| `glass_upright_example.py` | Constrained planning with profiles |
| `puzzle_piece_example.py` | Toolpath from CSV with TrajOpt |
| `puzzle_piece_auxillary_axes_example.py` | 9-DOF planning |
| `pick_and_place_example.py` | Attach/detach collision objects |
| `car_seat_example.py` | Multi-phase planning with meshes |
| `scene_graph_example.py` | Scene graph manipulation |
| `tesseract_collision_example.py` | Collision manager API |
| `tesseract_kinematics_example.py` | Kinematics groups |
| `tesseract_planning_lowlevel_example.py` | Planning without TaskComposer |
| `tesseract_planning_example_composer.py` | TaskComposer usage |

## When to Use Low-Level API

- Implementing custom planners
- Accessing features not yet wrapped in high-level API
- Porting C++ code directly
- Debugging planning issues at the binding level
