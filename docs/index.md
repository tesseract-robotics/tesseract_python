# Tesseract Robotics Python (nanobind)

Python bindings for [Tesseract](https://github.com/tesseract-robotics/tesseract)
robotics motion planning using [nanobind](https://github.com/wjakob/nanobind).

Developed and maintained by Wason Technology, LLC and Jelle Feringa.

## Features

- Scene loading and management (URDF, SRDF, meshes)
- Collision checking (Bullet, FCL)
- Kinematics (KDL, OPW, UR)
- Motion planning (OMPL, Descartes, TrajOpt)
- Time parameterization (TOTG, ISP, Ruckig)
- Task composition and pipelines
- Pythonic high-level API

## Related Repositories

- [Tesseract](https://github.com/tesseract-robotics/tesseract)
- [Tesseract Planning](https://github.com/tesseract-robotics/tesseract_planning)
- [General Robotics Toolbox](https://github.com/rpiRobotics/rpi_general_robotics_toolbox_py)
- [PyRI Open Source Teach Pendant](https://github.com/pyri-project)

## Contents

```{toctree}
:maxdepth: 2

readme.md
QUICKSTART.md
DEPENDENCIES.md
MIGRATION_NOTES.md
_source/important_notes.md
_source/examples/examples.md
_source/modules/api.rst
```
