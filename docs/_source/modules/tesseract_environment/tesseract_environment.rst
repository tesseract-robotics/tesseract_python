========================================
tesseract_robotics.tesseract_environment
========================================

.. contents:: Table of Contents
   :depth: 2
   :local:
   :backlinks: none

The :mod:`tesseract_robotics.tesseract_environment` module contains the core classes for representing the environment.
These are typically constructed using URDF and SRDF files. The functions in the environment can be used to access
collision checkers and kinematics utility classes. Planners require an environment to plan in.

The environment can be modified using "Environment Command" classes, that are executed using the 
``applyCommand`` or ``applyCommands`` functions.


.. include:: api_docs_generated.rst