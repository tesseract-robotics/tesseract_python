====================================================
tesseract_robotics.tesseract_motion_planners_trajopt
====================================================

.. contents:: Table of Contents
   :depth: 2
   :local:
   :backlinks: none

This package provides access to the TrajOpt motion planner library. TrajOpt is a local optimizer for trajectories.
If it is likely a local planner will not find a direct path, an OMPL planner can be used to find a clear path,
and then use TrajOpt to optimize the path.
Planning is typically done using the ``tesseract_task_composer`` interface.

.. include:: api_docs_generated.rst