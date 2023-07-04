=============================================
tesseract_robotics.tesseract_command_language
=============================================

.. contents:: Table of Contents
   :depth: 2
   :local:
   :backlinks: none

The command language is used to describe a sequence of commands that can be executed by a robot. The command language is
used to provide input waypoint sequences to the planner, and receive output trajectories from the planner.

The command language makes heavy use of type erasure. Unfortunately type erasure does not work with Python,
so module level utility functions must be used to create and cast instructions and waypoints. Example
commands that are frequently used are ``CartesianWaypointPoly_wrap_CartesianWaypoint``,
``MoveInstructionPoly_wrap_MoveInstruction``, ``InstructionPoly_as_MoveInstructionPoly``, 
and ``WaypointPoly_as_StateWaypointPoly``, howevere there are many more. While these conversions can be done
implicitly in C++, they must be done explicitly in Python.

The ``ProfileDictionary`` has a similar problem, and must use special accessor functions that are created for
each types that is stored in the dictionary. For instance, for ``OMPLDefaultPlanProfile`` the accessor
function ``ProfileDictionary_addProfile_OMPLPlanProfile`` must be used to store the profile in the dictionary.
See the planning examples for examples of using the accessors for the profile dictionary, and see the 
``ProfileDictionary_*`` accessor functions available in each module.

.. include:: api_docs_generated.rst