.. SphinxTest documentation master file, created by
   sphinx-quickstart on Tue Oct  3 11:09:13 2017.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

====================================
Welcome to the Tesseract Python wiki
====================================

Python wrapper for Tesseract package, generated automatically by SWIG.

.. Tesseract Python Supported Packages
.. -----------------------------------

.. * **tesseract_collision** – This package contains privides a common interface for collision checking prividing several implementation of a Bullet collision library and FCL collision library. It includes both continuous and discrete collision checking for convex-convex, convex-concave and concave-concave shapes.
.. * **tesseract_common** – This package contains common functionality needed by the majority of the packages.
.. * **tesseract_environment** – This package contains the Tesseract Environment which provides functionality to add,remove,move and modify links and joint. It also manages adding object to the contact managers and provides the ability.
.. * **tesseract_geometry** – This package contains geometry types used by Tesseract including primitive shapes, mesh, convex hull mesh, octomap and signed distance field.
.. * **tesseract_kinematics** –  This package contains a common interface for Forward and Inverse kinematics for Chain, Tree's and Graphs including implementation using KDL and OPW Kinematics.
.. * **tesseract_scene_graph** – This package contains the scene graph which is the data structure used to manage the connectivity of objects in the environment. It inherits from boost graph and provides addition functionality for adding,removing and modifying Links and Joints along with search implementation.
.. * **tesseract_support** – This package contains support data used for unit tests and examples throughout Tesseract.
.. * **tesseract_urdf** - This package contains a custom urdf parser supporting addition shapes and features currently not supported by urdfdom.
.. * **tesseract_visualization** – This package contains visualization utilities and libraries.
.. * **tesseract_command_language** - This is a generic programing language used as input for motion and process planning. It is very similar to how you currently program a robot on an industrial teach pendant.
.. * **tesseract_motion_planners** – This package contains a common interface for Planners and includes implementation for OMPL, TrajOpt, TrajOpt IFOPT and Descartes.
.. * **tesseract_process_managers** – This package contains a common interface for Process Planning and includes implementation for a wide variaty of process found industrial automation like painting, griding, welding, pick and place and more.
.. * **tesseract_time_parameterization** – This package contains a time parameterization algorithms and includes iteritive spline.

Related Repositories
--------------------

* [Tesseract](https://github.com/ros-industrial-consortium/tesseract)
* [Tesseract Planning](https://github.com/ros-industrial-consortium/tesseract_planning)

Contents
------------

.. toctree::
   :maxdepth: 2

      Readme <readme.md>
      Examples <_source/examples/examples.rst>
      API Reference <_source/modules/api.rst>

      
..    tesseract_scene_graph <_source/tesseract_scene_graph_doc.rst>
..    tesseract_collision <_source/tesseract_collision_doc.rst>
..    tesseract_geometry <_source/tesseract_geometry_doc.rst>
..    tesseract_ros <_source/tesseract_ros_doc.rst>
..    tesseract_msgs <_source/tesseract_msgs_doc.rst>
..    tesseract_rviz <_source/tesseract_rviz_doc.rst>
..    tesseract_monitoring <_source/tesseract_monitoring_doc.rst>
..    tesseract_planning <_source/tesseract_planning_doc.rst>
..    tesseract_urdf <_source/tesseract_urdf_doc.rst>
..    tesseract_srdf <_source/tesseract_srdf_doc.rst>


