================
tesseract_robotics.tesseract_common
================

Introduction
************

``tesseract_common`` contains functionality that is common to all tesseract packages. This includes general types,
container templates, utility functions, and more. The Python wrappers contain additional functionality for 
using with Eigen Geometry, Boost Filesystem, Boost Uuid, console_bridge, and TinyXML2.

Eigen Geometry
--------------

Matrix and vector types for Eigen are automatically converted to NumPy arrays when returned from a function, and 
NumPy arrays are automatically converted to Eigen types when passed into a function. This works for matrices and
vectors, but does not work well for the Eigen geometry types used by Tesseract. The following Eigen classes are
available in ``tesseract_common``:

* ``Quaterniond`` - Represents a Quaternion in (w,x,y,z) format
* ``AngleAxisd`` - Represents a rotation as an angle around an axis
* ``Translation3d`` - Represents a translation in 3D space
* ``Isometry3d`` - Represents a rotation and translation in 3D space. This is used by Tesseract to represent transforms and poses.

Boost Filesystem
----------------

``boost::filesystem::path`` is used by Tesseract to represent file paths. Because of function overloads, it is not
possible to use strings for filesystems. The ``FileSystem`` path class is used as a container for paths
to allow the correct function overloads to be selected.

Boost Uuid
----------

The ``boost::uuids::uuid`` is used by Tesseract to represent unique identifiers. The ``Uuid`` class is available 
to represent ``boost::uuids::uuid`` in Python.

The ``newRandomUuid`` function is available to generate a random ``Uuid`` in Python.

Console Bridge
--------------

The ``console_bridge`` package is used by Tesseract to provide logging functionality. Several functions are available
to modify the logging level and to log messages:

The following module level functions are available to modify the logging behavior:

* ``getLogLevel()``
* ``setLogLevel()``
* ``useOutputHandler()``
* ``noOutputHandler()``

The following module level constants are available for setting the logging level:

* ``CONSOLE_BRIDGE_LOG_NONE``
* ``CONSOLE_BRIDGE_LOG_ERROR``
* ``CONSOLE_BRIDGE_LOG_WARN``
* ``CONSOLE_BRIDGE_LOG_INFO``
* ``CONSOLE_BRIDGE_LOG_DEBUG``

The ``useOutputHandler()`` function can be used with the ``OutputHandler`` class to capture logging messages in Python:

.. code-block:: python

    class MyOutputHandler(tesseract_common.OutputHandler):
        def __init__(self):
            super().__init__()

        def log(self, text, level, filename, line):
            # Do something with the logging message
            print(text)

    output_handler = MyOutputHandler()
    tesseract_common.useOutputHandler(output_handler)

    # Always restore the original output handler when done to avoid segfaults on exit
    tesseract_common.restorePreviousOutputHandler()

TinyXML2 Support
----------------

The `tinyxml2` package is used to serialize and deserialize Tesseract types. The `tinyxml2` wrappers are included
in the `tesseract_common` package to allow for serialization and deserialization of Tesseract types in Python.

.. include:: ./api_docs_generated.rst