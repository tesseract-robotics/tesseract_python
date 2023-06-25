tesseract_robotics_viewer Package
=========================

The ``tesseract-robotics-viewer`` package provides a web browser based viewer for the Tesseract environment using
ThreeJS. The viewer is designed to be simple and portable for use with planning development and
monitoring.

The package must be installed separately from the main ``tesseract-robotics`` package.

.. code-block:: bash

    python -m pip install tesseract-robotics-viewer

The viewer provides a synchronous and asynchronous interface. The class ``tesseract_robotics_viewer.TesseractViewer``
provides the synchronous viewer. The class ``tesseract_robotics_viewer.tesseract_viewer_aio.TesseractViewerAIO``
provides the asynchronous implementation.

See the main ``tesseract_robotics`` Python examples for examples of using the viewer. These examples use the viewer
to display the planned trajectories.

API
---

.. toctree::
    :maxdepth: 4
    :caption: Viewer API Reference:

    tesseract_robotics_viewer
    tesseract_robotics_viewer_aio

