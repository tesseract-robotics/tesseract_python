"""tesseract_common Python bindings (nanobind)"""

from tesseract_robotics.tesseract_common._tesseract_common import *


# FilesystemPath - for SWIG compatibility, we keep it as a str subclass
# This allows it to be passed directly to functions expecting strings
# The C++ binding is available as _FilesystemPath for cases that need it
from tesseract_robotics.tesseract_common._tesseract_common import FilesystemPath as _FilesystemPath

class FilesystemPath(str):
    """A filesystem path wrapper for SWIG compatibility.

    Subclasses str so it can be passed to C++ functions expecting strings.
    Use _FilesystemPath for the raw C++ binding when needed.
    """
    def __new__(cls, path):
        return str.__new__(cls, path)

    def string(self):
        """Return the path as a string (SWIG compatibility)."""
        return str(self)

# TransformMap is a dict wrapper for SWIG compatibility
# In nanobind, plain Python dicts with string keys and Isometry3d values work automatically
class TransformMap(dict):
    """A dict wrapper for SWIG compatibility with TransformMap.

    In nanobind, Python dicts automatically convert to std::map<string, Isometry3d>.
    This class provides compatibility with code written for SWIG bindings.
    """
    pass

__all__ = [
    # Core types
    "ResourceLocator",
    "Resource",
    "GeneralResourceLocator",
    "SimpleLocatedResource",
    "BytesResource",

    # Manipulator
    "ManipulatorInfo",
    "JointState",

    # Collision
    "AllowedCollisionMatrix",
    "CollisionMarginData",
    "CollisionMarginOverrideType",

    # Kinematics
    "KinematicLimits",

    # Console bridge
    "OutputHandler",
    "LogLevel",
    "setLogLevel",
    "getLogLevel",
    "useOutputHandler",
    "restorePreviousOutputHandler",

    # Logging levels
    "CONSOLE_BRIDGE_LOG_DEBUG",
    "CONSOLE_BRIDGE_LOG_INFO",
    "CONSOLE_BRIDGE_LOG_WARN",
    "CONSOLE_BRIDGE_LOG_ERROR",
    "CONSOLE_BRIDGE_LOG_NONE",

    # Eigen helper types
    "Isometry3d",
    "Translation3d",
    "Quaterniond",
    "AngleAxisd",

    # Console bridge log function
    "log",

    # Plugin
    "PluginInfo",

    # Container types (SWIG compatibility)
    "VectorVector3d",
    "VectorIsometry3d",

    # Filesystem (SWIG compatibility)
    "FilesystemPath",

    # Transform map (SWIG compatibility)
    "TransformMap",
]
