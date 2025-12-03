"""tesseract_common Python bindings (nanobind)"""

from tesseract_robotics.tesseract_common._tesseract_common import *

__all__ = [
    # Core types
    "ResourceLocator",
    "Resource",
    "SimpleResourceLocator",
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
    "setLogLevel",
    "getLogLevel",
    "log",
    "useOutputHandler",
    "restorePreviousOutputHandler",

    # Logging levels
    "CONSOLE_BRIDGE_LOG_DEBUG",
    "CONSOLE_BRIDGE_LOG_INFO",
    "CONSOLE_BRIDGE_LOG_WARN",
    "CONSOLE_BRIDGE_LOG_ERROR",
    "CONSOLE_BRIDGE_LOG_NONE",

    # Eigen types
    "Isometry3d",
    "Translation3d",
    "Quaterniond",
    "AngleAxisd",
    "Vector3d",
    "VectorXd",
    "MatrixXd",

    # Plugin
    "PluginInfo",
]
