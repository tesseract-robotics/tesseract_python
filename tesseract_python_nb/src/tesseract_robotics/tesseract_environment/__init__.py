"""tesseract_environment Python bindings (nanobind)"""

# Import dependencies first to register their types for cross-module access
import tesseract_robotics.tesseract_common  # noqa: F401 - needed for CollisionMarginData, ACM
import tesseract_robotics.tesseract_scene_graph  # noqa: F401
import tesseract_robotics.tesseract_srdf  # noqa: F401 - needed for getKinematicsInformation
import tesseract_robotics.tesseract_kinematics  # noqa: F401 - needed for getKinematicGroup

from tesseract_robotics.tesseract_environment._tesseract_environment import *

# Re-export AnyPoly_wrap_EnvironmentConst from tesseract_task_composer for convenience
try:
    from tesseract_robotics.tesseract_task_composer import AnyPoly_wrap_EnvironmentConst
except ImportError:
    pass  # task_composer may not be available

__all__ = [
    # Environment
    "Environment",

    # Base command class
    "Command",

    # Link/Joint manipulation commands
    "AddLinkCommand",
    "RemoveLinkCommand",
    "AddSceneGraphCommand",
    "RemoveJointCommand",
    "ReplaceJointCommand",
    "MoveJointCommand",
    "MoveLinkCommand",

    # Joint limits commands
    "ChangeJointPositionLimitsCommand",
    "ChangeJointVelocityLimitsCommand",
    "ChangeJointAccelerationLimitsCommand",

    # Origin/transform commands
    "ChangeJointOriginCommand",
    "ChangeLinkOriginCommand",

    # Collision commands
    "ModifyAllowedCollisionsCommand",
    "ModifyAllowedCollisionsType",
    "ModifyAllowedCollisionsType_ADD",
    "ModifyAllowedCollisionsType_REMOVE",
    "ModifyAllowedCollisionsType_REPLACE",
    "RemoveAllowedCollisionLinkCommand",
    "ChangeCollisionMarginsCommand",
    "ChangeLinkCollisionEnabledCommand",

    # Visibility commands
    "ChangeLinkVisibilityCommand",

    # Events
    "Events",
    "Event",
    "CommandAppliedEvent",
    "SceneStateChangedEvent",

    # AnyPoly wrapper (re-exported from task_composer for SWIG compatibility)
    "AnyPoly_wrap_EnvironmentConst",
]
