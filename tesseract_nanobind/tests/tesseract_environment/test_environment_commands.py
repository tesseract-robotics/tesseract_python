"""Tests for Environment Command bindings"""

import pytest
import numpy as np
from tesseract_robotics.tesseract_environment import (
    Environment,
    Command,
    AddLinkCommand,
    RemoveLinkCommand,
    AddSceneGraphCommand,
    RemoveJointCommand,
    ReplaceJointCommand,
    MoveJointCommand,
    MoveLinkCommand,
    ChangeJointPositionLimitsCommand,
    ChangeJointVelocityLimitsCommand,
    ChangeJointAccelerationLimitsCommand,
    ChangeJointOriginCommand,
    ChangeLinkOriginCommand,
    ModifyAllowedCollisionsCommand,
    ModifyAllowedCollisionsType,
    ModifyAllowedCollisionsType_ADD,
    ModifyAllowedCollisionsType_REMOVE,
    ModifyAllowedCollisionsType_REPLACE,
    RemoveAllowedCollisionLinkCommand,
    ChangeCollisionMarginsCommand,
    ChangeLinkCollisionEnabledCommand,
    ChangeLinkVisibilityCommand,
)
from tesseract_robotics.tesseract_common import (
    GeneralResourceLocator,
    AllowedCollisionMatrix,
    CollisionMarginData,
    CollisionMarginOverrideType,
    Isometry3d,
)
from tesseract_robotics.tesseract_scene_graph import Link, Joint, JointType, Visual, Collision
from tesseract_robotics.tesseract_geometry import Box


SIMPLE_URDF = """
<robot name="test_robot">
  <link name="world"/>
  <link name="link1">
    <visual><geometry><box size="0.1 0.1 0.1"/></geometry></visual>
    <collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision>
  </link>
  <link name="link2">
    <visual><geometry><box size="0.1 0.1 0.1"/></geometry></visual>
    <collision><geometry><box size="0.1 0.1 0.1"/></geometry></collision>
  </link>
  <joint name="joint1" type="revolute">
    <parent link="world"/>
    <child link="link1"/>
    <axis xyz="0 0 1"/>
    <limit effort="100" lower="-1.57" upper="1.57" velocity="1.0"/>
  </joint>
  <joint name="joint2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <axis xyz="0 0 1"/>
    <limit effort="100" lower="-1.57" upper="1.57" velocity="1.0"/>
  </joint>
</robot>
"""


@pytest.fixture
def env():
    """Create a test environment"""
    environment = Environment()
    locator = GeneralResourceLocator()
    environment.init(SIMPLE_URDF, locator)
    return environment


class TestCommandImports:
    """Test that all command classes can be imported"""

    def test_command_base_class(self):
        assert Command is not None

    def test_modify_allowed_collisions_type_enum(self):
        assert ModifyAllowedCollisionsType.ADD == ModifyAllowedCollisionsType_ADD
        assert ModifyAllowedCollisionsType.REMOVE == ModifyAllowedCollisionsType_REMOVE
        assert ModifyAllowedCollisionsType.REPLACE == ModifyAllowedCollisionsType_REPLACE


class TestAddLinkCommand:
    """Tests for AddLinkCommand"""

    def test_constructor_link_only(self):
        link = Link("test_link")
        cmd = AddLinkCommand(link)
        assert cmd.getLink().getName() == "test_link"
        assert cmd.getJoint() is None
        assert not cmd.replaceAllowed()

    def test_constructor_with_joint(self):
        link = Link("test_link")
        joint = Joint("test_joint")
        joint.type = JointType.FIXED
        joint.parent_link_name = "world"
        joint.child_link_name = "test_link"
        cmd = AddLinkCommand(link, joint)
        assert cmd.getLink().getName() == "test_link"
        assert cmd.getJoint().getName() == "test_joint"

    def test_apply_command(self, env):
        link = Link("new_link")
        box = Box(0.05, 0.05, 0.05)
        visual = Visual()
        visual.geometry = box
        link.visual.append(visual)

        joint = Joint("new_joint")
        joint.type = JointType.FIXED
        joint.parent_link_name = "link1"
        joint.child_link_name = "new_link"

        cmd = AddLinkCommand(link, joint)
        assert "new_link" not in env.getLinkNames()
        env.applyCommand(cmd)
        assert "new_link" in env.getLinkNames()


class TestRemoveLinkCommand:
    """Tests for RemoveLinkCommand"""

    def test_constructor(self):
        cmd = RemoveLinkCommand("test_link")
        assert cmd.getLinkName() == "test_link"

    def test_apply_command(self, env):
        # First add a link
        link = Link("temp_link")
        joint = Joint("temp_joint")
        joint.type = JointType.FIXED
        joint.parent_link_name = "link2"
        joint.child_link_name = "temp_link"
        env.applyCommand(AddLinkCommand(link, joint))
        assert "temp_link" in env.getLinkNames()

        # Then remove it
        cmd = RemoveLinkCommand("temp_link")
        env.applyCommand(cmd)
        assert "temp_link" not in env.getLinkNames()


class TestRemoveJointCommand:
    """Tests for RemoveJointCommand"""

    def test_constructor(self):
        cmd = RemoveJointCommand("test_joint")
        assert cmd.getJointName() == "test_joint"


class TestChangeJointPositionLimitsCommand:
    """Tests for ChangeJointPositionLimitsCommand"""

    def test_constructor_single_joint(self):
        cmd = ChangeJointPositionLimitsCommand("joint1", -3.14, 3.14)
        limits = cmd.getLimits()
        assert "joint1" in limits
        assert limits["joint1"] == (-3.14, 3.14)

    def test_constructor_multiple_joints(self):
        limits_dict = {"joint1": (-2.0, 2.0), "joint2": (-1.5, 1.5)}
        cmd = ChangeJointPositionLimitsCommand(limits_dict)
        limits = cmd.getLimits()
        assert limits["joint1"] == (-2.0, 2.0)
        assert limits["joint2"] == (-1.5, 1.5)

    def test_apply_command(self, env):
        cmd = ChangeJointPositionLimitsCommand("joint1", -3.14, 3.14)
        env.applyCommand(cmd)
        # Command applied successfully (no exception)


class TestChangeJointVelocityLimitsCommand:
    """Tests for ChangeJointVelocityLimitsCommand"""

    def test_constructor_single_joint(self):
        cmd = ChangeJointVelocityLimitsCommand("joint1", 2.0)
        limits = cmd.getLimits()
        assert "joint1" in limits
        assert limits["joint1"] == 2.0

    def test_constructor_multiple_joints(self):
        limits_dict = {"joint1": 2.5, "joint2": 3.0}
        cmd = ChangeJointVelocityLimitsCommand(limits_dict)
        limits = cmd.getLimits()
        assert limits["joint1"] == 2.5
        assert limits["joint2"] == 3.0

    def test_apply_command(self, env):
        cmd = ChangeJointVelocityLimitsCommand("joint1", 2.0)
        env.applyCommand(cmd)


class TestChangeJointAccelerationLimitsCommand:
    """Tests for ChangeJointAccelerationLimitsCommand"""

    def test_constructor_single_joint(self):
        cmd = ChangeJointAccelerationLimitsCommand("joint1", 5.0)
        limits = cmd.getLimits()
        assert "joint1" in limits
        assert limits["joint1"] == 5.0

    def test_apply_command(self, env):
        cmd = ChangeJointAccelerationLimitsCommand("joint1", 5.0)
        env.applyCommand(cmd)


class TestChangeLinkCollisionEnabledCommand:
    """Tests for ChangeLinkCollisionEnabledCommand"""

    def test_constructor(self):
        cmd = ChangeLinkCollisionEnabledCommand("link1", False)
        assert cmd.getLinkName() == "link1"
        assert cmd.getEnabled() == False

    def test_apply_command(self, env):
        cmd = ChangeLinkCollisionEnabledCommand("link1", False)
        env.applyCommand(cmd)


class TestChangeLinkVisibilityCommand:
    """Tests for ChangeLinkVisibilityCommand"""

    def test_constructor(self):
        cmd = ChangeLinkVisibilityCommand("link1", False)
        assert cmd.getLinkName() == "link1"
        assert cmd.getEnabled() == False

    def test_apply_command(self, env):
        cmd = ChangeLinkVisibilityCommand("link1", False)
        env.applyCommand(cmd)


class TestModifyAllowedCollisionsCommand:
    """Tests for ModifyAllowedCollisionsCommand"""

    def test_constructor(self):
        acm = AllowedCollisionMatrix()
        acm.addAllowedCollision("link1", "link2", "Adjacent")
        cmd = ModifyAllowedCollisionsCommand(acm, ModifyAllowedCollisionsType.ADD)
        assert cmd.getModifyType() == ModifyAllowedCollisionsType.ADD

    def test_apply_command(self, env):
        acm = AllowedCollisionMatrix()
        acm.addAllowedCollision("link1", "link2", "TestReason")
        cmd = ModifyAllowedCollisionsCommand(acm, ModifyAllowedCollisionsType.ADD)
        env.applyCommand(cmd)


class TestRemoveAllowedCollisionLinkCommand:
    """Tests for RemoveAllowedCollisionLinkCommand"""

    def test_constructor(self):
        cmd = RemoveAllowedCollisionLinkCommand("link1")
        assert cmd.getLinkName() == "link1"

    def test_apply_command(self, env):
        cmd = RemoveAllowedCollisionLinkCommand("link1")
        env.applyCommand(cmd)


class TestChangeCollisionMarginsCommand:
    """Tests for ChangeCollisionMarginsCommand"""

    def test_constructor_with_default_margin(self):
        cmd = ChangeCollisionMarginsCommand(0.01, CollisionMarginOverrideType.OVERRIDE_DEFAULT_MARGIN)
        assert cmd.getCollisionMarginOverrideType() == CollisionMarginOverrideType.OVERRIDE_DEFAULT_MARGIN

    def test_constructor_with_margin_data(self):
        margin_data = CollisionMarginData(0.02)
        cmd = ChangeCollisionMarginsCommand(margin_data, CollisionMarginOverrideType.REPLACE)
        assert cmd.getCollisionMarginOverrideType() == CollisionMarginOverrideType.REPLACE

    def test_apply_command(self, env):
        cmd = ChangeCollisionMarginsCommand(0.01, CollisionMarginOverrideType.OVERRIDE_DEFAULT_MARGIN)
        env.applyCommand(cmd)


class TestChangeJointOriginCommand:
    """Tests for ChangeJointOriginCommand"""

    def test_constructor(self):
        origin = Isometry3d.Identity()
        cmd = ChangeJointOriginCommand("joint1", origin)
        assert cmd.getJointName() == "joint1"

    def test_apply_command(self, env):
        origin = Isometry3d.Identity()
        cmd = ChangeJointOriginCommand("joint1", origin)
        env.applyCommand(cmd)


class TestChangeLinkOriginCommand:
    """Tests for ChangeLinkOriginCommand"""

    def test_constructor(self):
        origin = Isometry3d.Identity()
        cmd = ChangeLinkOriginCommand("link1", origin)
        assert cmd.getLinkName() == "link1"

    @pytest.mark.xfail(reason="CHANGE_LINK_ORIGIN not implemented in tesseract C++ Environment")
    def test_apply_command(self, env):
        origin = Isometry3d.Identity()
        cmd = ChangeLinkOriginCommand("link1", origin)
        env.applyCommand(cmd)


class TestMoveJointCommand:
    """Tests for MoveJointCommand"""

    def test_constructor(self):
        cmd = MoveJointCommand("joint2", "world")
        assert cmd.getJointName() == "joint2"
        assert cmd.getParentLink() == "world"


class TestMoveLinkCommand:
    """Tests for MoveLinkCommand"""

    def test_constructor(self):
        joint = Joint("move_joint")
        joint.type = JointType.FIXED
        joint.parent_link_name = "world"
        joint.child_link_name = "link2"
        cmd = MoveLinkCommand(joint)
        assert cmd.getJoint().getName() == "move_joint"


class TestReplaceJointCommand:
    """Tests for ReplaceJointCommand"""

    def test_constructor(self):
        joint = Joint("joint1")
        joint.type = JointType.FIXED
        joint.parent_link_name = "world"
        joint.child_link_name = "link1"
        cmd = ReplaceJointCommand(joint)
        assert cmd.getJoint().getName() == "joint1"

    def test_apply_command(self, env):
        joint = Joint("joint1")
        joint.type = JointType.FIXED
        joint.parent_link_name = "world"
        joint.child_link_name = "link1"
        cmd = ReplaceJointCommand(joint)
        env.applyCommand(cmd)
