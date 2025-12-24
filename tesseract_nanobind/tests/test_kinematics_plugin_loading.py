"""Test kinematics plugins load correctly.

These tests verify that kinematics plugins (KDL, OPW) can be loaded and used
via getKinematicGroup().

For editable installs on macOS, the tesseract_robotics package automatically
adds rpath to plugin factory dylibs on first import (one-time fix).
"""
import pytest
from tesseract_robotics.tesseract_common import FilesystemPath, GeneralResourceLocator
from tesseract_robotics.tesseract_environment import Environment


def test_kdl_kinematics_plugin_loads():
    """Test KDL kinematics plugin loads for KUKA robot."""
    locator = GeneralResourceLocator()
    env = Environment()
    urdf = locator.locateResource("package://tesseract_support/urdf/lbr_iiwa_14_r820.urdf").getFilePath()
    srdf = locator.locateResource("package://tesseract_support/urdf/lbr_iiwa_14_r820.srdf").getFilePath()
    assert env.init(FilesystemPath(urdf), FilesystemPath(srdf), locator)

    # This should not raise RuntimeError if plugins load correctly
    kin_group = env.getKinematicGroup("manipulator")
    assert kin_group is not None


def test_opw_kinematics_plugin_loads():
    """Test OPW kinematics plugin loads for ABB robot."""
    locator = GeneralResourceLocator()
    env = Environment()
    urdf = locator.locateResource("package://tesseract_support/urdf/abb_irb2400.urdf").getFilePath()
    srdf = locator.locateResource("package://tesseract_support/urdf/abb_irb2400.srdf").getFilePath()
    assert env.init(FilesystemPath(urdf), FilesystemPath(srdf), locator)

    # This should not raise RuntimeError if plugins load correctly
    kin_group = env.getKinematicGroup("manipulator")
    assert kin_group is not None
