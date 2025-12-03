import os
import numpy as np

from tesseract_robotics.tesseract_environment import Environment
from tesseract_robotics.tesseract_common import FilesystemPath, ManipulatorInfo
from tesseract_robotics.tesseract_kinematics import KinGroupIKInput, KinGroupIKInputs, getRedundantSolutions
from ..tesseract_support_resource_locator import TesseractSupportResourceLocator

def get_environment():
    env = Environment()
    locator = TesseractSupportResourceLocator()
    tesseract_support = os.environ["TESSERACT_SUPPORT_DIR"]
    urdf_path = FilesystemPath(os.path.join(tesseract_support, "urdf/abb_irb2400.urdf"))
    srdf_path = FilesystemPath(os.path.join(tesseract_support, "urdf/abb_irb2400.srdf"))
    assert env.init(urdf_path, srdf_path, locator)
    manip_info = ManipulatorInfo()
    manip_info.manipulator = "manipulator"
    manip_info.tcp_frame = "tool0"
    manip_info.working_frame = "base_link"
    joint_names = list(env.getJointGroup("manipulator").getJointNames())

    # Return locator to keep it alive (prevent segfault on gc)
    return env, manip_info, joint_names, locator

def test_get_environment():
    import gc
    env, manip_info, joint_names, locator = get_environment()
    # Explicit cleanup
    del joint_names, manip_info, env, locator
    gc.collect()

def test_kinematic_group():
    import gc

    env, manip_info, joint_names, locator = get_environment()

    kin_group = env.getKinematicGroup(manip_info.manipulator)

    joint_vals = np.ones((6,),dtype=np.float64)*0.1
    pose_map = kin_group.calcFwdKin(joint_vals)
    pose = pose_map[manip_info.tcp_frame]

    ik = KinGroupIKInput()
    ik.pose = pose
    ik.tip_link_name = "tool0"
    ik.working_frame = "base_link"
    iks = KinGroupIKInputs()
    iks.append(ik)

    invkin1 = kin_group.calcInvKin(iks,joint_vals*0.1)
    invkin = invkin1[0]

    np.testing.assert_allclose(invkin.flatten(),joint_vals)

    # Explicit cleanup to prevent segfault from gc order issues
    del invkin, invkin1, iks, ik, pose, pose_map, kin_group
    del joint_names, manip_info, env, locator
    gc.collect()

def test_kinematic_info():
    import gc
    import pytest

    env, manip_info, joint_names, locator = get_environment()

    # Skip if getKinematicsInformation is not yet bound
    if not hasattr(env, 'getKinematicsInformation'):
        pytest.skip("getKinematicsInformation not yet bound")

    kin_info = env.getKinematicsInformation()

    assert list(kin_info.joint_groups) == []
    assert list(kin_info.link_groups) == []

    # Explicit cleanup
    del kin_info, joint_names, manip_info, env, locator
    gc.collect()

def test_tesseract_redundant_solutions_tesseract_function():
    import gc

    env, manip_info, joint_names, locator = get_environment()

    kin_group = env.getKinematicGroup(manip_info.manipulator)

    limits = kin_group.getLimits()
    redundancy_indices = list(kin_group.getRedundancyCapableJointIndices())

    import tesseract_robotics.tesseract_kinematics as tes_com
    sol = np.ones(6)*np.deg2rad(5)  # 1D array, not 2D
    redun_sol = tes_com.getRedundantSolutions(sol, limits.joint_limits, redundancy_indices)

    assert len(redun_sol) == 2

    assert np.allclose(redun_sol[0].flatten(),
        np.array([0.08726646, 0.08726646,  0.08726646, 0.08726646, 0.08726646, -6.19591884])) or \
            np.allclose(redun_sol[0].flatten(),
        np.array([0.08726646, 0.08726646,  0.08726646, 0.08726646, 0.08726646, 6.19591884]))

    # Explicit cleanup
    del redun_sol, sol, redundancy_indices, limits, kin_group
    del joint_names, manip_info, env, locator
    gc.collect()

