from tesseract_robotics import tesseract_kinematics
from tesseract_robotics import tesseract_scene_graph
from tesseract_robotics import tesseract_urdf
from tesseract_robotics import tesseract_common
from tesseract_robotics import tesseract_state_solver

import re
import os
import traceback
import numpy as np
import numpy.testing as nptest

from ..tesseract_support_resource_locator import TesseractSupportResourceLocator

def get_scene_graph():
    tesseract_support = os.environ["TESSERACT_SUPPORT_DIR"]
    path =  os.path.join(tesseract_support, "urdf/lbr_iiwa_14_r820.urdf")
    locator = TesseractSupportResourceLocator()
    return tesseract_urdf.parseURDFFile(path, locator).release()


def get_plugin_factory():
    support_dir = os.environ["TESSERACT_SUPPORT_DIR"]
    kin_config = tesseract_common.FilesystemPath(support_dir + "/urdf/" + "lbr_iiwa_14_r820_plugins.yaml")
    locator = TesseractSupportResourceLocator()
    return tesseract_kinematics.KinematicsPluginFactory(kin_config, locator), locator


def run_inv_kin_test(inv_kin, fwd_kin):

     pose = np.eye(4)
     pose[2,3] = 1.306

     seed = np.array([-0.785398, 0.785398, -0.785398, 0.785398, -0.785398, 0.785398, -0.785398])
     tip_pose = tesseract_common.TransformMap()
     tip_pose["tool0"] = tesseract_common.Isometry3d(pose)
     solutions = inv_kin.calcInvKin(tip_pose,seed)
     assert len(solutions) > 0

     result = fwd_kin.calcFwdKin(solutions[0])
    
     nptest.assert_almost_equal(pose,result["tool0"].matrix(),decimal=3)

def test_kdl_kin_chain_lma_inverse_kinematic():
    plugin_factory, p_locator = get_plugin_factory()
    scene_graph = get_scene_graph()
    solver = tesseract_state_solver.KDLStateSolver(scene_graph)
    scene_state1 = solver.getState(np.zeros((7,)))
    scene_state2 = solver.getState(np.zeros((7,)))
    inv_kin = plugin_factory.createInvKin("manipulator","KDLInvKinChainLMA",scene_graph,scene_state1)
    fwd_kin = plugin_factory.createFwdKin("manipulator","KDLFwdKinChain",scene_graph,scene_state2)

    assert inv_kin
    assert fwd_kin

    run_inv_kin_test(inv_kin, fwd_kin)

    del inv_kin
    del fwd_kin


def test_jacobian():
    plugin_factory, p_locator = get_plugin_factory()
    scene_graph = get_scene_graph()
    solver = tesseract_state_solver.KDLStateSolver(scene_graph)
    scene_state = solver.getState(np.zeros((7,)))
    fwd_kin = plugin_factory.createFwdKin("manipulator","KDLFwdKinChain",scene_graph,scene_state)
    scene_graph = get_scene_graph()
    
    jvals = np.array([-0.785398, 0.785398, -0.785398, 0.785398, -0.785398, 0.785398, -0.785398])

    link_name = "tool0"
    jacobian = fwd_kin.calcJacobian(jvals,link_name)
    
    assert jacobian.shape == (6,7)

    del fwd_kin

