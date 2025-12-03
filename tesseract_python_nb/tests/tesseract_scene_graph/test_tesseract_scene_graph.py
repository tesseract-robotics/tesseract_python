import traceback
import tesseract_robotics.tesseract_scene_graph as sg
from tesseract_robotics import tesseract_common
from tesseract_robotics import tesseract_srdf
import numpy as np
import re
import os
from ..tesseract_support_resource_locator import TesseractSupportResourceLocator

def _translation(p):
    H = np.eye(4)
    H[0:3,3] = p
    return tesseract_common.Isometry3d(H)

def test_tesseract_scene_graph():
    g = sg.SceneGraph()
    assert g.addLink(sg.Link("base_link"))
    assert g.addLink(sg.Link("link_1"))
    assert g.addLink(sg.Link("link_2"))
    assert g.addLink(sg.Link("link_3"))
    assert g.addLink(sg.Link("link_4"))
    assert g.addLink(sg.Link("link_5"))

    base_joint = sg.Joint("base_joint")
    base_joint.parent_link_name = "base_link"
    base_joint.child_link_name = "link_1"
    base_joint.type = sg.JointType_FIXED
    assert g.addJoint(base_joint)

    joint_1 = sg.Joint("joint_1")
    joint_1.parent_link_name = "link_1"
    joint_1.child_link_name = "link_2"
    joint_1.type = sg.JointType_FIXED
    assert g.addJoint(joint_1)

    joint_2 = sg.Joint("joint_2")
    joint_2.parent_to_joint_origin_transform = _translation([1.25,0,0])
    joint_2.parent_link_name = "link_2"
    joint_2.child_link_name = "link_3"
    joint_2.type = sg.JointType_PLANAR
    joint_2.limits = sg.JointLimits(-1,1,1,1,1,1)
    assert g.addJoint(joint_2)

    joint_3 = sg.Joint("joint_3")
    joint_3.parent_to_joint_origin_transform = _translation([1.25,0,0])
    joint_3.parent_link_name = "link_3"
    joint_3.child_link_name = "link_4"
    joint_3.type = sg.JointType_FLOATING
    assert g.addJoint(joint_3)

    joint_4 = sg.Joint("joint_4")
    joint_4.parent_to_joint_origin_transform = _translation([0,1.25,0])
    joint_4.parent_link_name = "link_2"
    joint_4.child_link_name = "link_5"
    joint_4.type = sg.JointType_REVOLUTE
    joint_4.limits = sg.JointLimits(-1,1,1,1,1,1)
    assert g.addJoint(joint_4)

    adjacent_links = g.getAdjacentLinkNames("link_3")
    assert len(adjacent_links) == 1
    assert adjacent_links[0] == "link_4"

    inv_adjacent_links = g.getInvAdjacentLinkNames("link_3")
    assert len(inv_adjacent_links) == 1
    assert inv_adjacent_links[0] == "link_2"

    child_link_names = g.getLinkChildrenNames("link_5")
    assert len(child_link_names) == 0

    child_link_names = g.getLinkChildrenNames("link_3")
    assert len(child_link_names) == 1
    assert child_link_names[0] == "link_4"

    child_link_names = g.getLinkChildrenNames("link_2")
    assert len(child_link_names) == 3
    assert "link_3" in child_link_names
    assert "link_4" in child_link_names
    assert "link_5" in child_link_names

    child_link_names = g.getJointChildrenNames("joint_4")
    assert len(child_link_names) == 1
    assert child_link_names[0] == "link_5"

    child_link_names = g.getJointChildrenNames("joint_3")
    assert len(child_link_names) == 1
    assert child_link_names[0] == "link_4"

    child_link_names = g.getJointChildrenNames("joint_1")
    assert len(child_link_names) == 4
    assert "link_2" in child_link_names
    assert "link_3" in child_link_names
    assert "link_4" in child_link_names
    assert "link_5" in child_link_names

    assert g.isAcyclic()
    assert g.isTree()

    g.addLink(sg.Link("link_6"))
    assert not g.isTree()

    g.removeLink("link_6")
    assert g.isTree()

    joint_5 = sg.Joint("joint_5")
    joint_5.parent_to_joint_origin_transform = _translation([0,1.5,0])
    joint_5.parent_link_name = "link_5"
    joint_5.child_link_name = "link_4"
    joint_5.type = sg.JointType_CONTINUOUS
    g.addJoint(joint_5)

    assert g.isAcyclic()
    assert not g.isTree()

    joint_6 = sg.Joint("joint_6")
    joint_6.parent_to_joint_origin_transform = _translation([0,1.25,0])
    joint_6.parent_link_name = "link_5"
    joint_6.child_link_name = "link_1"
    joint_6.type = sg.JointType_CONTINUOUS
    g.addJoint(joint_6)

    assert not g.isAcyclic()
    assert not g.isTree()

    path = g.getShortestPath("link_1", "link_4")
    
    assert len(path.links) == 4
    assert "link_1" in path.links
    assert "link_2" in path.links
    assert "link_3" in path.links
    assert "link_4" in path.links
    assert len(path.joints) == 3
    assert "joint_1" in path.joints
    assert "joint_2" in path.joints
    assert "joint_3" in path.joints

    print(g.getName())

def test_load_srdf_unit():
    tesseract_support = os.environ["TESSERACT_SUPPORT_DIR"]
    srdf_file =  os.path.join(tesseract_support, "urdf/lbr_iiwa_14_r820.srdf")

    locator = TesseractSupportResourceLocator()

    g = sg.SceneGraph()

    g.setName("kuka_lbr_iiwa_14_r820")

    assert g.addLink(sg.Link("base_link"))
    assert g.addLink(sg.Link("link_1"))
    assert g.addLink(sg.Link("link_2"))
    assert g.addLink(sg.Link("link_3"))
    assert g.addLink(sg.Link("link_4"))
    assert g.addLink(sg.Link("link_5"))
    assert g.addLink(sg.Link("link_6"))
    assert g.addLink(sg.Link("link_7"))
    assert g.addLink(sg.Link("tool0"))
    
    joint_1 = sg.Joint("joint_a1")
    joint_1.parent_link_name = "base_link"
    joint_1.child_link_name = "link_1"
    joint_1.type = sg.JointType_FIXED
    assert g.addJoint(joint_1)

    joint_2 = sg.Joint("joint_a2")
    joint_2.parent_link_name = "link_1"
    joint_2.child_link_name = "link_2"
    joint_2.type = sg.JointType_REVOLUTE
    joint_2.limits = sg.JointLimits(-1,1,1,1,1,1)
    assert g.addJoint(joint_2)

    joint_3 = sg.Joint("joint_a3")
    joint_3.parent_to_joint_origin_transform = _translation([1.25,0,0])
    joint_3.parent_link_name = "link_2"
    joint_3.child_link_name = "link_3"
    joint_3.type = sg.JointType_REVOLUTE
    joint_3.limits = sg.JointLimits(-1,1,1,1,1,1)
    assert g.addJoint(joint_3)

    joint_4 = sg.Joint("joint_a4")
    joint_4.parent_to_joint_origin_transform = _translation([1.25,0,0])
    joint_4.parent_link_name = "link_3"
    joint_4.child_link_name = "link_4"
    joint_4.type = sg.JointType_REVOLUTE
    joint_4.limits = sg.JointLimits(-1,1,1,1,1,1)
    assert g.addJoint(joint_4)

    joint_5 = sg.Joint("joint_a5")
    joint_5.parent_to_joint_origin_transform = _translation([0,1.25,0])
    joint_5.parent_link_name = "link_4"
    joint_5.child_link_name = "link_5"
    joint_5.type = sg.JointType_REVOLUTE
    joint_5.limits = sg.JointLimits(-1,1,1,1,1,1)
    assert g.addJoint(joint_5)

    joint_6 = sg.Joint("joint_a6")
    joint_6.parent_to_joint_origin_transform = _translation([0,1.25,0])
    joint_6.parent_link_name = "link_5"
    joint_6.child_link_name = "link_6"
    joint_6.type = sg.JointType_REVOLUTE
    joint_6.limits = sg.JointLimits(-1,1,1,1,1,1)
    assert g.addJoint(joint_6)

    joint_7 = sg.Joint("joint_a7")
    joint_7.parent_to_joint_origin_transform = _translation([0,1.25,0])
    joint_7.parent_link_name = "link_6"
    joint_7.child_link_name = "link_7"
    joint_7.type = sg.JointType_REVOLUTE
    joint_7.limits = sg.JointLimits(-1,1,1,1,1,1)
    assert g.addJoint(joint_7)

    joint_tool0 = sg.Joint("base_joint")
    joint_tool0.parent_link_name = "link_7"
    joint_tool0.child_link_name = "tool0"
    joint_tool0.type = sg.JointType_FIXED
    assert g.addJoint(joint_tool0)

    srdf = tesseract_srdf.SRDFModel()
    srdf.initFile(g,srdf_file,locator)

    tesseract_srdf.processSRDFAllowedCollisions(g, srdf)

    acm = g.getAllowedCollisionMatrix()

    assert acm.isCollisionAllowed("link_1", "link_2")
    assert not acm.isCollisionAllowed("base_link", "link_5")

    g.removeAllowedCollision("link_1", "link_2")

    assert not acm.isCollisionAllowed("link_1", "link_2")

    g.clearAllowedCollisions()
    assert len(acm.getAllAllowedCollisions()) == 0
