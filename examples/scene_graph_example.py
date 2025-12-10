"""
Scene Graph Example

This example demonstrates manipulating the scene graph, including:
- Moving joints between parent links
- Moving links with new joint configurations

Based on: tesseract_examples/src/scene_graph_example.cpp

Required environment variables:
- TESSERACT_RESOURCE_PATH: Path to tesseract repo (for tesseract_support)
"""

import os
import sys
import numpy as np
import math

from tesseract_robotics.tesseract_common import (
    GeneralResourceLocator,
    FilesystemPath,
    Isometry3d,
    Translation3d,
    AngleAxisd,
)
from tesseract_robotics.tesseract_environment import (
    Environment,
    MoveJointCommand,
    MoveLinkCommand,
)
from tesseract_robotics.tesseract_scene_graph import Joint, JointType

# Viewer (skip in pytest)
TesseractViewer = None
if "pytest" not in sys.modules:
    try:
        from tesseract_robotics_viewer import TesseractViewer
    except ImportError:
        pass


def main():
    # Initialize resource locator and environment
    locator = GeneralResourceLocator()

    # Load KUKA IIWA robot
    urdf_url = "package://tesseract_support/urdf/lbr_iiwa_14_r820.urdf"
    srdf_url = "package://tesseract_support/urdf/lbr_iiwa_14_r820.srdf"
    urdf_path = FilesystemPath(locator.locateResource(urdf_url).getFilePath())
    srdf_path = FilesystemPath(locator.locateResource(srdf_url).getFilePath())

    env = Environment()
    if not env.init(urdf_path, srdf_path, locator):
        print("Failed to initialize environment")
        return False

    print(f"Environment initialized: {env.getName()}")
    print(f"Root link: {env.getRootLinkName()}")
    print(f"Links: {list(env.getLinkNames())}")
    print(f"Joints: {list(env.getJointNames())}")

    # Get the scene graph
    scene_graph = env.getSceneGraph()
    print(f"\nScene graph name: {scene_graph.getName()}")

    # Optional: save the initial scene graph to DOT file
    # scene_graph.saveDOT("scene_graph_example_initial.dot")

    # Example 1: Move a joint to a new parent link using MoveJointCommand
    # This changes which link a joint is attached to
    print("\n--- Example 1: MoveJointCommand ---")
    print("Moving joint 'joint_a4' to have parent 'base_link' instead of 'link_3'")

    # Get current parent of joint_a4
    joint = scene_graph.getJoint("joint_a4")
    if joint:
        print(f"  Current parent: {joint.parent_link_name}")
        print(f"  Current child: {joint.child_link_name}")

    # Create and apply MoveJointCommand
    move_joint_cmd = MoveJointCommand("joint_a4", "base_link")
    if env.applyCommand(move_joint_cmd):
        print("  Command applied successfully!")
        # Check new parent
        joint = env.getSceneGraph().getJoint("joint_a4")
        if joint:
            print(f"  New parent: {joint.parent_link_name}")
    else:
        print("  Failed to apply command")

    # Reset environment for next example
    env = Environment()
    env.init(urdf_path, srdf_path, locator)

    # Example 2: Move a link using MoveLinkCommand
    # This creates a new joint to attach a link to a different parent
    print("\n--- Example 2: MoveLinkCommand ---")
    print("Moving link_4 to be attached to link_1 with a new fixed joint")

    # Create a new joint for the move operation
    new_joint = Joint("moved_link_joint")
    new_joint.parent_link_name = "link_1"
    new_joint.child_link_name = "link_4"
    new_joint.type = JointType.FIXED

    # Set the transform for the new joint
    # Rotate 90 degrees around Y axis and translate
    transform = Isometry3d.Identity()
    transform = transform * AngleAxisd(-math.pi / 2, np.array([0, 1, 0], dtype=np.float64))
    transform = transform * Translation3d(0.15, 0.0, 0.0)
    new_joint.parent_to_joint_origin_transform = transform

    # Create and apply MoveLinkCommand
    move_link_cmd = MoveLinkCommand(new_joint)
    if env.applyCommand(move_link_cmd):
        print("  Command applied successfully!")
        print(f"  Link 4 now attached to link_1 via joint 'moved_link_joint'")
        # Verify the change
        joint = env.getSceneGraph().getJoint("moved_link_joint")
        if joint:
            print(f"  New joint parent: {joint.parent_link_name}")
            print(f"  New joint child: {joint.child_link_name}")
    else:
        print("  Failed to apply command")

    # Example 3: Query scene graph structure
    print("\n--- Example 3: Scene Graph Queries ---")
    print(f"Number of links: {scene_graph.getLinks().__len__()}")
    print(f"Number of joints: {scene_graph.getJoints().__len__()}")

    # Get adjacent links
    root_link = env.getRootLinkName()
    adjacent = scene_graph.getAdjacentLinkNames(root_link)
    print(f"\nLinks adjacent to '{root_link}': {list(adjacent)}")

    # Get child links
    print("\nKinematic chain from root:")
    current_link = root_link
    depth = 0
    visited = set()
    stack = [(root_link, 0)]

    while stack:
        link_name, depth = stack.pop()
        if link_name in visited:
            continue
        visited.add(link_name)
        print(f"  {'  ' * depth}{link_name}")

        # Get child links through joints
        for joint in scene_graph.getJoints():
            if joint.parent_link_name == link_name:
                stack.append((joint.child_link_name, depth + 1))

    # Optional: visualize with viewer
    if TesseractViewer is not None:
        print("\nStarting viewer at http://localhost:8000")
        viewer = TesseractViewer()
        viewer.update_environment(env, [0, 0, 0])
        viewer.start_serve_background()
        input("Press Enter to exit...")

    return True


if __name__ == "__main__":
    success = main()
    exit(0 if success else 1)
