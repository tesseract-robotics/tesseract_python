"""
Collision Checking Example (High-Level API)

Demonstrates collision checking using the high-level planning API.
Shows how to:
- Load robot with Robot.from_tesseract_support()
- Create geometry with sphere() helper
- Add links with proper joint transforms
- Check collisions at different robot configurations

Simplified version of tesseract_collision_example.py using high-level API.
"""

import numpy as np
from tesseract_robotics.planning import Robot, Pose, sphere, create_fixed_joint
from tesseract_robotics.tesseract_scene_graph import Link, Visual, Collision
from tesseract_robotics.tesseract_collision import (
    ContactResultMap,
    ContactTestType_ALL,
    ContactRequest,
    ContactResultVector,
)
from tesseract_robotics.tesseract_common import CollisionMarginData


def main():
    # Load robot using high-level API
    robot = Robot.from_tesseract_support("abb_irb2400")

    # Create sphere link with visual and collision geometry
    sphere_link = Link("sphere_link")
    sphere_geom = sphere(0.1)

    visual = Visual()
    visual.geometry = sphere_geom
    sphere_link.visual.append(visual)

    collision = Collision()
    collision.geometry = sphere_geom
    sphere_link.collision.append(collision)

    # Create fixed joint at desired position using high-level API
    sphere_joint = create_fixed_joint(
        name="sphere_joint",
        parent_link="base_link",
        child_link="sphere_link",
        origin=Pose.from_xyz(0.7, 0, 1.5),
    )

    robot.add_link(sphere_link, sphere_joint)

    # Get discrete contact manager
    manager = robot.env.getDiscreteContactManager()
    manager.setActiveCollisionObjects(robot.env.getActiveLinkNames())
    manager.setCollisionMarginData(CollisionMarginData(0.1))  # 10cm margin

    # Robot joint configuration
    joint_names = [f"joint_{i+1}" for i in range(6)]
    joint_pos = np.zeros(6)

    # Check collisions at different configurations
    for i in range(-5, 5):
        joint_pos[0] = i * np.deg2rad(5)
        print(f"Contact check at robot position: {joint_pos}")

        # Update robot state
        robot.set_joints(joint_pos, joint_names=joint_names)
        scene_state = robot.env.getState()
        manager.setCollisionObjectsTransform(scene_state.link_transforms)

        # Print link poses
        print(f"Link 6 Pose:\n{scene_state.link_transforms['link_6'].matrix()}")
        print(f"Sphere Link Pose:\n{scene_state.link_transforms['sphere_link'].matrix()}")

        # Execute collision check
        contact_result_map = ContactResultMap()
        manager.contactTest(contact_result_map, ContactRequest(ContactTestType_ALL))
        result_vector = ContactResultVector()
        contact_result_map.flattenMoveResults(result_vector)

        # Print results
        print(f"Found {len(result_vector)} contact results")
        for j in range(len(result_vector)):
            contact_result = result_vector[j]
            print(f"Contact {j}:")
            print(f"\tDistance: {contact_result.distance}")
            print(f"\tLink A: {contact_result.link_names[0]}")
            print(f"\tLink B: {contact_result.link_names[1]}")
        print()


if __name__ == "__main__":
    main()
