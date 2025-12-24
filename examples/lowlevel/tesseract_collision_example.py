from tesseract_robotics.tesseract_common import FilesystemPath, GeneralResourceLocator, Isometry3d, Translation3d, \
    CollisionMarginData
from tesseract_robotics.tesseract_environment import Environment, AddLinkCommand
from tesseract_robotics.tesseract_scene_graph import Joint, Link, Visual, Collision, JointType_FIXED
from tesseract_robotics.tesseract_geometry import Sphere
from tesseract_robotics.tesseract_collision import ContactResultMap, ContactTestType_ALL, \
    ContactRequest, ContactResultVector
from tesseract_robotics.tesseract_state_solver import OFKTStateSolver
import numpy as np

# Initialize Environment with a robot from URDF file
# The collision checker is configured using a yaml configuration file specified by the SRDF file. This configuration
# file must be configured for collision checking to work. This example uses the `contact_manager_plugins.yaml` file
# to configure the plugins using Bullet for collision checking. This configuration file can be copied and
# used for most scenes.

# This example uses the GeneralResourceLocator to find resources on the file system. The GeneralResourceLocator
# uses the TESSERACT_RESOURCE_PATH environmental variable.
#
# TESSERACT_RESOURCE_PATH must be set to the directory containing the `tesseract_support` package. This can be done
# by running:
#
# git clone https://github.com/tesseract-robotics/tesseract.git
# export TESSERACT_RESOURCE_PATH="$(pwd)/tesseract/"
#
# or on Windows
#
# git clone https://github.com/tesseract-robotics/tesseract.git
# set TESSERACT_RESOURCE_PATH=%cd%\tesseract\


def main():
    locator = GeneralResourceLocator()
    env = Environment()
    urdf_path_str = locator.locateResource("package://tesseract_support/urdf/abb_irb2400.urdf").getFilePath()
    srdf_path_str = locator.locateResource("package://tesseract_support/urdf/abb_irb2400.srdf").getFilePath()
    urdf_path = FilesystemPath(urdf_path_str)
    srdf_path = FilesystemPath(srdf_path_str)
    assert env.init(urdf_path, srdf_path, locator)

    robot_joint_names = [f"joint_{i+1}" for i in range(6)]
    robot_joint_pos = np.zeros(6)

    # Add a sphere using Environment commands
    sphere_link = Link("sphere_link")
    sphere_link_visual = Visual()
    sphere_link_visual.geometry = Sphere(0.1)
    sphere_link.visual.append(sphere_link_visual)
    sphere_link_collision = Collision()
    sphere_link_collision.geometry = Sphere(0.1)
    sphere_link.collision.append(sphere_link_collision)
    sphere_joint = Joint("sphere_joint")
    sphere_joint.parent_link_name = "base_link"
    sphere_joint.child_link_name = sphere_link.getName()
    sphere_joint.type = JointType_FIXED
    sphere_link_joint_transform = Isometry3d.Identity() * Translation3d(0.7, 0, 1.5)
    sphere_joint.parent_to_joint_origin_transform = sphere_link_joint_transform
    add_sphere_command = AddLinkCommand(sphere_link, sphere_joint)
    env.applyCommand(add_sphere_command)

    # Get the state solver from the scene graph
    # Note: Using OFKTStateSolver directly since getStateSolver() returns type from different module
    scene_graph = env.getSceneGraph()
    solver = OFKTStateSolver(scene_graph)

    # Get the discrete contact manager. This must be called again after the environment is updated
    manager = env.getDiscreteContactManager()
    manager.setActiveCollisionObjects(env.getActiveLinkNames())

    # Set the collision margin for check. Objects with closer than the specified margin will be returned
    margin_data = CollisionMarginData(0.1) # 10cm margin
    manager.setCollisionMarginData(margin_data)

    # Move the robot around and check for collisions
    for i in range(-5, 5):
        robot_joint_pos[0] = i * np.deg2rad(5)
        print("Contact check at robot position: " + str(robot_joint_pos))

        # Set the transform of the active collision objects from SceneState
        solver.setStateByNamesAndValues(robot_joint_names, robot_joint_pos)
        scene_state = solver.getState()
        manager.setCollisionObjectsTransform(scene_state.link_transforms)

        # Print pose of link_6 and sphere_link
        print(f"Link 6 Pose: {scene_state.link_transforms['link_6'].matrix()}")
        print(f"Sphere Link Pose: {scene_state.link_transforms[sphere_link.getName()].matrix()}")

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
   