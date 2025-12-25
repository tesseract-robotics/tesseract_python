from tesseract_robotics.tesseract_common import FilesystemPath, GeneralResourceLocator, Isometry3d, Translation3d, \
    TransformMap, Quaterniond
from tesseract_robotics.tesseract_environment import Environment
from tesseract_robotics.tesseract_kinematics import KinGroupIKInput, KinGroupIKInputs
import numpy as np

# Example of using kinematics to solve for forward and inverse kinematics. A tesseract environment is created
# using URDF and SRDF files. The kinematics solver is configured using the SRDF file and plugin configuration files.

# Initialize Environment with a robot from URDF file
# The URDF and SRDF file must be configured. The kinematics solver also requires plugin configuration,
# which is specified in the SRDF file. For this example, the plugin configuration file is `abb_irb2400_plugins.yaml`
# and is located in the same directory as the SRDF file. This example uses the OPW kinematics solver, which is
# a solver for industrial 6-dof robots with spherical wrists. The kinematic parameters for the robot must
# be specified in the plugin configuration file in addition to the URDF file for the plugin to work.
# The other main solver is the KDL solver, which is used by the lbr_iiwa_14_r820 robot also included in the
# tesseract_support package. The KDL solver is a numerical solver and does not require additional configuration.

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

    # Get the kinematics solver. The name "manipulator" is specified in the SRDF file
    kin_group = env.getKinematicGroup("manipulator")

    # Solve forward kinematics at a specific joint position
    robot_joint_pos = np.deg2rad(np.array([10, 20,-5, 70, 30, 90], dtype=np.float64))
    fwdkin_result = kin_group.calcFwdKin(robot_joint_pos)
    #fwdkin_result is a TransformMap, which is a dictionary of link names to Isometry3d. For this robot, we are
    #interested in the transform of the "tool0" link
    tool0_transform = fwdkin_result["tool0"]
    # Print the transform as a translation and quaternion
    print("Tool0 transform at joint position " + str(robot_joint_pos) + " is: ")
    q = Quaterniond(tool0_transform.rotation())
    print("Translation: " + str(tool0_transform.translation().flatten()))
    print(f"Rotation: {q.w()} {q.x()} {q.y()} {q.z()}")

    # Solve inverse kinematics at a specific tool0 pose
    tool0_transform2 = Isometry3d.Identity() * Translation3d(0.7, -0.1, 1) * Quaterniond(0.70711, 0, 0.7171, 0)

    # Create a KinGroupIKInput and KinGroupIKInputs object. The KinGroupIKInputs object is a list of KinGroupIKInput
    ik = KinGroupIKInput()
    ik.pose = tool0_transform2
    ik.tip_link_name = "tool0"
    ik.working_frame = "base_link"
    iks = KinGroupIKInputs()
    iks.append(ik)
    # Solve IK
    ik_result = kin_group.calcInvKin(iks, robot_joint_pos)
    # Print the result
    print(f"Found {len(ik_result)} solutions")
    for i in range(len(ik_result)):
        print("Solution " + str(i) + ": " + str(ik_result[i].flatten()))


if __name__ == "__main__":
    main()
