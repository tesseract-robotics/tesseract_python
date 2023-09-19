import os

from tesseract_robotics.tesseract_command_language import (
    InstructionPoly_as_MoveInstructionPoly,
    WaypointPoly_as_StateWaypointPoly,
)
from tesseract_robotics.tesseract_common import (
    FilesystemPath,
    ManipulatorInfo,
    GeneralResourceLocator,
)
from tesseract_robotics.tesseract_environment import Environment

TESSERACT_SUPPORT_DIR = os.environ["TESSERACT_RESOURCE_PATH"]
TESSERACT_TASK_COMPOSER_DIR = os.environ["TESSERACT_TASK_COMPOSER_CONFIG_FILE"]

task_composer_filename = os.environ["TESSERACT_TASK_COMPOSER_CONFIG_FILE"]


def tesseract_task_composer_config_file():
    # OVERRIDE defaults that has no IPOPT trajopt
    # TODO
    config_path = FilesystemPath(
        "Y:\\CADCAM\\tesseract_planning\\tesseract_task_composer\\config\\task_composer_plugins.yaml"
    )
    return config_path


def support_dir(pth):
    return FilesystemPath(os.path.join(TESSERACT_SUPPORT_DIR, pth))


def compose_dir(pth):
    return FilesystemPath(os.path.join(TESSERACT_TASK_COMPOSER_DIR, pth))


def get_environment(url) -> tuple[Environment, ManipulatorInfo, list[str]]:
    """
    given a `url` load a URDF & SRDF and return an Enviornment and Manipulator instance and a
    list of joint names
    """
    locator = GeneralResourceLocator()
    env = Environment()
    # tesseract_support = os.environ["TESSERACT_SUPPORT_DIR"]
    urdf_path = locator.locateResource(f"{url}.urdf").getFilePath()
    srdf_path = locator.locateResource(f"{url}.srdf").getFilePath()

    urdf_path_str = FilesystemPath(urdf_path)
    srdf_path_str = FilesystemPath(srdf_path)

    assert env.init(urdf_path_str, srdf_path_str, locator)
    manip_info = ManipulatorInfo()
    manip_info.tcp_frame = "tool0"
    manip_info.manipulator = "manipulator"
    manip_info.working_frame = "base_link"
    joint_names = list(env.getJointGroup("manipulator").getJointNames())

    return env, manip_info, joint_names


def print_joints(results):
    # Display the output
    # Print out the resulting waypoints
    for instr in results:
        assert instr.isMoveInstruction()
        move_instr1 = InstructionPoly_as_MoveInstructionPoly(instr)
        wp1 = move_instr1.getWaypoint()
        assert wp1.isStateWaypoint()
        wp = WaypointPoly_as_StateWaypointPoly(wp1)
        print(f"Joint Positions: {wp.getPosition().flatten()} time: {wp.getTime()}")
