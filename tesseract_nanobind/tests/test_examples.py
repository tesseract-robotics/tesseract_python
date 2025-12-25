"""Tests for example script functionality

These tests verify that the basic components used in examples work correctly.
Full planning tests are skipped if TESSERACT_TASK_COMPOSER_CONFIG_FILE is not set.
"""

import os
import pytest
import numpy as np

from tesseract_robotics.tesseract_common import (
    GeneralResourceLocator,
    FilesystemPath,
    Isometry3d,
    Translation3d,
    ManipulatorInfo,
)
from tesseract_robotics.tesseract_environment import (
    Environment,
    AddLinkCommand,
    MoveJointCommand,
    MoveLinkCommand,
)
from tesseract_robotics.tesseract_scene_graph import (
    Link,
    Joint,
    JointType,
    Visual,
    Collision,
)
from tesseract_robotics.tesseract_geometry import Sphere, Box
from tesseract_robotics.tesseract_command_language import (
    CompositeInstruction,
    MoveInstruction,
    MoveInstructionType_FREESPACE,
    MoveInstructionType_LINEAR,
    StateWaypoint,
    CartesianWaypoint,
    StateWaypointPoly_wrap_StateWaypoint,
    CartesianWaypointPoly_wrap_CartesianWaypoint,
    MoveInstructionPoly_wrap_MoveInstruction,
    ProfileDictionary,
)


@pytest.fixture
def iiwa_env():
    """Create a KUKA IIWA environment for testing"""
    locator = GeneralResourceLocator()
    urdf_url = "package://tesseract_support/urdf/lbr_iiwa_14_r820.urdf"
    srdf_url = "package://tesseract_support/urdf/lbr_iiwa_14_r820.srdf"

    urdf_path = FilesystemPath(locator.locateResource(urdf_url).getFilePath())
    srdf_path = FilesystemPath(locator.locateResource(srdf_url).getFilePath())

    env = Environment()
    assert env.init(urdf_path, srdf_path, locator)
    return env


class TestFreespaceOMPLExample:
    """Tests for freespace_ompl_example components"""

    def test_add_sphere_obstacle(self, iiwa_env):
        """Test adding a sphere obstacle to the environment"""
        link_sphere = Link("sphere_attached")

        visual = Visual()
        visual.origin = Isometry3d.Identity()
        visual.origin = visual.origin * Translation3d(0.5, 0, 0.55)
        visual.geometry = Sphere(0.15)
        link_sphere.visual.append(visual)

        collision = Collision()
        collision.origin = visual.origin
        collision.geometry = visual.geometry
        link_sphere.collision.append(collision)

        joint_sphere = Joint("joint_sphere_attached")
        joint_sphere.parent_link_name = "base_link"
        joint_sphere.child_link_name = link_sphere.getName()
        joint_sphere.type = JointType.FIXED

        cmd = AddLinkCommand(link_sphere, joint_sphere)
        assert iiwa_env.applyCommand(cmd)
        assert "sphere_attached" in iiwa_env.getLinkNames()

    def test_create_freespace_program(self, iiwa_env):
        """Test creating a freespace planning program"""
        joint_names = [f"joint_a{i}" for i in range(1, 8)]
        joint_start_pos = np.array([-0.4, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0])
        joint_end_pos = np.array([0.4, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0])

        # Set initial state
        iiwa_env.setState(joint_names, joint_start_pos)

        # Create manipulator info
        manip_info = ManipulatorInfo()
        manip_info.manipulator = "manipulator"
        manip_info.working_frame = "base_link"
        manip_info.tcp_frame = "tool0"

        # Create program
        program = CompositeInstruction("FREESPACE")
        program.setManipulatorInfo(manip_info)

        # Create waypoints
        wp0 = StateWaypoint(joint_names, joint_start_pos)
        wp1 = StateWaypoint(joint_names, joint_end_pos)

        # Create instructions
        start_instruction = MoveInstruction(
            StateWaypointPoly_wrap_StateWaypoint(wp0),
            MoveInstructionType_FREESPACE,
            "FREESPACE"
        )
        plan_f0 = MoveInstruction(
            StateWaypointPoly_wrap_StateWaypoint(wp1),
            MoveInstructionType_FREESPACE,
            "FREESPACE"
        )

        program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(start_instruction))
        program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(plan_f0))

        assert len(program) == 2


class TestBasicCartesianExample:
    """Tests for basic_cartesian_example components"""

    def test_add_box_obstacle(self, iiwa_env):
        """Test adding a box obstacle to the environment"""
        link_box = Link("box_obstacle")

        visual = Visual()
        visual.origin = Isometry3d.Identity()
        visual.origin = visual.origin * Translation3d(1.0, 0, 0)
        visual.geometry = Box(0.5, 0.5, 0.5)
        link_box.visual.append(visual)

        collision = Collision()
        collision.origin = visual.origin
        collision.geometry = visual.geometry
        link_box.collision.append(collision)

        joint_box = Joint("joint_box_obstacle")
        joint_box.parent_link_name = "base_link"
        joint_box.child_link_name = link_box.getName()
        joint_box.type = JointType.FIXED

        cmd = AddLinkCommand(link_box, joint_box)
        assert iiwa_env.applyCommand(cmd)
        assert "box_obstacle" in iiwa_env.getLinkNames()

    def test_create_cartesian_program(self, iiwa_env):
        """Test creating a program with Cartesian waypoints"""
        joint_names = [f"joint_a{i}" for i in range(1, 8)]
        joint_pos = np.array([-0.4, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0])

        iiwa_env.setState(joint_names, joint_pos)

        manip_info = ManipulatorInfo()
        manip_info.manipulator = "manipulator"
        manip_info.working_frame = "base_link"
        manip_info.tcp_frame = "tool0"

        program = CompositeInstruction("cartesian_program")
        program.setManipulatorInfo(manip_info)

        # Start waypoint (joint position)
        wp0 = StateWaypoint(joint_names, joint_pos)

        # Cartesian waypoint
        from tesseract_robotics.tesseract_common import Quaterniond
        wp1 = CartesianWaypoint(
            Isometry3d.Identity() * Translation3d(0.5, -0.2, 0.62) * Quaterniond(0, 0, 1.0, 0)
        )

        start_instruction = MoveInstruction(
            StateWaypointPoly_wrap_StateWaypoint(wp0),
            MoveInstructionType_FREESPACE,
            "freespace_profile"
        )
        plan_f0 = MoveInstruction(
            CartesianWaypointPoly_wrap_CartesianWaypoint(wp1),
            MoveInstructionType_LINEAR,
            "RASTER"
        )

        program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(start_instruction))
        program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(plan_f0))

        assert len(program) == 2


class TestSceneGraphExample:
    """Tests for scene_graph_example components"""

    def test_move_joint_command(self, iiwa_env):
        """Test moving a joint to a new parent using MoveJointCommand"""
        # Get current parent
        scene_graph = iiwa_env.getSceneGraph()
        joint = scene_graph.getJoint("joint_a4")
        assert joint is not None
        original_parent = joint.parent_link_name

        # Move joint to different parent
        move_joint_cmd = MoveJointCommand("joint_a4", "base_link")
        assert iiwa_env.applyCommand(move_joint_cmd)

        # Verify change
        joint = iiwa_env.getSceneGraph().getJoint("joint_a4")
        assert joint.parent_link_name == "base_link"

    def test_move_link_command(self, iiwa_env):
        """Test moving a link with a new joint using MoveLinkCommand"""
        import math
        from tesseract_robotics.tesseract_common import AngleAxisd

        # Create new joint
        new_joint = Joint("moved_link_joint")
        new_joint.parent_link_name = "link_1"
        new_joint.child_link_name = "link_4"
        new_joint.type = JointType.FIXED

        transform = Isometry3d.Identity()
        # AngleAxisd requires numpy array for axis
        transform = transform * AngleAxisd(-math.pi / 2, np.array([0, 1, 0], dtype=np.float64))
        transform = transform * Translation3d(0.15, 0.0, 0.0)
        new_joint.parent_to_joint_origin_transform = transform

        move_link_cmd = MoveLinkCommand(new_joint)
        assert iiwa_env.applyCommand(move_link_cmd)

        # Verify the new joint exists
        joint = iiwa_env.getSceneGraph().getJoint("moved_link_joint")
        assert joint is not None
        assert joint.parent_link_name == "link_1"
        assert joint.child_link_name == "link_4"

    def test_scene_graph_queries(self, iiwa_env):
        """Test scene graph query methods"""
        scene_graph = iiwa_env.getSceneGraph()

        # Query methods
        assert scene_graph.getName() is not None
        assert len(scene_graph.getLinks()) > 0
        assert len(scene_graph.getJoints()) > 0

        # Get adjacent links
        root_link = iiwa_env.getRootLinkName()
        adjacent = scene_graph.getAdjacentLinkNames(root_link)
        assert len(adjacent) > 0


class TestGlassUprightExample:
    """Tests for glass_upright_example components"""

    def test_create_linear_program(self, iiwa_env):
        """Test creating a program with LINEAR motion type"""
        joint_names = [f"joint_a{i}" for i in range(1, 8)]
        joint_start_pos = np.array([-0.4, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0])
        joint_end_pos = np.array([0.4, 0.2762, 0.0, -1.3348, 0.0, 1.4959, 0.0])

        iiwa_env.setState(joint_names, joint_start_pos)

        manip_info = ManipulatorInfo()
        manip_info.manipulator = "manipulator"
        manip_info.working_frame = "base_link"
        manip_info.tcp_frame = "tool0"

        program = CompositeInstruction("UPRIGHT")
        program.setManipulatorInfo(manip_info)

        wp0 = StateWaypoint(joint_names, joint_start_pos)
        wp1 = StateWaypoint(joint_names, joint_end_pos)

        # Use LINEAR motion type
        start_instruction = MoveInstruction(
            StateWaypointPoly_wrap_StateWaypoint(wp0),
            MoveInstructionType_LINEAR,
            "UPRIGHT"
        )
        plan_f0 = MoveInstruction(
            StateWaypointPoly_wrap_StateWaypoint(wp1),
            MoveInstructionType_LINEAR,
            "UPRIGHT"
        )

        program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(start_instruction))
        program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(plan_f0))

        assert len(program) == 2


class TestPuzzlePieceExample:
    """Tests for puzzle_piece_example components"""

    def test_load_csv_toolpath(self):
        """Test loading the puzzle_bent.csv toolpath file"""
        locator = GeneralResourceLocator()
        resource = locator.locateResource("package://tesseract_support/urdf/puzzle_bent.csv")
        csv_path = resource.getFilePath()

        assert os.path.exists(csv_path)

        # Read and parse CSV
        import csv
        poses_count = 0
        with open(csv_path, 'r') as f:
            reader = csv.reader(f)
            for lnum, row in enumerate(reader):
                if lnum < 2:  # Skip headers
                    continue
                if len(row) >= 7:
                    poses_count += 1

        assert poses_count > 0, "Should have parsed some poses from CSV"

    def test_create_cartesian_toolpath_program(self, iiwa_env):
        """Test creating a program from toolpath poses"""
        from tesseract_robotics.tesseract_common import Quaterniond

        manip_info = ManipulatorInfo()
        manip_info.manipulator = "manipulator"
        manip_info.working_frame = "base_link"
        manip_info.tcp_frame = "tool0"

        program = CompositeInstruction("DEFAULT")
        program.setManipulatorInfo(manip_info)

        # Create some test poses (simplified from actual toolpath)
        test_poses = [
            Isometry3d.Identity() * Translation3d(0.4, 0, 0.5),
            Isometry3d.Identity() * Translation3d(0.4, 0.1, 0.5),
            Isometry3d.Identity() * Translation3d(0.4, 0.2, 0.5),
        ]

        for i, pose in enumerate(test_poses):
            wp = CartesianWaypoint(pose)
            plan_instruction = MoveInstruction(
                CartesianWaypointPoly_wrap_CartesianWaypoint(wp),
                MoveInstructionType_LINEAR,
                "CARTESIAN"
            )
            plan_instruction.setDescription(f"waypoint_{i}")
            program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(plan_instruction))

        assert len(program) == 3


class TestProfileDictionary:
    """Tests for ProfileDictionary used in examples"""

    def test_create_profile_dictionary(self):
        """Test creating an empty profile dictionary"""
        profiles = ProfileDictionary()
        assert profiles is not None

    def test_profile_dictionary_in_planning(self, iiwa_env):
        """Test using profile dictionary with planning data"""
        profiles = ProfileDictionary()

        # Profiles can be used even if empty - defaults will be used
        from tesseract_robotics.tesseract_command_language import AnyPoly_wrap_ProfileDictionary
        profiles_any = AnyPoly_wrap_ProfileDictionary(profiles)
        assert profiles_any is not None


# ============================================================================
# Tests that run the actual example scripts
# ============================================================================

import subprocess
import sys
from pathlib import Path

# tests/test_examples.py -> tesseract_nanobind/tests -> tesseract_nanobind -> repo root
EXAMPLES_DIR = Path(__file__).parent.parent.parent / "examples"
LOWLEVEL_EXAMPLES_DIR = EXAMPLES_DIR / "lowlevel"


def get_env_with_vars():
    """Get current environment with proper vars for examples"""
    import copy
    env = copy.copy(os.environ)
    return env


class TestCollisionExample:
    """Test tesseract_collision_example.py"""

    def test_collision_example_runs(self):
        """Run the collision example and verify it completes successfully"""
        script = EXAMPLES_DIR / "tesseract_collision_example.py"
        assert script.exists(), f"Example script not found: {script}"

        result = subprocess.run(
            [sys.executable, str(script)],
            capture_output=True,
            text=True,
            timeout=60,
            env=get_env_with_vars(),
        )
        # Check exit code
        assert result.returncode == 0, f"Script failed:\n{result.stderr}"
        # Verify some expected output
        assert "Contact check at robot position" in result.stdout
        assert "Found" in result.stdout and "contact results" in result.stdout


class TestKinematicsExample:
    """Test tesseract_kinematics_example.py"""

    def test_kinematics_example_runs(self):
        """Run the kinematics example and verify it completes successfully"""
        script = EXAMPLES_DIR / "tesseract_kinematics_example.py"
        assert script.exists(), f"Example script not found: {script}"

        result = subprocess.run(
            [sys.executable, str(script)],
            capture_output=True,
            text=True,
            timeout=60,
            env=get_env_with_vars(),
        )
        # Check exit code
        assert result.returncode == 0, f"Script failed:\n{result.stderr}"
        # Verify some expected output (high-level API format)
        assert "Tool0 transform" in result.stdout
        assert "Translation:" in result.stdout
        assert "solution" in result.stdout.lower()


class TestFreespaceOMPLExampleRun:
    """Test running the actual freespace_ompl_example.py script"""

    @pytest.fixture
    def has_task_composer_config(self):
        """Check if task composer config is available"""
        config = os.environ.get("TESSERACT_TASK_COMPOSER_CONFIG_FILE")
        return config and os.path.exists(config)

    def test_freespace_ompl_example_runs(self, has_task_composer_config):
        """Run the freespace OMPL example end-to-end"""
        if not has_task_composer_config:
            pytest.skip("TESSERACT_TASK_COMPOSER_CONFIG_FILE not set or file not found")

        script = EXAMPLES_DIR / "freespace_ompl_example.py"
        assert script.exists(), f"Example script not found: {script}"

        result = subprocess.run(
            [sys.executable, str(script)],
            capture_output=True,
            text=True,
            timeout=120,
            env=get_env_with_vars(),
        )
        # Verify expected output (nanobind leak warnings may cause exit=1)
        assert "Loaded robot" in result.stdout, f"Missing 'Loaded robot':\n{result.stdout}"
        assert "Added sphere obstacle" in result.stdout
        assert "Running OMPL planner" in result.stdout
        assert "Planning successful" in result.stdout, f"Planning failed:\n{result.stdout}\n{result.stderr}"
        assert "waypoints" in result.stdout


class TestBasicCartesianExampleRun:
    """Test running the actual basic_cartesian_example.py script"""

    @pytest.fixture
    def has_task_composer_config(self):
        """Check if task composer config is available"""
        config = os.environ.get("TESSERACT_TASK_COMPOSER_CONFIG_FILE")
        return config and os.path.exists(config)

    def test_basic_cartesian_example_runs(self, has_task_composer_config):
        """Run the basic Cartesian example end-to-end"""
        if not has_task_composer_config:
            pytest.skip("TESSERACT_TASK_COMPOSER_CONFIG_FILE not set or file not found")

        script = EXAMPLES_DIR / "basic_cartesian_example.py"
        assert script.exists(), f"Example script not found: {script}"

        result = subprocess.run(
            [sys.executable, str(script)],
            capture_output=True,
            text=True,
            timeout=120,
            env=get_env_with_vars(),
        )
        # Verify expected output (nanobind leak warnings may cause exit=1)
        assert "Planning successful" in result.stdout, f"Planning failed:\n{result.stdout}\n{result.stderr}"
        assert "waypoints" in result.stdout.lower()


class TestSceneGraphExampleRun:
    """Test running the actual scene_graph_example.py script (lowlevel only)"""

    def test_scene_graph_example_runs(self):
        """Run the scene graph example end-to-end"""
        script = LOWLEVEL_EXAMPLES_DIR / "scene_graph_c_api_example.py"
        assert script.exists(), f"Example script not found: {script}"

        result = subprocess.run(
            [sys.executable, str(script)],
            capture_output=True,
            text=True,
            timeout=60,
            env=get_env_with_vars(),
        )
        # Check that expected output is present (nanobind leak warnings may cause exit=1)
        assert "Environment initialized" in result.stdout
        assert "Scene graph name" in result.stdout
        assert "MoveJointCommand" in result.stdout
        assert "MoveLinkCommand" in result.stdout
        assert "Command applied successfully" in result.stdout


class TestGlassUprightExampleRun:
    """Test running the actual glass_upright_example.py script"""

    @pytest.fixture
    def has_task_composer_config(self):
        """Check if task composer config is available"""
        config = os.environ.get("TESSERACT_TASK_COMPOSER_CONFIG_FILE")
        return config and os.path.exists(config)

    def test_glass_upright_example_runs(self, has_task_composer_config):
        """Run the glass upright example end-to-end"""
        if not has_task_composer_config:
            pytest.skip("TESSERACT_TASK_COMPOSER_CONFIG_FILE not set or file not found")

        script = EXAMPLES_DIR / "glass_upright_example.py"
        assert script.exists(), f"Example script not found: {script}"

        result = subprocess.run(
            [sys.executable, str(script)],
            capture_output=True,
            text=True,
            timeout=120,
            env=get_env_with_vars(),
        )
        # Verify expected output (nanobind leak warnings may cause exit=1)
        assert "Loaded robot" in result.stdout, f"Missing 'Loaded robot':\n{result.stdout}"
        assert "Planning successful" in result.stdout, f"Planning failed:\n{result.stdout}\n{result.stderr}"


