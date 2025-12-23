"""Tests for the Pythonic planning API."""
import os
import numpy as np
import pytest

from tesseract_robotics.planning import (
    Robot,
    RobotState,
    Pose,
    translation,
    rotation_x,
    rotation_y,
    rotation_z,
    rotation_from_quaternion,
    MotionProgram,
    CartesianTarget,
    JointTarget,
    StateTarget,
    MoveType,
    box,
    sphere,
    cylinder,
    create_obstacle,
)


class TestPose:
    """Test Pose class and helpers."""

    def test_identity(self):
        t = Pose.identity()
        np.testing.assert_array_almost_equal(t.position, [0, 0, 0])
        np.testing.assert_array_almost_equal(t.quaternion, [0, 0, 0, 1])

    def test_from_xyz(self):
        t = Pose.from_xyz(1, 2, 3)
        assert t.x == 1
        assert t.y == 2
        assert t.z == 3
        np.testing.assert_array_almost_equal(t.position, [1, 2, 3])

    def test_from_position(self):
        t = Pose.from_position([1.5, 2.5, 3.5])
        np.testing.assert_array_almost_equal(t.position, [1.5, 2.5, 3.5])

    def test_from_xyz_quat(self):
        # 90 degree rotation around Z
        t = Pose.from_xyz_quat(1, 2, 3, 0, 0, 0.707, 0.707)
        np.testing.assert_array_almost_equal(t.position, [1, 2, 3])
        np.testing.assert_array_almost_equal(t.quaternion, [0, 0, 0.707, 0.707], decimal=3)

    def test_from_xyz_rpy(self):
        # 90 degrees around Z
        t = Pose.from_xyz_rpy(1, 2, 3, 0, 0, np.pi/2)
        np.testing.assert_array_almost_equal(t.position, [1, 2, 3])
        roll, pitch, yaw = t.rpy
        np.testing.assert_almost_equal(yaw, np.pi/2, decimal=5)

    def test_translation_helper(self):
        t = translation(1, 2, 3)
        np.testing.assert_array_almost_equal(t.position, [1, 2, 3])

    def test_rotation_x(self):
        t = rotation_x(np.pi/2)
        # Should rotate Y to Z
        y_vec = np.array([0, 1, 0])
        rotated = t.rotation_matrix @ y_vec
        np.testing.assert_array_almost_equal(rotated, [0, 0, 1], decimal=5)

    def test_rotation_y(self):
        t = rotation_y(np.pi/2)
        # Should rotate Z to X
        z_vec = np.array([0, 0, 1])
        rotated = t.rotation_matrix @ z_vec
        np.testing.assert_array_almost_equal(rotated, [1, 0, 0], decimal=5)

    def test_rotation_z(self):
        t = rotation_z(np.pi/2)
        # Should rotate X to Y
        x_vec = np.array([1, 0, 0])
        rotated = t.rotation_matrix @ x_vec
        np.testing.assert_array_almost_equal(rotated, [0, 1, 0], decimal=5)

    def test_pose_chaining(self):
        t1 = translation(1, 0, 0)
        t2 = translation(0, 2, 0)
        combined = t1 @ t2
        np.testing.assert_array_almost_equal(combined.position, [1, 2, 0])

    def test_pose_inverse(self):
        t = Pose.from_xyz(1, 2, 3)
        inv = t.inverse()
        combined = t @ inv
        np.testing.assert_array_almost_equal(combined.position, [0, 0, 0], decimal=5)

    def test_to_isometry(self):
        t = Pose.from_xyz(1, 2, 3)
        iso = t.to_isometry()
        assert iso is not None
        # Round-trip
        t2 = Pose.from_isometry(iso)
        np.testing.assert_array_almost_equal(t.position, t2.position)


class TestRobot:
    """Test Robot loading and state management."""

    @pytest.fixture
    def robot(self):
        """Load test robot."""
        return Robot.from_tesseract_support("abb_irb2400")

    def test_load_robot(self, robot):
        assert robot is not None
        assert len(robot.get_link_names()) > 0

    def test_get_joint_names(self, robot):
        joints = robot.get_joint_names("manipulator")
        assert len(joints) == 6
        assert "joint_1" in joints

    def test_get_state(self, robot):
        state = robot.get_state()
        assert isinstance(state, RobotState)
        assert len(state.joint_names) > 0
        assert len(state.joint_positions) == len(state.joint_names)

    def test_set_joints_dict(self, robot):
        robot.set_joints({"joint_1": 0.5, "joint_2": -0.3})
        state = robot.get_state(["joint_1", "joint_2"])
        np.testing.assert_almost_equal(state["joint_1"], 0.5)
        np.testing.assert_almost_equal(state["joint_2"], -0.3)

    def test_set_joints_array(self, robot):
        joints = robot.get_joint_names("manipulator")
        values = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
        robot.set_joints(values, joint_names=joints)
        state = robot.get_state(joints)
        np.testing.assert_array_almost_equal(state.joint_positions, values)

    def test_fk(self, robot):
        pose = robot.fk("manipulator", [0, 0, 0, 0, 0, 0])
        assert isinstance(pose, Pose)
        # ABB IRB2400 tool0 at zeros should be around x=0.94, z=1.455
        assert pose.x > 0.9
        assert pose.z > 1.4

    def test_get_manipulator_info(self, robot):
        info = robot.get_manipulator_info("manipulator")
        assert info.manipulator == "manipulator"
        assert info.tcp_frame is not None


class TestMotionProgram:
    """Test MotionProgram builder."""

    def test_create_empty(self):
        program = MotionProgram("manipulator")
        assert len(program) == 0

    def test_add_joint_target(self):
        program = MotionProgram("manipulator")
        program.move_to(JointTarget([0, 0, 0, 0, 0, 0]))
        assert len(program) == 1

    def test_add_cartesian_target(self):
        program = MotionProgram("manipulator")
        program.move_to(CartesianTarget(Pose.from_xyz(0.5, 0, 0.5)))
        assert len(program) == 1

    def test_fluent_api(self):
        program = (MotionProgram("manipulator")
            .move_to(JointTarget([0, 0, 0, 0, 0, 0]))
            .move_to(CartesianTarget(Pose.from_xyz(0.5, 0, 0.5)))
            .linear_to(CartesianTarget(Pose.from_xyz(0.5, 0.2, 0.5)))
            .move_to(JointTarget([0.5, 0, 0, 0, 0, 0]))
        )
        assert len(program) == 4

    def test_to_composite_instruction(self):
        joint_names = ["j1", "j2", "j3", "j4", "j5", "j6"]
        program = (MotionProgram("manipulator", tcp_frame="tool0")
            .set_joint_names(joint_names)
            .move_to(JointTarget([0, 0, 0, 0, 0, 0]))
            .move_to(JointTarget([0.5, 0, 0, 0, 0, 0]))
        )
        composite = program.to_composite_instruction()
        assert composite is not None
        assert len(composite) == 2


class TestTargets:
    """Test target types."""

    def test_joint_target(self):
        target = JointTarget([0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
        assert target.move_type == MoveType.FREESPACE
        np.testing.assert_array_equal(target.positions, [0.1, 0.2, 0.3, 0.4, 0.5, 0.6])

    def test_joint_target_with_names(self):
        names = ["j1", "j2", "j3"]
        target = JointTarget([0.1, 0.2, 0.3], names=names)
        wp = target.to_waypoint()
        assert wp is not None

    def test_cartesian_target_from_pose(self):
        pose = Pose.from_xyz(1, 2, 3)
        target = CartesianTarget(pose)
        assert target.pose.x == 1
        assert target.pose.y == 2
        assert target.pose.z == 3

    def test_cartesian_target_from_position(self):
        target = CartesianTarget(position=[1, 2, 3])
        assert target.pose.x == 1
        assert target.pose.y == 2
        assert target.pose.z == 3

    def test_cartesian_target_from_position_quaternion(self):
        target = CartesianTarget(
            position=[1, 2, 3],
            quaternion=[0, 0, 0.707, 0.707],
        )
        np.testing.assert_array_almost_equal(target.pose.position, [1, 2, 3])

    def test_state_target(self):
        target = StateTarget(
            positions=[0.1, 0.2, 0.3],
            names=["j1", "j2", "j3"],
            velocities=[1.0, 2.0, 3.0],
            accelerations=[0.5, 0.5, 0.5],
            time=0.5,
        )
        wp = target.to_waypoint()
        assert wp is not None
        # Check values are set correctly
        np.testing.assert_array_almost_equal(wp.getPosition(), [0.1, 0.2, 0.3])
        np.testing.assert_array_almost_equal(wp.getVelocity(), [1.0, 2.0, 3.0])
        np.testing.assert_array_almost_equal(wp.getAcceleration(), [0.5, 0.5, 0.5])
        assert wp.getTime() == 0.5


class TestGeometry:
    """Test geometry helpers."""

    def test_box(self):
        b = box(1, 2, 3)
        assert b is not None

    def test_sphere(self):
        s = sphere(0.5)
        assert s is not None

    def test_cylinder(self):
        c = cylinder(0.5, 1.0)
        assert c is not None


class TestCreateObstacle:
    """Test obstacle creation."""

    @pytest.fixture
    def robot(self):
        return Robot.from_tesseract_support("abb_irb2400")

    def test_create_box_obstacle(self, robot):
        result = create_obstacle(
            robot,
            name="test_box",
            geometry=box(0.5, 0.5, 0.5),
            transform=Pose.from_xyz(0.5, 0, 0.3),
        )
        assert result is True
        assert "test_box" in robot.get_link_names()

    def test_create_sphere_obstacle(self, robot):
        result = create_obstacle(
            robot,
            name="test_sphere",
            geometry=sphere(0.1),
            transform=Pose.from_xyz(0.3, 0.2, 0.6),
            color=(1.0, 0, 0, 1.0),
        )
        assert result is True
        assert "test_sphere" in robot.get_link_names()


class TestPlanningIntegration:
    """Integration tests for planning (require task composer)."""

    @pytest.fixture
    def robot(self):
        return Robot.from_tesseract_support("abb_irb2400")

    @pytest.mark.xfail(reason="macOS RTTI issue: std::type_index differs across DSOs for same type")
    def test_plan_trajopt(self, robot):
        """Test TrajOpt planning through TaskComposer.

        Note: On macOS, TaskComposer planning fails due to std::type_index
        mismatch across shared libraries. The AnyPoly type check fails because
        typeid(shared_ptr<const Environment>) gives different values in the
        Python bindings vs the tesseract_task_composer_planning_nodes library.
        """
        composer_config = os.environ.get("TESSERACT_TASK_COMPOSER_CONFIG_FILE")
        if not composer_config:
            pytest.skip("TESSERACT_TASK_COMPOSER_CONFIG_FILE not set")

        from tesseract_robotics.planning import plan_trajopt

        joint_names = robot.get_joint_names("manipulator")
        program = (MotionProgram("manipulator", tcp_frame="tool0")
            .set_joint_names(joint_names)
            .move_to(JointTarget([0, 0, 0, 0, 0, 0]))
            .move_to(JointTarget([0.5, 0, 0, 0, 0, 0]))
        )

        result = plan_trajopt(robot, program)

        assert result.successful
        assert len(result) > 0
        assert result.trajectory[0].positions is not None
