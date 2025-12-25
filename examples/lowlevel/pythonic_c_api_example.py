#!/usr/bin/env python
"""
Pythonic API Example

Demonstrates the high-level planning API with:
- Robot loading from URDF/SRDF
- Pose helpers and transforms
- Obstacle creation
- Motion program building with fluent API
- Planning with plan_freespace()
"""
import sys
import numpy as np

from tesseract_robotics.planning import (
    Robot,
    MotionProgram,
    CartesianTarget,
    JointTarget,
    Pose,
    translation,
    rotation_z,
    box,
    sphere,
    create_obstacle,
    plan_freespace,
)


def main():
    print("=" * 60)
    print("Pythonic API Example")
    print("=" * 60)

    # =========================================================================
    # 1. Load Robot - Compare with 10+ lines in basic_cartesian_example.py
    # =========================================================================
    print("\n1. Loading robot...")

    robot = Robot.from_tesseract_support("abb_irb2400")

    print(f"   Loaded: {robot}")
    print(f"   Links: {robot.get_link_names()[:5]}...")
    print(f"   Manipulator joints: {robot.get_joint_names('manipulator')}")

    # =========================================================================
    # 2. Robot State - Simple state access
    # =========================================================================
    print("\n2. Accessing robot state...")

    state = robot.get_state()
    print(f"   Current state: {state}")

    # Set joint positions using dict
    robot.set_joints({
        "joint_1": 0.0,
        "joint_2": 0.0,
        "joint_3": 0.0,
        "joint_4": 0.0,
        "joint_5": 0.0,
        "joint_6": 0.0,
    })
    print("   Set joints to zero position")

    # =========================================================================
    # 3. Forward Kinematics - One-liner
    # =========================================================================
    print("\n3. Forward kinematics...")

    pose = robot.fk("manipulator", [0, 0, 0, 0, 0, 0])
    print(f"   FK at zeros: {pose}")
    print(f"   Position: x={pose.x:.3f}, y={pose.y:.3f}, z={pose.z:.3f}")

    # =========================================================================
    # 4. Poses - Clean API
    # =========================================================================
    print("\n4. Pose helpers...")

    # Create pose from position and quaternion
    t1 = Pose.from_xyz_quat(0.5, 0, 0.8, 0, 0, 0.707, 0.707)
    print(f"   From xyz_quat: {t1}")

    # Create with factory functions and chaining
    t2 = translation(0.5, 0, 0.8) @ rotation_z(1.57)
    print(f"   From factories: {t2}")

    # Convert to/from numpy
    print(f"   Position array: {t1.position}")
    print(f"   Quaternion: {t1.quaternion}")

    # =========================================================================
    # 5. Add Obstacles - Compare with 20+ lines in basic_cartesian_example.py
    # =========================================================================
    print("\n5. Adding obstacles...")

    # Add a box obstacle - one function call vs ~20 lines
    create_obstacle(
        robot,
        name="table",
        geometry=box(0.8, 0.8, 0.05),
        transform=Pose.from_xyz(0.5, 0, 0.3),
        color=(0.6, 0.4, 0.2, 1.0),
    )
    print("   Added table obstacle")

    # Add a sphere obstacle
    create_obstacle(
        robot,
        name="ball",
        geometry=sphere(0.1),
        transform=Pose.from_xyz(0.4, 0.2, 0.6),
        color=(1.0, 0.0, 0.0, 1.0),
    )
    print("   Added ball obstacle")

    # =========================================================================
    # 6. Motion Program - Fluent builder API
    # =========================================================================
    print("\n6. Building motion program...")

    # Compare with ~30 lines in basic_cartesian_example.py
    # No manual poly wrapping needed!
    program = (MotionProgram("manipulator", tcp_frame="tool0")
        .set_joint_names(robot.get_joint_names("manipulator"))
        # Start at current position (joints)
        .move_to(JointTarget([0, 0, 0, 0, 0, 0]))
        # Move to Cartesian target
        .move_to(CartesianTarget(
            Pose.from_xyz_quat(0.8, -0.2, 0.8, 0.707, 0, 0.707, 0)
        ))
        # Another Cartesian target
        .move_to(CartesianTarget(
            position=[0.8, 0.2, 0.8],
            quaternion=[0.707, 0, 0.707, 0],
        ))
        # Back to joint target
        .move_to(JointTarget([0, 0, 0, 0, 0, 0]))
    )

    print(f"   Created program with {len(program)} targets")

    # =========================================================================
    # 7. Plan Motion - All AnyPoly wrapping handled automatically
    # =========================================================================
    print("\n7. Planning motion...")

    result = plan_freespace(robot, program)

    if result.successful:
        print(f"   Planning successful!")
        print(f"   Trajectory has {len(result)} waypoints")

        # Access trajectory points
        if result.trajectory:
            print(f"   First point: {result[0].positions}")
            print(f"   Last point: {result[-1].positions}")

            # Convert to numpy for analysis
            trajectory_array = result.to_numpy()
            print(f"   Trajectory shape: {trajectory_array.shape}")
    else:
        print(f"   Planning failed: {result.message}")

    print("\n" + "=" * 60)
    print("Example completed!")
    print("=" * 60)

    return True


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
