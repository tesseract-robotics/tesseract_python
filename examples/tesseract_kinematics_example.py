"""
Kinematics Example (High-Level API)

Demonstrates FK/IK using the high-level planning API.
Uses Robot.from_tesseract_support(), robot.fk(), robot.ik(), and Pose helpers.

Based on: tesseract_kinematics_example.py
"""

import numpy as np
from tesseract_robotics.planning import Robot, Pose


def main():
    # Load robot from tesseract_support - uses KDL solver (no extra config needed)
    robot = Robot.from_tesseract_support("lbr_iiwa_14_r820")

    # === Forward Kinematics ===
    joint_pos = np.array([0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.0])

    # Compute FK using high-level API
    tool0_pose = robot.fk("manipulator", joint_pos, tip_link="tool0")

    print(f"Tool0 transform at joint position {joint_pos}:")
    print(f"Translation: {tool0_pose.position}")
    print(f"Rotation (quat): {tool0_pose.quaternion}")

    # === Inverse Kinematics ===
    # Target pose: scalar-last quaternion (x, y, z, w)
    target_pose = Pose.from_xyz_quat(0.7, -0.1, 1.0, 0.7071, 0, 0.7071, 0)

    # Solve IK with seed state
    ik_solutions = []
    ik_result = robot.ik("manipulator", target_pose, seed=joint_pos, tip_link="tool0")

    if ik_result is not None:
        ik_solutions.append(ik_result)

    # Print results
    print(f"\nFound {len(ik_solutions)} solution(s)")
    for i, sol in enumerate(ik_solutions):
        print(f"Solution {i}: {sol}")


if __name__ == "__main__":
    main()
