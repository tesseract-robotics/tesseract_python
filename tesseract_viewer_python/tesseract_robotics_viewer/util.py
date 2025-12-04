from tesseract_robotics.tesseract_common import Quaterniond
from tesseract_robotics.tesseract_command_language import CompositeInstruction, InstructionPoly_as_MoveInstructionPoly, \
    WaypointPoly_as_StateWaypointPoly, WaypointPoly_as_JointWaypointPoly
import json
import numpy as np

def tesseract_trajectory_to_list(tesseract_trajectory):
    """Convert a CompositeInstruction trajectory to a list of joint positions.

    Supports both StateWaypointPoly and JointWaypointPoly waypoints.
    Handles mixed trajectories where some waypoints may be different types.
    """
    # Get joint names from first waypoint that has them
    joint_names = None
    for i in range(len(tesseract_trajectory)):
        instr = tesseract_trajectory[i]
        instr_m = InstructionPoly_as_MoveInstructionPoly(instr)
        wp = instr_m.getWaypoint()

        if wp.isStateWaypoint():
            state_wp = WaypointPoly_as_StateWaypointPoly(wp)
            joint_names = list(state_wp.getNames())
            break
        elif wp.isJointWaypoint():
            joint_wp = WaypointPoly_as_JointWaypointPoly(wp)
            joint_names = list(joint_wp.getNames())
            break

    if joint_names is None:
        raise ValueError("No StateWaypoint or JointWaypoint found in trajectory")

    trajectory2 = []
    for i in range(len(tesseract_trajectory)):
        instr = tesseract_trajectory[i]
        instr_m = InstructionPoly_as_MoveInstructionPoly(instr)
        wp = instr_m.getWaypoint()

        # Handle each waypoint based on its type
        if wp.isStateWaypoint():
            state_wp = WaypointPoly_as_StateWaypointPoly(wp)
            trajectory2.append(state_wp.getPosition().flatten().tolist() + [state_wp.getTime()])
        elif wp.isJointWaypoint():
            joint_wp = WaypointPoly_as_JointWaypointPoly(wp)
            # JointWaypoint doesn't have time, use index as pseudo-time
            trajectory2.append(joint_wp.getPosition().flatten().tolist() + [float(i)])
        else:
            raise ValueError(f"Waypoint {i} must be StateWaypoint or JointWaypoint")

    return joint_names, trajectory2

def trajectory_list_to_json(joint_names, trajectory_list, use_time = True, loop_time = 20):

    trajectory_json = dict()
    trajectory_json["use_time"] = use_time
    trajectory_json["loop_time"] = loop_time
    trajectory_json["joint_names"] = joint_names
    trajectory_json["trajectory"] = trajectory_list
    return json.dumps(trajectory_json)

def joint_positions_to_trajectory_json(joint_names, joint_positions):
    # Create "infinite" animation with constant joint angles

    trajectory_json = dict()
    trajectory_json["use_time"] = True
    trajectory_json["loop_time"] = 10000

    assert joint_names and all(isinstance(s,str) for s in joint_names), "Joint names must all be strings"
    trajectory_json["joint_names"] = joint_names
    assert isinstance(joint_positions,np.ndarray), "Expected numpy array for joint_positions"
    assert joint_positions.dtype == np.float64, "Expected double float array for joint_positions"
    joint_positions = list(joint_positions.flatten())
    assert len(joint_positions) == len(joint_names)
    trajectory_json["trajectory"] = [joint_positions + [0.0], joint_positions + [1e6]]

    return json.dumps(trajectory_json)

def trajectory_list_to_frames(tesseract_env, manipulator_info, joint_names, trajectory_list):
    ret = []
    kin = tesseract_env.getKinematicGroup(manipulator_info.manipulator)
    for i in range(len(trajectory_list)):
        frames = kin.calcFwdKin(np.asarray(trajectory_list[i],dtype=np.float64))
        frame = frames[manipulator_info.tcp_frame]
        p = frame.translation().flatten()
        q1 = Quaterniond(frame.linear())
        q = np.array([q1.w(), q1.x(), q1.y(), q1.z()])

        ret.append([p,q])
    return ret
