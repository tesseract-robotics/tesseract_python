from tesseract_robotics.tesseract_common import JointTrajectory
from tesseract_robotics.tesseract_command_language import CompositeInstruction, InstructionPoly_as_MoveInstructionPoly, \
    WaypointPoly_as_StateWaypointPoly



def tesseract_trajectory_to_list(tesseract_trajectory):
        
    start_instruction_o = tesseract_trajectory[0]
    start_waypoint_m = InstructionPoly_as_MoveInstructionPoly(start_instruction_o)
    start_waypoint_o = start_waypoint_m.getWaypoint()
    assert start_waypoint_o.isStateWaypoint()
    start_waypoint = WaypointPoly_as_StateWaypointPoly(start_waypoint_o)

    joint_names = list(start_waypoint.getNames())
    
    trajectory2 = []
    for i in range(len(tesseract_trajectory)):
        instr = tesseract_trajectory[i]
        instr_m = InstructionPoly_as_MoveInstructionPoly(instr)
        wp = instr_m.getWaypoint()
        wp.isStateWaypoint()
        state_wp = WaypointPoly_as_StateWaypointPoly(wp)
        trajectory2.append(state_wp.getPosition().flatten().tolist() + [state_wp.getTime()])
    
    return trajectory2, joint_names