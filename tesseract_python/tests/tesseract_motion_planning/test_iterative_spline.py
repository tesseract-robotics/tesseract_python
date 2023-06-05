from tesseract_robotics.tesseract_command_language import CompositeInstruction, StateWaypoint, WaypointPoly, \
    InstructionPoly, MoveInstruction, Instructions, MoveInstructionType_FREESPACE, StateWaypointPoly, \
     MoveInstructionPoly, InstructionPoly_as_MoveInstructionPoly, WaypointPoly_as_StateWaypointPoly, \
     MoveInstructionPoly_wrap_MoveInstruction, StateWaypointPoly_wrap_StateWaypoint
from tesseract_robotics.tesseract_time_parameterization import IterativeSplineParameterization, \
    InstructionsTrajectory
import numpy as np

def create_straight_trajectory():

    num = 10
    max_ = 2.0
    joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]

    program = CompositeInstruction()

    for i in range(num):
        p = np.zeros((6,),dtype=np.float64)
        p[0] = i * (max_/num)
        swp = StateWaypoint(joint_names, p)        
        program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(MoveInstruction(
                                        StateWaypointPoly_wrap_StateWaypoint(swp), 
                                        MoveInstructionType_FREESPACE)))

    p = np.zeros((6,),dtype=np.float64)
    p[0] = max_
    swp = StateWaypoint(joint_names, p)
    program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(MoveInstruction(
                                    StateWaypointPoly_wrap_StateWaypoint(swp), 
                                    MoveInstructionType_FREESPACE)))

    return program

def test_time_parameterization():

    time_parameterization = IterativeSplineParameterization(False)

    program = create_straight_trajectory()
    traj = InstructionsTrajectory(program)
    max_velocity = np.array([2.088, 2.082, 3.27, 3.6, 3.3, 3.078],dtype=np.float64)
    max_acceleration = np.array([ 1, 1, 1, 1, 1, 1],dtype=np.float64)
    assert time_parameterization.compute(traj, max_velocity, max_acceleration)
    res_instr1 = InstructionPoly_as_MoveInstructionPoly(program[-1])
    WaypointPoly_as_StateWaypointPoly(res_instr1.getWaypoint()).getTime() > 1.0
    res_instr2 = InstructionPoly_as_MoveInstructionPoly(program[-1])
    WaypointPoly_as_StateWaypointPoly(res_instr2.getWaypoint()).getTime() < 5.0

def test_time_parameterization_vec():

    time_parameterization = IterativeSplineParameterization(False)

    program = create_straight_trajectory()
    traj = InstructionsTrajectory(program)
    max_velocity = np.array([2.088, 2.082, 3.27, 3.6, 3.3, 3.078],dtype=np.float64)
    max_acceleration = np.array([ 1, 1, 1, 1, 1, 1],dtype=np.float64)
    assert time_parameterization.compute(traj, max_velocity, max_acceleration)
    res_instr1 = InstructionPoly_as_MoveInstructionPoly(program[-1])
    WaypointPoly_as_StateWaypointPoly(res_instr1.getWaypoint()).getTime() > 1.0
    res_instr2 = InstructionPoly_as_MoveInstructionPoly(program[-1])
    WaypointPoly_as_StateWaypointPoly(res_instr2.getWaypoint()).getTime() < 5.0
