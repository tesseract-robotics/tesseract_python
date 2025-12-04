"""
ABB IRB2400 motion planning example with viewer.

This example demonstrates:
- Loading robot from URDF/SRDF
- OMPL motion planning
- TrajOpt trajectory optimization (optional, used if available)
- Time parameterization with TOTG (disabled - see note in code)
- Trajectory visualization
"""

from tesseract_robotics.tesseract_common import FilesystemPath, Isometry3d, Translation3d, Quaterniond, \
    ManipulatorInfo, GeneralResourceLocator
from tesseract_robotics.tesseract_environment import Environment
from tesseract_robotics.tesseract_command_language import CartesianWaypoint, \
    MoveInstructionType_FREESPACE, MoveInstruction, \
    CompositeInstruction, ProfileDictionary, \
    CartesianWaypointPoly_wrap_CartesianWaypoint, MoveInstructionPoly_wrap_MoveInstruction

from tesseract_robotics.tesseract_motion_planners import PlannerRequest
from tesseract_robotics.tesseract_motion_planners_simple import generateInterpolatedProgram
from tesseract_robotics.tesseract_motion_planners_ompl import OMPLMotionPlanner, OMPLRealVectorPlanProfile, \
    ProfileDictionary_addOMPLProfile
from tesseract_robotics.tesseract_time_parameterization import TimeOptimalTrajectoryGeneration, \
    InstructionsTrajectory

# TrajOpt imports - optional, skip if not available
try:
    from tesseract_robotics.tesseract_motion_planners_trajopt import (
        TrajOptDefaultPlanProfile,
        TrajOptDefaultCompositeProfile,
        TrajOptMotionPlanner,
        ProfileDictionary_addTrajOptPlanProfile,
        ProfileDictionary_addTrajOptCompositeProfile,
    )
    TRAJOPT_AVAILABLE = True
except ImportError:
    TRAJOPT_AVAILABLE = False

from tesseract_robotics_viewer import TesseractViewer
import numpy as np
import os
import sys

OMPL_DEFAULT_NAMESPACE = "OMPLMotionPlannerTask"
TRAJOPT_DEFAULT_NAMESPACE = "TrajOptMotionPlannerTask"


def main():
    HEADLESS = os.environ.get("TESSERACT_HEADLESS", "0") == "1" or "pytest" in sys.modules

    # Load robot
    locator = GeneralResourceLocator()
    abb_irb2400_urdf_package_url = "package://tesseract_support/urdf/abb_irb2400.urdf"
    abb_irb2400_srdf_package_url = "package://tesseract_support/urdf/abb_irb2400.srdf"
    abb_irb2400_urdf_fname = FilesystemPath(locator.locateResource(abb_irb2400_urdf_package_url).getFilePath())
    abb_irb2400_srdf_fname = FilesystemPath(locator.locateResource(abb_irb2400_srdf_package_url).getFilePath())

    t_env = Environment()
    assert t_env.init(abb_irb2400_urdf_fname, abb_irb2400_srdf_fname, locator)

    manip_info = ManipulatorInfo()
    manip_info.tcp_frame = "tool0"
    manip_info.manipulator = "manipulator"
    manip_info.working_frame = "base_link"

    viewer = None
    if not HEADLESS:
        viewer = TesseractViewer()
        viewer.update_environment(t_env, [0, 0, 0])

    joint_names = [f"joint_{i+1}" for i in range(6)]
    if viewer:
        viewer.update_joint_positions(joint_names, np.array([1, -0.2, 0.01, 0.3, -0.5, 1]))
        viewer.start_serve_background()

    t_env.setState(joint_names, np.ones(6) * 0.1)

    # Define waypoints
    wp1 = CartesianWaypoint(Isometry3d.Identity() * Translation3d(0.8, -0.3, 1.455) * Quaterniond(0.70710678, 0, 0.70710678, 0))
    wp2 = CartesianWaypoint(Isometry3d.Identity() * Translation3d(0.8, 0.3, 1.455) * Quaterniond(0.70710678, 0, 0.70710678, 0))
    wp3 = CartesianWaypoint(Isometry3d.Identity() * Translation3d(0.8, 0.5, 1.455) * Quaterniond(0.70710678, 0, 0.70710678, 0))

    start_instruction = MoveInstruction(CartesianWaypointPoly_wrap_CartesianWaypoint(wp1), MoveInstructionType_FREESPACE, "DEFAULT")
    plan_f1 = MoveInstruction(CartesianWaypointPoly_wrap_CartesianWaypoint(wp2), MoveInstructionType_FREESPACE, "DEFAULT")
    plan_f2 = MoveInstruction(CartesianWaypointPoly_wrap_CartesianWaypoint(wp3), MoveInstructionType_FREESPACE, "DEFAULT")

    program = CompositeInstruction("DEFAULT")
    program.setManipulatorInfo(manip_info)
    program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(start_instruction))
    program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(plan_f1))

    # OMPL planning
    plan_profile = OMPLRealVectorPlanProfile()
    profiles = ProfileDictionary()
    ProfileDictionary_addOMPLProfile(profiles, OMPL_DEFAULT_NAMESPACE, "DEFAULT", plan_profile)

    request = PlannerRequest()
    request.instructions = program
    request.env = t_env
    request.profiles = profiles

    ompl_planner = OMPLMotionPlanner(OMPL_DEFAULT_NAMESPACE)
    response = ompl_planner.solve(request)
    assert response.successful
    results_instruction = response.results

    interpolated_results_instruction = generateInterpolatedProgram(results_instruction, t_env, 3.14, 1.0, 3.14, 10)

    # TrajOpt trajectory optimization for smooth trajectories
    trajopt_success = False
    if TRAJOPT_AVAILABLE:
        print("TrajOpt available - running trajectory optimization...")
        trajopt_plan_profile = TrajOptDefaultPlanProfile()
        trajopt_composite_profile = TrajOptDefaultCompositeProfile()

        trajopt_profiles = ProfileDictionary()
        # Use helper functions for cross-module profile registration
        ProfileDictionary_addTrajOptPlanProfile(
            trajopt_profiles, TRAJOPT_DEFAULT_NAMESPACE, "DEFAULT", trajopt_plan_profile
        )
        ProfileDictionary_addTrajOptCompositeProfile(
            trajopt_profiles, TRAJOPT_DEFAULT_NAMESPACE, "DEFAULT", trajopt_composite_profile
        )

        trajopt_planner = TrajOptMotionPlanner(TRAJOPT_DEFAULT_NAMESPACE)

        trajopt_request = PlannerRequest()
        trajopt_request.instructions = interpolated_results_instruction
        trajopt_request.env = t_env
        trajopt_request.profiles = trajopt_profiles

        trajopt_response = trajopt_planner.solve(trajopt_request)
        if trajopt_response.successful:
            print("TrajOpt optimization successful!")
            final_results_instruction = trajopt_response.results
            trajopt_success = True
        else:
            print(f"TrajOpt optimization failed: {trajopt_response.message}")
            print("Falling back to interpolated OMPL results")
            final_results_instruction = interpolated_results_instruction
    else:
        print("TrajOpt not available - using interpolated OMPL results")
        final_results_instruction = interpolated_results_instruction

    # Time parameterization - only works with TrajOpt output (StateWaypointPoly)
    # OMPL returns JointWaypointPoly which InstructionsTrajectory doesn't support
    if trajopt_success:
        print("Running time parameterization on TrajOpt results...")
        time_parameterization = TimeOptimalTrajectoryGeneration()
        instructions_trajectory = InstructionsTrajectory(final_results_instruction)
        max_velocity = np.array([2.088, 2.082, 3.27, 3.6, 3.3, 3.078])
        vel_limits = np.column_stack((-max_velocity, max_velocity))
        acc_limits = np.column_stack((-np.ones(6), np.ones(6)))
        jerk_limits = np.column_stack((-np.ones(6), np.ones(6)))
        if time_parameterization.compute(instructions_trajectory, vel_limits, acc_limits, jerk_limits):
            print("Time parameterization successful!")
        else:
            print("Time parameterization failed")

    # Trajectory visualization - util.py now supports both StateWaypointPoly and JointWaypointPoly
    if viewer:
        viewer.update_trajectory(final_results_instruction)
        viewer.plot_trajectory(final_results_instruction, manip_info, axes_length=0.05)

    planner_type = "OMPL + TrajOpt" if trajopt_success else "OMPL"
    print(f"{planner_type} planning completed successfully!")
    print(f"Number of waypoints: {final_results_instruction.size()}")
    if viewer:
        input("Press Enter to exit...")


if __name__ == "__main__":
    main()
