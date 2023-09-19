import time
import os, sys

from compas.geometry import Vector

# this example is transliterated from C++, you'll find the original here:
# https://github.com/tesseract-robotics/tesseract_planning/blob/master/tesseract_examples/src/puzzle_piece_example.cpp
# For some context of this particular example, see ROSCon 2018 Madrid: Optimization Motion
# Planning with Tesseract and TrajOpt for Industrial Applications
# https://vimeo.com/293314190 @ 4:27

# TODO
sys.path.extend(
    ["Y:\CADCAM\tesseract_python\tesseract_viewer_python\tesseract_robotics_viewer"]
)

from tesseract_viewer_python.tesseract_robotics_viewer import TesseractViewer

# TODO
os.environ["TRAJOPT_LOG_THRESH"] = "DEBUG"

import numpy as np
from tesseract_robotics.tesseract_command_language import (
    ProfileDictionary,
    CompositeInstruction,
    CartesianWaypoint,
    MoveInstruction,
    CartesianWaypointPoly_wrap_CartesianWaypoint,
    MoveInstructionPoly_wrap_MoveInstruction,
    MoveInstructionType_LINEAR,
    AnyPoly_wrap_CompositeInstruction,
    AnyPoly_as_CompositeInstruction,
    toJointTrajectory,
)
from tesseract_robotics.tesseract_common import (
    Isometry3d,
    ManipulatorInfo,
    GeneralResourceLocator,
    JointTrajectory,
    AnyPoly,
)
from tesseract_robotics.tesseract_environment import Environment
from tesseract_robotics.tesseract_motion_planners import assignCurrentStateAsSeed
from tesseract_robotics.tesseract_motion_planners_trajopt import (
    TrajOptDefaultPlanProfile,
    TrajOptDefaultCompositeProfile,
    TrajOptDefaultSolverProfile,
    ProfileDictionary_addProfile_TrajOptPlanProfile,
    ProfileDictionary_addProfile_TrajOptCompositeProfile,
    ProfileDictionary_addProfile_TrajOptSolverProfile,
    BasicTrustRegionSQPParameters,
    ModelType,
    CollisionEvaluatorType_SINGLE_TIMESTEP,
)

from tesseract_robotics.tesseract_task_composer import (
    TaskComposerPluginFactory,
    TaskComposerDataStorage,
    TaskComposerInput,
    PlanningTaskComposerProblemUPtr,
    PlanningTaskComposerProblemUPtr_as_TaskComposerProblemUPtr,
)

from utils import get_environment, tesseract_task_composer_config_file

TRAJOPT_DEFAULT_NAMESPACE = "TrajOptMotionPlannerTask"


def create_trajopt_profile() -> ProfileDictionary:
    # Create TrajOpt Profile
    trajopt_plan_profile = TrajOptDefaultPlanProfile()
    trajopt_plan_profile.cartesian_coeff = np.array(
        [10, 10, 10, 10, 10, 0], dtype=np.float64
    )

    trajopt_composite_profile = TrajOptDefaultCompositeProfile()
    trajopt_composite_profile.collision_constraint_config.enabled = False
    trajopt_composite_profile.collision_cost_config.enabled = True
    trajopt_composite_profile.collision_cost_config.safety_margin = 0.025
    trajopt_composite_profile.collision_cost_config.type = (
        CollisionEvaluatorType_SINGLE_TIMESTEP
    )
    trajopt_composite_profile.collision_cost_config.coeff = 20

    trajopt_solver_profile = TrajOptDefaultSolverProfile()

    btr_params = BasicTrustRegionSQPParameters()
    btr_params.max_iter = 200
    btr_params.min_approx_improve = 1e-3
    btr_params.min_trust_box_size = 1e-3

    mt = ModelType(ModelType.OSQP)
    # seems to do its job: fails when I set gurobi; not build with that options
    # mt = ModelType(ModelType.GUROBI)

    trajopt_solver_profile.opt_info = btr_params
    trajopt_solver_profile.convex_solver = mt

    # Create profile dictionary
    trajopt_profiles = ProfileDictionary()
    ProfileDictionary_addProfile_TrajOptPlanProfile(
        trajopt_profiles, TRAJOPT_DEFAULT_NAMESPACE, "CARTESIAN", trajopt_plan_profile
    )

    ProfileDictionary_addProfile_TrajOptSolverProfile(
        trajopt_profiles, TRAJOPT_DEFAULT_NAMESPACE, "DEFAULT", trajopt_solver_profile
    )

    ProfileDictionary_addProfile_TrajOptCompositeProfile(
        trajopt_profiles,
        TRAJOPT_DEFAULT_NAMESPACE,
        "DEFAULT",
        trajopt_composite_profile,
    )
    return trajopt_profiles


def _move_instruction_from_iso(
    goal: Isometry3d, move_type=MoveInstructionType_LINEAR
) -> MoveInstructionPoly_wrap_MoveInstruction:
    cwp_cw = CartesianWaypointPoly_wrap_CartesianWaypoint(CartesianWaypoint(goal))
    mip_mi = MoveInstructionPoly_wrap_MoveInstruction(
        MoveInstruction(cwp_cw, move_type, "CARTESIAN")
    )
    return mip_mi


class Planner:
    def __init__(
        self, mi: ManipulatorInfo, t_env: Environment, profile_dict: ProfileDictionary
    ):
        # Create Task Composer Plugin Factory
        fs_pth = tesseract_task_composer_config_file()
        self.factory = TaskComposerPluginFactory(fs_pth)

        # Create Program
        self.program = CompositeInstruction("DEFAULT")
        self.program.setManipulatorInfo(mi)
        self.t_env = t_env

        self.profiles = profile_dict
        self.task_data = TaskComposerDataStorage()

    def create_task(self):
        """create task after all poses have been added"""
        # Create an AnyPoly containing the program. This explicit step is required because the Python bindings do not
        # support implicit conversion from the CompositeInstruction to the AnyPoly.
        self.program_anypoly = AnyPoly_wrap_CompositeInstruction(self.program)

        # Create the task composer node. In this case the FreespacePipeline is used. Many other are available.
        # self.task = self.factory.createTaskComposerNode("FreespacePipeline")

        # # Create task
        self.task = self.factory.createTaskComposerNode("TrajOptPipeline")
        self.input_key = self.task.getInputKeys()[0]
        self.output_key = self.task.getOutputKeys()[0]

        # Create Task Input Data
        self.input_data = TaskComposerDataStorage()
        self.input_data.setData(self.input_key, self.program_anypoly)

        # Create the task data storage and set the data
        self.task_data.setData(self.input_key, self.program_anypoly)

        # Create the task problem and input
        # self.problem = PlanningTaskComposerProblem(
        #     self.t_env,
        #     self.input_data,
        #     self.profiles
        # )

        # Create the task problem and input
        self.task_planning_problem = PlanningTaskComposerProblemUPtr.make_unique(
            self.t_env, self.task_data, self.profiles
        )

        self.task_problem = PlanningTaskComposerProblemUPtr_as_TaskComposerProblemUPtr(
            self.task_planning_problem
        )
        self.task_input = TaskComposerInput(self.task_problem)

        # Create an executor to run the task
        self.task_executor = self.factory.createTaskComposerExecutor("TaskflowExecutor")

    def add_poses(self, tool_poses):
        # Create cartesian waypoint
        for n, i in enumerate(tool_poses):
            plan_instruction = _move_instruction_from_iso(i)
            plan_instruction.setDescription(f"waypoint_{n}")
            self.program.appendMoveInstruction(plan_instruction)

    def plan(self) -> AnyPoly_as_CompositeInstruction:

        # Assign the current state as the seed for cartesian waypoints
        assignCurrentStateAsSeed(self.program, self.t_env)

        start = time.time()
        # Run the task and wait for completion
        future = self.task_executor.run(self.task.get(), self.task_input)
        future.wait()
        stop = time.time() - start

        print(f"Planning took {stop} seconds.")

        # Retrieve the output, converting the AnyPoly back to a CompositeInstruction
        try:
            results = AnyPoly_as_CompositeInstruction(
                self.task_input.data_storage.getData(self.output_key)
            )
        except RuntimeError as e:
            print(e)
            return None
        else:
            print(results)
            return results


def as_joint_trajectory(planner: Planner) -> JointTrajectory:
    # Plot Process Trajectory
    # TODO as composite instruction
    _ci: AnyPoly = planner.input_data.getData(planner.output_key)

    ci = AnyPoly_as_CompositeInstruction(_ci)

    trajectory: JointTrajectory = toJointTrajectory(ci)
    state_solver = planner.t_env.getStateSolver()
    return trajectory


def make_puzzle_tool_poses() -> list[Isometry3d]:
    path = []  # results
    locator = GeneralResourceLocator()
    # Locate the CSV file using the provided resource locator
    resource = locator.locateResource(
        "package://tesseract_support/urdf/puzzle_bent.csv"
    )
    file_path = resource.getFilePath()

    # Open the CSV file for reading
    with open(file_path, "r") as indata:
        lines = indata.readlines()

    for lnum, line in enumerate(lines):
        if lnum < 2:
            continue

        cells = line.split(",")
        xyzijk = [float(cell) for cell in cells[1:]]  # Ignore the first value

        pos = Vector(xyzijk[0], xyzijk[1], xyzijk[2])  # Convert from mm to meters
        pos /= 1000

        print(pos)
        norm = Vector(xyzijk[3], xyzijk[4], xyzijk[5])

        norm.unitize()

        temp_x = (Vector(-1, -1, -1) * pos).unitized()
        y_axis = norm.cross(temp_x).unitized()
        x_axis = y_axis.cross(norm).unitized()

        # Create an Isometry3d pose
        pose = Isometry3d()
        mat = pose.matrix()

        mat[0][:3] = x_axis
        mat[1][:3] = y_axis
        mat[2][:3] = norm
        mat[3][:3] = pos

        path.append(pose)

    return path


# path = ResourceLocator()


def create_manip() -> ManipulatorInfo:
    # Create manipulator information for the program
    mi = ManipulatorInfo()
    mi.manipulator = "manipulator"
    mi.working_frame = "part"
    mi.tcp_frame = "grinder_frame"
    return mi


class PuzzlePieceExample:
    def __init__(self, env, plotter=None):
        self.env = env
        self.plotter = plotter

    def run(self):
        if self.plotter is not None:
            self.plotter.waitForConnection()

        # Set the robot initial state
        joint_names = [
            "joint_a1",
            "joint_a2",
            "joint_a3",
            "joint_a4",
            "joint_a5",
            "joint_a6",
            "joint_a7",
        ]
        joint_pos = [-0.785398, 0.4, 0.0, -1.9, 0.0, 1.0, 0.0]

        # joint_state = JointState(joint_names, joint_pos)

        self.env.setState(joint_names, np.array(joint_pos))

        mi = create_manip()

        # Get Tool Poses
        tool_poses = make_puzzle_tool_poses()

        profile_dict = create_trajopt_profile()

        pl = Planner(mi, env, profile_dict)
        pl.add_poses(tool_poses)
        pl.create_task()
        results = pl.plan()

        joint_trajectory = as_joint_trajectory(pl)

        # print("Final trajectory is collision-free")
        # return input.isSuccessful()

        # return results
        return joint_trajectory


if __name__ == "__main__":
    make_puzzle_tool_poses()
    env, manip_info, joint_names = get_environment(
        "package://tesseract_support/urdf/puzzle_piece_workcell"
    )

    # Create a viewer and set the environment so the results can be displayed later
    viewer = TesseractViewer()
    viewer.update_environment(env, [0, 0, 0])

    # Set the initial state of the robot
    viewer.update_joint_positions(joint_names, np.ones(len(joint_names)) * 0.1)

    # Start the viewer
    viewer.start_serve_background()

    ppe = PuzzlePieceExample(env)
    results = ppe.run()

    # Update the viewer with the results to animate the trajectory
    # Open web browser to http://localhost:8000 to view the results
    viewer.update_trajectory(results)
    viewer.plot_trajectory(results, manip_info)

    input("press to exit the viewer ( http://localhost:8000 )")
