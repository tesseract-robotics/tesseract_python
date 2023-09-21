#  ported from file glass_upright_example.cpp
# for the original see:
#  https://github.com/tesseract-robotics/tesseract_planning/blob/master/tesseract_examples/src/glass_upright_example.cpp

import time

import numpy as np
from tesseract_robotics import tesseract_environment as te
from tesseract_robotics import tesseract_geometry as tg
from tesseract_robotics import tesseract_scene_graph as tsg
from tesseract_robotics.tesseract_command_language import (
    CompositeInstruction,
    MoveInstructionType_LINEAR,
    MoveInstruction,
    ProfileDictionary,
    CompositeInstructionOrder_ORDERED,
    StateWaypoint,
    StateWaypointPoly_wrap_StateWaypoint,
    MoveInstructionPoly_wrap_MoveInstruction,
    AnyPoly_wrap_CompositeInstruction,
    AnyPoly_as_CompositeInstruction,
    InstructionPoly_as_MoveInstructionPoly,
    WaypointPoly_as_StateWaypointPoly,
    toJointTrajectory,
)
from tesseract_robotics.tesseract_common import (
    Isometry3d,
    ManipulatorInfo,
    AnyPoly,
    JointTrajectory,
)
from tesseract_robotics.tesseract_environment import Environment
from tesseract_robotics.tesseract_motion_planners_trajopt import (
    ProfileDictionary_addProfile_TrajOptCompositeProfile,
    TrajOptDefaultCompositeProfile,
    TrajOptDefaultPlanProfile,
    ProfileDictionary_addProfile_TrajOptPlanProfile,
)
from tesseract_robotics.tesseract_task_composer import (
    TaskComposerPluginFactory,
    TaskComposerDataStorage,
    PlanningTaskComposerProblemUPtr,
    PlanningTaskComposerProblemUPtr_as_TaskComposerProblemUPtr,
    TaskComposerInput,
)

from examples.puzzle import TRAJOPT_DEFAULT_NAMESPACE
from examples.utils import (
    get_environment,
    tesseract_task_composer_config_file,
    as_joint_trajectory,
)
from tesseract_viewer_python.tesseract_robotics_viewer import TesseractViewer


class GlassUprightExample:
    def __init__(self, env: Environment, manip_info: ManipulatorInfo, debug: bool):
        self.env = env
        self.manip_info = manip_info

        self.manip_info.manipulator = "manipulator"
        self.manip_info.tcp_frame = "tool0"
        self.manip_info.working_frame = "base_link"

        self.debug = debug
        self._initial_joint_state()
        self.update_env()

    def addSphere(self) -> te.AddLinkCommand:
        link_sphere = tsg.Link("sphere_attached")
        visual = tsg.Visual()
        origin = Isometry3d.Identity()

        ll = np.array([0.5, 0, 0.55])
        # TODO: stange that a `Translation3d` is not an accepted argument
        origin.setTranslation(ll)

        sphere = tg.Sphere(0.15)
        visual.geometry = sphere
        link_sphere.visual.push_back(visual)

        collision = tsg.Collision()
        collision.origin = visual.origin
        collision.geometry = visual.geometry
        link_sphere.collision.push_back(collision)

        joint_sphere = tsg.Joint("joint_sphere_attached")
        joint_sphere.parent_link_name = "base_link"
        joint_sphere.child_link_name = link_sphere.getName()
        joint_sphere.type = tsg.JointType_FIXED

        cmd = te.AddLinkCommand(link_sphere, joint_sphere)
        self.env.applyCommand(cmd)

        return cmd

    def _initial_joint_state(self):
        #   // Set the robot initial state
        self.joint_names = []
        self.joint_names.append("joint_a3")
        self.joint_names.append("joint_a4")
        self.joint_names.append("joint_a5")
        self.joint_names.append("joint_a2")
        self.joint_names.append("joint_a1")
        self.joint_names.append("joint_a6")
        self.joint_names.append("joint_a7")

        self.joint_start_pos = np.zeros((7,))
        self.joint_end_pos = np.zeros((7,))

        self.joint_start_pos[0] = -0.4
        self.joint_start_pos[1] = 0.2762
        self.joint_start_pos[2] = 0.0
        self.joint_start_pos[3] = -1.3348
        self.joint_start_pos[4] = 0.0
        self.joint_start_pos[5] = 1.4959
        self.joint_start_pos[6] = 0.0
        #
        #   Eigen::VectorXd self.joint_end_pos(7);
        self.joint_end_pos[0] = 0.4
        self.joint_end_pos[1] = 0.2762
        self.joint_end_pos[2] = 0.0
        self.joint_end_pos[3] = -1.3348
        self.joint_end_pos[4] = 0.0
        self.joint_end_pos[5] = 1.4959
        self.joint_end_pos[6] = 0.0

        self.env.setState(self.joint_names, self.joint_start_pos)

    def update_env(self):
        # Get the state solver. This must be called again after environment is updated
        solver = self.env.getStateSolver()

        # Get the discrete contact manager. This must be called again after the environment is updated
        manager = self.env.getDiscreteContactManager()
        manager.setActiveCollisionObjects(self.env.getActiveLinkNames())

    def plan(self):
        self._initial_joint_state()
        self.update_env()
        cmd = self.addSphere()
        self.update_env()

        # Create the input command program. Note the use of *_wrap_MoveInstruction functions. This is required because the
        # Python bindings do not support implicit conversion from the MoveInstruction to the MoveInstructionPoly.
        program = CompositeInstruction("UPRIGHT", CompositeInstructionOrder_ORDERED)
        program.setManipulatorInfo(self.manip_info)

        waypointA = StateWaypoint(self.joint_names, self.joint_start_pos)
        waypointB = StateWaypoint(self.joint_names, self.joint_end_pos)
        wp0 = StateWaypointPoly_wrap_StateWaypoint(waypointA)
        wp1 = StateWaypointPoly_wrap_StateWaypoint(waypointB)

        start_instruction = MoveInstruction(wp0, MoveInstructionType_LINEAR, "UPRIGHT")
        start_instruction.setDescription("start instruction")

        plan_f0 = MoveInstruction(wp1, MoveInstructionType_LINEAR, "UPRIGHT")
        plan_f0.setDescription("freespace_plan")

        program.appendMoveInstruction(
            MoveInstructionPoly_wrap_MoveInstruction(start_instruction)
        )
        program.appendMoveInstruction(MoveInstructionPoly_wrap_MoveInstruction(plan_f0))

        self.profile = ProfileDictionary()

        # todo: no ifopt
        composite_profile = TrajOptDefaultCompositeProfile()

        composite_profile.collision_cost_config.enabled = True
        # todo: expected tc.CollisionEvaluatorType_DISCRETE_CONTINUOUS
        # composite_profile.collision_cost_config.type = tc.CollisionEvaluatorType_CONTINUOUS
        composite_profile.collision_cost_config.safety_margin = 0.01
        composite_profile.collision_cost_config.safety_margin_buffer = 0.01
        composite_profile.collision_cost_config.coeff = 1
        composite_profile.collision_constraint_config.enabled = True
        # todo: expected tc.CollisionEvaluatorType_DISCRETE_CONTINUOUS
        # composite_profile.collision_constraint_config.type = tc.CollisionEvaluatorType_CONTINUOUS
        composite_profile.collision_constraint_config.safety_margin = 0.01
        composite_profile.collision_constraint_config.safety_margin_buffer = 0.01
        composite_profile.collision_constraint_config.coeff = 1
        composite_profile.smooth_velocities = True
        composite_profile.smooth_accelerations = False
        composite_profile.smooth_jerks = False
        composite_profile.velocity_coeff = np.array([1.0])

        # TODO: add a method `add_profile` that add the correct profile
        # by inspecting composite_profile.__class__ to have something a little more
        # pythonic

        ProfileDictionary_addProfile_TrajOptCompositeProfile(
            self.profile, TRAJOPT_DEFAULT_NAMESPACE, "UPRIGHT", composite_profile
        )

        plan_profile = TrajOptDefaultPlanProfile()
        plan_profile.joint_coeff = np.ones((7,))
        plan_profile.cartesian_coeff = np.array(
            [0.0, 0.0, 0.0, 5.0, 5.0, 5.0],
        )

        ProfileDictionary_addProfile_TrajOptPlanProfile(
            self.profile, TRAJOPT_DEFAULT_NAMESPACE, "UPRIGHT", plan_profile
        )

        # TODO: refactor to use the planner class

        fs_pth = tesseract_task_composer_config_file()
        self.factory = TaskComposerPluginFactory(fs_pth)

        self.task = self.factory.createTaskComposerNode("TrajOptPipeline")
        self.input_key = self.task.getInputKeys()[0]
        self.output_key = self.task.getOutputKeys()[0]

        program_anypoly = AnyPoly_wrap_CompositeInstruction(program)

        self.task_data = TaskComposerDataStorage()
        self.task_data.setData(self.input_key, program_anypoly)

        self.task_planning_problem = PlanningTaskComposerProblemUPtr.make_unique(
            self.env, self.task_data, self.profile
        )

        self.task_problem = PlanningTaskComposerProblemUPtr_as_TaskComposerProblemUPtr(
            self.task_planning_problem
        )
        self.task_input = TaskComposerInput(self.task_problem)

        # Create an executor to run the task
        self.task_executor = self.factory.createTaskComposerExecutor("TaskflowExecutor")

        start = time.time()
        # Run the task and wait for completion
        future = self.task_executor.run(self.task.get(), self.task_input)
        future.wait()
        stop = time.time() - start

        print(f"was planning aborted? {self.task_input.isAborted()}")
        print(f"was planning succesful? {self.task_input.isSuccessful()}")

        print(f"planning took {stop} seconds")

        try:
            # output_key = self.task.getOutputKeys()[0]
            _ci: AnyPoly = self.task_data.getData(self.output_key)
            ci = AnyPoly_as_CompositeInstruction(_ci)
        except RuntimeError:
            print("could not create a composite instruction from results")
            raise
        else:
            trajectory: JointTrajectory = toJointTrajectory(ci)

    # TODO: move to planner
    def _create_viewer(self):
        # Create a viewer and set the environment so the results can be displayed later
        viewer = TesseractViewer()
        viewer.update_environment(self.env, [0, 0, 0])

        # Set the initial state of the robot
        viewer.update_joint_positions(
            self.joint_names, np.array([1, -0.2, 0.01, 0.3, -0.5, 1])
        )

        # Start the viewer
        viewer.start_serve_background()
        return viewer

    def plot(self):
        # Retrieve the output, converting the AnyPoly back to a CompositeInstruction
        results = as_joint_trajectory(self.task, self.task_data)

        # Display the output
        # Print out the resulting waypoints
        for instr in results:
            print(f"instr: {instr}")
            assert instr.isMoveInstruction()
            move_instr1 = InstructionPoly_as_MoveInstructionPoly(instr)
            wp1 = move_instr1.getWaypoint()
            # assert wp1.isStateWaypoint()
            wp = WaypointPoly_as_StateWaypointPoly(wp1)
            print(f"Joint Positions: {wp.getPosition().flatten()} time: {wp.getTime()}")

        viewer = self._create_viewer()

        # Update the viewer with the results to animate the trajectory
        # Open web browser to http://localhost:8000 to view the results
        viewer.update_trajectory(results)
        viewer.plot_trajectory(results, self.manip_info)


def run():
    env, manip_info, joint_names = get_environment(
        "package://tesseract_support/urdf/lbr_iiwa_14_r820"
    )

    print(env, manip_info)

    gue = GlassUprightExample(env, manip_info, True)
    gue.plan()
    gue.plot()

    input("press enter to exit")


if __name__ == "__main__":
    run()
