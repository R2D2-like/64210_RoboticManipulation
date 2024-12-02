import numpy as np
from pydrake.all import (
    DiagramBuilder,
    InverseKinematics,
    MeshcatVisualizer,
    MeshcatVisualizerParams,
    RigidTransform,
    RotationMatrix,
    Solve,
    StartMeshcat,
    Simulator
)
from manipulation.station import LoadScenario, MakeHardwareStation
from manipulation.scenarios import AddMultibodyTriad
from manipulation.meshcat_utils import AddMeshcatTriad
import time
from pydrake.trajectories import PiecewisePolynomial
from pydrake.systems.primitives import TrajectorySource, MatrixGain
from pydrake.systems.framework import LeafSystem, BasicVector

# Start the visualizer
meshcat = StartMeshcat()
print("Meshcat URL:", meshcat.web_url())


def build_env():
    """Build the simulation environment and set up visualization with Meshcat."""
    builder = DiagramBuilder()
    scenario = LoadScenario(filename="scenario.yaml")
    station = builder.AddSystem(MakeHardwareStation(scenario, meshcat))
    plant = station.GetSubsystemByName("plant")
    scene_graph = station.GetSubsystemByName("scene_graph")

    MeshcatVisualizer.AddToBuilder(
        builder,
        station.GetOutputPort("query_object"),
        meshcat,
        MeshcatVisualizerParams(delete_on_initialization_event=False),
    )
    AddMultibodyTriad(plant.GetFrameByName("body"), scene_graph)
    diagram = builder.Build()
    context = plant.CreateDefaultContext()
    initial_q = plant.GetPositions(context)
    return diagram, plant, scene_graph, initial_q


def solve_ik(X_WG, max_tries=10, initial_guess=None):
    """Solve the inverse kinematics problem for the given goal pose, including orientation constraints."""
    diagram, plant, scene_graph, initial_q = build_env()
    context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyContextFromRoot(context)

    ik = InverseKinematics(plant, plant_context)
    q_variables = ik.q()
    prog = ik.prog()

    q_nominal = initial_guess if initial_guess is not None else np.zeros(len(q_variables))
    prog.AddQuadraticErrorCost(np.eye(len(q_variables)), q_nominal, q_variables)

    # Add position constraint
    ik.AddPositionConstraint(
        frameB=plant.GetFrameByName("body"),
        p_BQ=np.array([0, 0, 0]),
        frameA=plant.world_frame(),
        p_AQ_lower=X_WG.translation() - 0.01,
        p_AQ_upper=X_WG.translation() + 0.01,
    )

    # Add orientation constraint
    ik.AddOrientationConstraint(
        frameAbar=plant.world_frame(),
        R_AbarA=X_WG.rotation(),
        frameBbar=plant.GetFrameByName("body"),
        R_BbarB=RotationMatrix.Identity(),
        theta_bound=0.1,
    )

    for count in range(max_tries):
        q_initial_guess = (
            initial_guess if initial_guess is not None else np.random.uniform(-1.0, 1.0, len(q_variables))
        )
        prog.SetInitialGuess(q_variables, q_initial_guess)
        result = Solve(prog)

        if result.is_success():
            print(f"Succeeded in {count + 1} tries.")
            return diagram, plant, scene_graph, initial_q, result.GetSolution(q_variables), context

    print("IK failed!")
    return None, None, None, None, None, None


class GripperControlSystem(LeafSystem):
    """Control system for the gripper."""
    def __init__(self, open_position=0.1, close_position=0.0, close_time=20.0, open_time=30.0):
        super().__init__()
        self.open_position = open_position
        self.close_position = close_position
        self.close_time = close_time
        self.open_time = open_time
        self.DeclareVectorOutputPort("wsg.position", BasicVector(1), self.CalcGripperPosition)

    def CalcGripperPosition(self, context, output):
        current_time = context.get_time()
        if current_time >= self.open_time:
            output.SetAtIndex(0, self.open_position)
        elif current_time >= self.close_time:
            output.SetAtIndex(0, self.close_position)
        else:
            output.SetAtIndex(0, self.open_position)


def visualize_goal_frames(meshcat, goal_poses):
    """Visualize multiple goal poses in Meshcat."""
    for idx, pose in enumerate(goal_poses, start=1):
        AddMeshcatTriad(
            meshcat,
            path=f"goal_frame_{idx}",
            X_PT=pose,
            length=0.2,
            radius=0.01,
            opacity=1.0,
        )


def generate_trajectory(initial_q, final_q, num_steps=100):
    """Generate a linear trajectory between two configurations."""
    alphas = np.linspace(0, 1, num_steps)
    return [(1 - alpha) * initial_q + alpha * final_q for alpha in alphas]


def build_and_simulate_trajectory(q_traj, gripper_close_time=20.0, gripper_open_time=30.0):
    """Simulate the robot's motion along the generated trajectory."""
    builder = DiagramBuilder()
    scenario = LoadScenario(filename="scenario.yaml")
    station = builder.AddSystem(MakeHardwareStation(scenario, meshcat))

    # Add trajectory source
    q_traj_system = builder.AddSystem(TrajectorySource(q_traj))
    trim_gain = builder.AddSystem(MatrixGain(np.eye(7, 16)))
    builder.Connect(q_traj_system.get_output_port(), trim_gain.get_input_port())
    builder.Connect(trim_gain.get_output_port(), station.GetInputPort("iiwa.position"))

    # Add gripper control
    gripper_control_system = builder.AddSystem(
        GripperControlSystem(
            open_position=0.04,
            close_position=0.0,
            close_time=gripper_close_time,
            open_time=gripper_open_time
        )
    )
    builder.Connect(gripper_control_system.get_output_port(0), station.GetInputPort("wsg.position"))

    diagram = builder.Build()
    simulator = Simulator(diagram)
    simulator.set_target_realtime_rate(1.0)

    # Simulate
    meshcat.StartRecording(set_visualizations_while_recording=False)
    simulator.AdvanceTo(gripper_open_time + 1.0)
    meshcat.PublishRecording()
    print("Simulation completed.")


# Define goal poses
goal_rotation = RotationMatrix([[-1, 0, 0], [0, 0, -1], [0, -1, 0]])
goal_pose1 = RigidTransform(goal_rotation, np.array([-0.04, 0.65, 0.6]))
goal_pose2 = RigidTransform(goal_rotation, np.array([-0.04, 0.65, 0.53]))
goal_pose3 = RigidTransform(goal_rotation, np.array([0.2, 0.5, 0.6]))

# Visualize goal frames
visualize_goal_frames(meshcat, [goal_pose1, goal_pose2, goal_pose3])

# Solve IK for all poses
diagram1, plant1, scene_graph1, initial_q1, res1, context1 = solve_ik(goal_pose1)
diagram2, plant2, scene_graph2, initial_q2, res2, context2 = solve_ik(goal_pose2, initial_guess=res1)
diagram3, plant3, scene_graph3, initial_q3, res3, context3 = solve_ik(goal_pose3, initial_guess=res2)

if res1 is not None and res2 is not None and res3 is not None:
    # Generate trajectories
    num_steps = 100
    trajectory1 = generate_trajectory(initial_q1, res1, num_steps)
    trajectory2 = generate_trajectory(res1, res2, num_steps)
    trajectory3 = generate_trajectory(res2, res3, num_steps)

    # Combine trajectories
    combined_times = np.hstack([np.linspace(0, 10, num_steps), np.linspace(10, 20, num_steps)[1:], np.linspace(20, 30, num_steps)[1:]])
    combined_trajectory = np.hstack([
        np.array(trajectory1).T,
        np.array(trajectory2).T[:, 1:],
        np.array(trajectory3).T[:, 1:]
    ])

    q_traj_combined = PiecewisePolynomial.FirstOrderHold(combined_times, combined_trajectory)

    # Simulate
    build_and_simulate_trajectory(q_traj_combined)

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("Visualization interrupted.")
