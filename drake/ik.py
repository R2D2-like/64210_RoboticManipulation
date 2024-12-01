import numpy as np
from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    DiagramBuilder,
    DiscreteContactApproximation,
    InverseKinematics,
    MeshcatVisualizer,
    MeshcatVisualizerParams,
    Parser,
    RigidTransform,
    RotationMatrix,
    Solve,
    StartMeshcat,
    WeldJoint,
    eq,
    Simulator
)
from manipulation import ConfigureParser, running_as_notebook
from manipulation.station import LoadScenario, MakeHardwareStation
from manipulation.scenarios import AddMultibodyTriad
from manipulation.meshcat_utils import AddMeshcatTriad

import time

# Start the visualizer
meshcat = StartMeshcat()
print("Meshcat URL:", meshcat.web_url())

def build_env():
    """
    Build the simulation environment and set up visualization with Meshcat.
    """
    builder = DiagramBuilder()
    scenario = LoadScenario(filename="scenario.yaml")
    station = builder.AddSystem(MakeHardwareStation(scenario, meshcat))
    plant = station.GetSubsystemByName("plant")
    scene_graph = station.GetSubsystemByName("scene_graph")

    # Add visualization for the scene
    MeshcatVisualizer.AddToBuilder(
        builder,
        station.GetOutputPort("query_object"),
        meshcat,
        MeshcatVisualizerParams(delete_on_initialization_event=False),
    )
    AddMultibodyTriad(plant.GetFrameByName("body"), scene_graph)
    diagram = builder.Build()
    # context = plant.CreateDefaultContext()
    # gripper = plant.GetBodyByName("body")

    # initial_pose = plant.EvalBodyPoseInWorld(context, gripper)
    context = plant.CreateDefaultContext()
    initial_q = plant.GetPositions(context) 
    return diagram, plant, scene_graph, initial_q

goal_rotation = RotationMatrix(
    [
        [1, 0, 0],
        [0, 1, 0],
        [0, 0, 1],
    ]
)
goal_position = np.array([0, 0.75, 0.5])
goal_pose = RigidTransform(goal_rotation, goal_position)

def solve_ik(X_WG, max_tries=10, fix_base=False, base_pose=np.zeros(3)):
    """
    Solve the inverse kinematics problem for the given goal pose.
    """
    diagram, plant, scene_graph, initial_q = build_env()

    plant.GetFrameByName("body")  # End effector frame

    context = diagram.CreateDefaultContext()
    plant_context = plant.GetMyContextFromRoot(context)
    scene_graph.GetMyContextFromRoot(context)

    # Set up inverse kinematics
    ik = InverseKinematics(plant, plant_context)
    q_variables = ik.q()  # Get variables for MathematicalProgram
    prog = ik.prog()  # Get MathematicalProgram
    q_nominal = np.zeros(len(q_variables))
    q_nominal[0:3] = base_pose
    prog.AddQuadraticErrorCost(np.eye(len(q_variables)), q_nominal, q_variables)

    # Add position constraint
    ik.AddPositionConstraint(
        frameB=plant.GetFrameByName("body"),
        p_BQ=np.array([0, 0, 0]),
        frameA=plant.world_frame(),
        p_AQ_lower=X_WG.translation() - 0.01,
        p_AQ_upper=X_WG.translation() + 0.01,
    )

    for count in range(max_tries):
        # Compute a random initial guess
        q_initial_guess = np.random.uniform(-1.0, 1.0, len(q_variables))
        prog.SetInitialGuess(q_variables, q_initial_guess)

        result = Solve(prog)

        if result.is_success():
            print("Succeeded in %d tries!" % (count + 1))
            return diagram, plant, scene_graph, initial_q, result.GetSolution(q_variables), context

    print("Failed!")
    return None, None, None, None, None, None

# Solve the inverse kinematics problem and visualize the result 
diagram, plant, scene_graph, initial_q, res, context = solve_ik(
    goal_pose,
    max_tries=20,
    fix_base=True,
    base_pose=np.array([0, 0, 0]),
    )



def execute_trajectory(plant, diagram_context, trajectory, time_step=0.01):
    """
    Execute the trajectory in the simulation.
    """
    # Get the subsystem context for the plant
    plant_context = plant.GetMyContextFromRoot(diagram_context)

    for q in trajectory:
        # Update the joint positions in the plant context
        plant.SetPositions(plant_context, q)
        # Visualization update
        meshcat.PublishRecording()
        time.sleep(time_step)

def generate_trajectory(initial_q, final_q, num_steps=100):
    """
    Generate a trajectory from the initial joint configuration to the final joint configuration.
    """
    trajectory = []
    for i in range(num_steps):
        alpha = i / (num_steps - 1)  # Interpolation factor
        q_interp = (1 - alpha) * initial_q + alpha * final_q
        trajectory.append(q_interp)
    return trajectory

def interpolate_pose(plant, diagram_context, q):
    """
    Compute the pose of the end-effector for a given joint configuration.
    """
    # Get the subsystem context for the plant
    plant_context = plant.GetMyContextFromRoot(diagram_context)

    # Update the plant context with the joint positions
    plant.SetPositions(plant_context, q)

    # Get the pose of the end-effector
    ee_frame = plant.GetFrameByName("body")
    ee_pose = plant.CalcRelativeTransform(plant_context, plant.world_frame(), ee_frame)

    return ee_pose
from pydrake.systems.primitives import TrajectorySource
from pydrake.trajectories import PiecewisePolynomial


from pydrake.systems.primitives import TrajectorySource, MatrixGain

from pydrake.systems.primitives import ConstantVectorSource, TrajectorySource, MatrixGain

from pydrake.systems.primitives import ConstantVectorSource, TrajectorySource, MatrixGain

def BuildAndSimulateTrajectory(q_traj, duration=0.01):
    """Simulate trajectory for manipulation station.
    @param q_traj: Trajectory class used to initialize TrajectorySource for joints.
    """
    builder = DiagramBuilder()

    # Load the scenario and create the station
    scenario = LoadScenario(filename="scenario.yaml")
    station = builder.AddSystem(MakeHardwareStation(scenario, meshcat))
    plant = station.GetSubsystemByName("plant")
    scene_graph = station.GetSubsystemByName("scene_graph")
    AddMultibodyTriad(plant.GetFrameByName("body"), scene_graph)

    # Add the trajectory source for the robot joints
    q_traj_system = builder.AddSystem(TrajectorySource(q_traj))
    
    # Add a MatrixGain to trim the output to the correct size (7)
    trim_matrix = np.eye(7, 16)  # A 7x16 matrix to extract the first 7 elements
    trim_gain = builder.AddSystem(MatrixGain(trim_matrix))
    builder.Connect(q_traj_system.get_output_port(), trim_gain.get_input_port())

    # Connect the trimmed output to the robot's input port
    builder.Connect(trim_gain.get_output_port(), station.GetInputPort("iiwa.position"))

    # Add a constant source for the gripper (wsg) position
    wsg_position_source = builder.AddSystem(ConstantVectorSource([0.04]))  # Gripper open position
    builder.Connect(wsg_position_source.get_output_port(), station.GetInputPort("wsg.position"))

    # Build the diagram and start the simulation
    diagram = builder.Build()
    simulator = Simulator(diagram)

    # Record visualization in Meshcat
    meshcat.StartRecording(set_visualizations_while_recording=False)
    simulator.AdvanceTo(duration)
    meshcat.PublishRecording()

    return simulator, plant

if res is not None:
    num_steps = 100  # Number of steps for the trajectory
    times = np.linspace(0, 10, num_steps)  # Simulation time points

    # Generate the trajectory
    trajectory_points = np.array(generate_trajectory(initial_q, res, num_steps)).T
    q_traj = PiecewisePolynomial.FirstOrderHold(times, trajectory_points)

    print("Trajectory generated. Visualizing and simulating...")

    # Visualize the trajectory using AddMeshcatTriad
    for idx, q in enumerate(trajectory_points.T):  # Transpose to iterate over configurations
        pose = interpolate_pose(plant, context, q)  # Compute the pose
        AddMeshcatTriad(
            meshcat,
            path=f"trajectory/pose_{idx}",
            X_PT=pose,
            length=0.1,  # Length of the triad axes
            radius=0.005,  # Radius of the axes
            opacity=0.2,
        )

    print("Trajectory visualization complete. Starting simulation...")

    # Run the simulation
    simulator, plant = BuildAndSimulateTrajectory(q_traj, 20.0)

    print("Simulation completed.")

try:
    # Keep the visualization running
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("Visualization interrupted.")


