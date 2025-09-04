import numpy as np
from pydrake.all import *
import time
from kinematics_solver import SawyerKinematicsSolver


start = time.time()
builder = DiagramBuilder()
plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.05)
# note i change the urdf to have the head as a fixed joint instead of a revolute joint
# i also added actuators which is required for drake urdf files
urdf = "sawyer_description/urdf/sawyer.urdf"
(arm_idx,) = Parser(plant).AddModels(urdf)
X_robot = RigidTransform()
X_robot.set_translation([0, 0, 0.05])
plant.WeldFrames(
    plant.world_frame(), plant.GetFrameByName("base_link", arm_idx), X_robot
)
plant.Finalize()
print(f"plant finalized in {time.time() - start:.2f} seconds")

print("=== Positions and Velocities")
print("Num actuators: ", plant.num_actuators())
print("Num positions: ", plant.num_positions())
print("Num velocities: ", plant.num_velocities())

start = time.time()
ik_solver = SawyerKinematicsSolver()
position = [0.61549853, -0.00750006, 0.39920147]
orientation = [0.06163138, 0.70441622, 0.06162621, 0.70441577]
print(f"target {position=}")
print(f"target {orientation=}")

success, q = ik_solver.solve_ik(
    plant, pos_W=position, q_W=orientation, ee_link_name="right_hand"
)
print(f"IK success: {success}")
print(f"IK solution: {q}")
print(f"ik solved in {time.time() - start:.2f} seconds")


# double check here to make sure that the joint values actually put the ee in that spot
context = plant.CreateDefaultContext()
plant.SetPositions(context, q)
plant.SetVelocities(context, np.zeros(7))
ee_body = plant.GetBodyByName("right_hand")
ee_pose = plant.CalcRelativeTransform(
    context, plant.world_frame(), ee_body.body_frame()
)
ee_pos = ee_pose.translation()
ee_rot = ee_pose.rotation().ToQuaternion().wxyz()
print(f"this should match target position  {ee_pos=}")
print(f"this should match your target orientation {ee_rot=}")

# below is just a simple visualization of the robot at the ik solution
# Robot position and velocities
q_home = np.array(q)
q_home0 = np.hstack((q_home, np.zeros(plant.num_velocities())))

kp = np.array([1, 1, 1, 1, 1, 1, 1])
kd = 2.0 * np.sqrt(kp)
ki = np.zeros(7)

controller = InverseDynamicsController(
    robot=plant, kp=kp, ki=ki, kd=kd, has_reference_acceleration=False
)

builder.AddSystem(controller)
desired_state = builder.AddSystem(ConstantVectorSource(q_home0 * 0.8))
builder.Connect(
    plant.get_state_output_port(), controller.get_input_port_estimated_state()
)
builder.Connect(
    desired_state.get_output_port(), controller.get_input_port_desired_state()
)
builder.Connect(controller.get_output_port(0), plant.get_actuation_input_port())

# Add visualization to see the geometries.
meshcat = StartMeshcat()
AddDefaultVisualization(builder=builder, meshcat=meshcat)

# Start recording
meshcat.StartRecording()

# Finalize the diagram
diagram = builder.Build()
diagram_context = diagram.CreateDefaultContext()
plant_context = diagram.GetMutableSubsystemContext(plant, diagram_context)
plant.SetPositionsAndVelocities(plant_context, q_home0)

# Simulate the controller
simulator = Simulator(diagram, diagram_context)
simulator.set_publish_every_time_step(True)
simulator.set_target_realtime_rate(1.0)
simulator.set_publish_at_initialization(True)
simulator.Initialize()


print("=== Starting Simulation ===")
sim_time = 5.0

simulator.AdvanceTo(sim_time)
meshcat.StopRecording()
meshcat.PublishRecording()

html_content = meshcat.StaticHtml()
with open("latest_pd_trajectory.html", "w") as f:
    f.write(html_content)

print("Simulation completed!")
