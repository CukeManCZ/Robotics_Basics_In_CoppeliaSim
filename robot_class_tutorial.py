from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from Include.robot import Robot, LocomotionTypes

client = RemoteAPIClient()
sim = client.require("sim")
sim.setStepping(True)
sim.stopSimulation()

print("Connected.")

left_motor = sim.getObject("/Left_drive")
left_wheels = [left_motor]
right_motor = sim.getObject("/Right_drive")
right_wheels = [right_motor]
steering_motor = sim.getObject("/Steering_drive")
middle_motors = [steering_motor]

goal_object = sim.getObject("/Goal_point")
robot_object = sim.getObject("/Main_frame")
robot_body = sim.getObject("/Robot")

R = 0.05

my_robot = Robot(sim, LocomotionTypes.STEERING, robot_object, R, 0, 0, 55, left_wheels, right_wheels, middle_motors)

sim.startSimulation()
while True:
    x, y, z = sim.getObjectPosition(goal_object, sim.handle_world)
    goal_point = [x, y]

    my_robot.drive_to_point(goal_point, 0.1, 1)

    sim.step()
