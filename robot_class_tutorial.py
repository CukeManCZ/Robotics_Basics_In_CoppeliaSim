import math

from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from Include.robot import Robot, LocomotionTypes, create_line

# setting CoppeliaSim remote API
client = RemoteAPIClient()
sim = client.require("sim")
sim.setStepping(True)
sim.stopSimulation()

print("Connected.")

# Known robot definitions in Mobile_robots.ttt
"""
#Steering robot
locomotion = LocomotionTypes.STEERING

left_motor = sim.getObject("/Left_drive")
left_wheels = [left_motor]
right_motor = sim.getObject("/Right_drive")
right_wheels = [right_motor]
steering_motor = sim.getObject("/Steering_drive")
middle_wheels = [steering_motor]

robot_object = sim.getObject("/Main_frame")
R = 0.05                        #   wheel radius
W = 0.152                       #   wheel separation
"""

# Differential robot
locomotion = LocomotionTypes.DIFFERENTIAL

left_f = sim.getObject("/Robot_diff/motor_left_f")
left_b = sim.getObject("/Robot_diff/motor_left_b")
left_wheels = [left_f, left_b]
right_f = sim.getObject("/Robot_diff/motor_right_f")
right_b = sim.getObject("/Robot_diff/motor_right_b")
right_wheels = [right_f, right_b]
middle_wheels = []

robot_object = sim.getObject("/Robot_diff/Main_frame")
R = 0.03                        # wheel radius
W = 0.122                       # wheel separation

# controller inputs
line_v = 0.01                   # line following velocity
points_v = 0.005                # points following velocity
kv = 0.1                        # Controller constant: velocity proportional to distance
kh = 3                          # Controller constant: angle/angular speed proportional to heading angle
kd = 1.2                        # Controller constant: angle/angular speed proportional to normal distance from line

goal_object = sim.getObject("/Goal_point")
goal_path = "/Path"
goal_p1 = sim.getObject("/Line/pA")
goal_p2 = sim.getObject("/Line/pB")


def get_points_from_path(path_obj, point_name, num):
    points = []
    for i in range(num):
        point_addr = path_obj + '/' + point_name + '[' + str(i) + ']'
        point_obj = sim.getObject(point_addr)
        x_i, y_i, z_i = sim.getObjectPosition(point_obj, sim.handle_world)
        points.append([x_i, y_i])

    return points


my_robot = Robot(sim, locomotion, robot_object, R, W, 55, left_wheels, right_wheels, middle_wheels)

sim.startSimulation()

# blocking call going throughout list of points
goal_points = get_points_from_path(goal_path, "ctrlPt", 10)
my_robot.drive_throughout_points(goal_points, 0.01, points_v, kh)

# go to point until certain distance from it
distance = 0.1
x, y, z = sim.getObjectPosition(goal_object, sim.handle_world)
goal_point = [x, y]
while math.dist(my_robot.get_2d_position(), goal_point) > distance:
    x, y, z = sim.getObjectPosition(goal_object, sim.handle_world)
    goal_point = [x, y]

    my_robot.drive_to_point(goal_point, kv, kh)
    sim.step()
my_robot.stop_robot()

# go to line
while True:
    x1, y1, z1 = sim.getObjectPosition(goal_p1)
    x2, y2, z2 = sim.getObjectPosition(goal_p2)
    line = create_line([x1, y1], [x2, y2])
    my_robot.drive_to_line(line, line_v, kd, kh)
    sim.step()
