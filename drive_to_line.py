import math
from coppeliasim_zmqremoteapi_client import RemoteAPIClient


def create_line(point_a, point_b):
    k_a = point_b[1] - point_a[1]
    k_b = point_a[0] - point_b[0]
    k_c = - k_a * point_a[0] - k_b * point_a[1]
    return k_a, k_b, k_c


def normal_dist_from_line(line_abc, point):
    numerator = line_abc[0] * point[0] + line_abc[1]*point[1] + line_abc[2] * 1
    denominator = math.sqrt(line_abc[0]**2 + line_abc[1]**2)
    return numerator/denominator


def line_slope_angle(line_abc):
    return math.atan2(line_abc[0], -line_abc[1])


def controller_line_follow(line_abc, position, orientation, kd=0.5, kh=1):
    d = normal_dist_from_line(line_abc, position)
    gamma_x = line_slope_angle(line_abc)

    max_steering = math.radians(40)

    steering = kd*d + kh*(gamma_x - orientation)

    if steering > max_steering:
        steering = max_steering
    elif steering < -max_steering:
        steering = -max_steering

    return steering


def get_xy(object_handle):
    pos = sim.getObjectPosition(object_handle, sim.handle_world)
    return pos[0], pos[1]


def get_yaw_angle(object_handle):
    orientation = sim.getObjectOrientation(object_handle, sim.handle_world)
    return orientation[2]


client = RemoteAPIClient()
sim = client.require("sim")
sim.setStepping(True)
sim.stopSimulation()

print("Connected.")

left_motor = sim.getObject("/Left_drive")
right_motor = sim.getObject("/Right_drive")
steering_motor = sim.getObject("/Steering_drive")

goal_p1 = sim.getObject("/Path/pA")
goal_p2 = sim.getObject("/Path/pB")
robot_object = sim.getObject("/Main_frame")
robot_body = sim.getObject("/Robot")

v = 2

robot_start_pos = sim.getObjectPosition(robot_body, sim.handle_world)
n_of_iterations = 5

sim.startSimulation()

goal_line = create_line(get_xy(goal_p1), get_xy(goal_p2))
while True:
    robot_pos = get_xy(robot_object)
    robot_ori = get_yaw_angle(robot_object)

    gamma = controller_line_follow(goal_line, robot_pos, robot_ori)

    sim.setJointTargetVelocity(left_motor, v)
    sim.setJointTargetVelocity(right_motor, v)
    sim.setJointTargetPosition(steering_motor, gamma)

    sim.step()
