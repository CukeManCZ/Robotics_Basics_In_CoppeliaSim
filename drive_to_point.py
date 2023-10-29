import math
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient()
sim = client.require("sim")
sim.setStepping(True)
sim.stopSimulation()

print("Connected.")

left_motor = sim.getObject("/Left_drive")
right_motor = sim.getObject("/Right_drive")
steering_motor = sim.getObject("/Steering_drive")

goal_object = sim.getObject("/Goal_point")
robot_object = sim.getObject("/Main_frame")
robot_body = sim.getObject("/Robot")


def get_xy(object_handle):
    pos = sim.getObjectPosition(object_handle, sim.handle_world)
    return pos[0], pos[1]


def controller_speed_proportional_to_dist(goal_poit, robot_point, kv=0.7):
    distance = math.dist(goal_poit, robot_point)

    velocity = kv * distance
    return velocity


def get_rel_angle(p1, p2):
    return math.atan2(p1[1]-p2[1], p1[0]-p2[0])


def get_yaw_angle(object_handle):
    orientation = sim.getObjectOrientation(object_handle, sim.handle_world)
    return orientation[2]


def controller_steer_to_point(goal_point, position, orientation, kh=1, max_steering=55):
    diff = get_rel_angle(goal_point, position) - orientation

    max_steering = math.radians(max_steering)

    if diff > max_steering:
        diff = max_steering
    elif diff < -max_steering:
        diff = -max_steering
    return kh * diff


robot_start_pos = sim.getObjectPosition(robot_body, sim.handle_world)
n_of_iterations = 5

sim.startSimulation()
for iteration in range(1, n_of_iterations+1, 1):
    goal_pos = get_xy(goal_object)
    robot_pos = get_xy(robot_object)
    robot_ori = get_yaw_angle(robot_object)

    while math.dist(goal_pos, robot_pos) > 0.08:
        v = controller_speed_proportional_to_dist(goal_pos, robot_pos)
        gamma = controller_steer_to_point(goal_pos, robot_pos, robot_ori)

        sim.setJointTargetVelocity(left_motor, v)
        sim.setJointTargetVelocity(right_motor, v)
        sim.setJointTargetPosition(steering_motor, gamma)

        goal_pos = get_xy(goal_object)
        robot_pos = get_xy(robot_object)
        robot_ori = get_yaw_angle(robot_object)
        sim.step()

    sim.setObjectPosition(robot_body, robot_start_pos, sim.handle_world)
    sim.wait(1)
    print(iteration, ". iteration successful")

sim.stopSimulation()
