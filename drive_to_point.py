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

wheel_diameter = 0.1
wheel_base = 0.3


def get_xy(object_handle):
    pos = sim.getObjectPosition(object_handle, sim.handle_world)
    return pos[0], pos[1]


def compute_vel():
    kv = 0.7
    ob1_pos = get_xy(goal_object)
    ob2_pos = get_xy(robot_object)

    distance = math.dist(ob1_pos, ob2_pos)

    velocity = kv * distance
    return velocity


def get_rel_angle():
    ob1_pos = get_xy(goal_object)
    ob2_pos = get_xy(robot_object)
    return math.atan2(ob1_pos[1]-ob2_pos[1], ob1_pos[0]-ob2_pos[0], )


def get_yaw_angle(object_handle):
    orientation = sim.getObjectOrientation(object_handle, sim.handle_world)
    return orientation[2]


def compute_steering():
    kh = 1
    diff = get_rel_angle() - get_yaw_angle(robot_object)

    max_steering = math.radians(55)

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

    while math.dist(goal_pos, robot_pos) > 0.08:
        v = compute_vel()
        gamma = compute_steering()

        sim.setJointTargetVelocity(left_motor, v)
        sim.setJointTargetVelocity(right_motor, v)
        sim.setJointTargetPosition(steering_motor, gamma)

        goal_pos = get_xy(goal_object)
        robot_pos = get_xy(robot_object)
        sim.step()

    sim.setObjectPosition(robot_body, robot_start_pos, sim.handle_world)
    sim.wait(1)
    print(iteration, ". iteration successful")

sim.stopSimulation()
