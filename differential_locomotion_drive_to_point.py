import math
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient()
sim = client.require("sim")
sim.setStepping(True)
sim.stopSimulation()

print("Connected.")

left_f = sim.getObject("/Robot_diff/motor_left_f")
left_b = sim.getObject("/Robot_diff/motor_left_b")
right_f = sim.getObject("/Robot_diff/motor_right_f")
right_b = sim.getObject("/Robot_diff/motor_right_b")

goal_object = sim.getObject("/Goal_point")
robot_object = sim.getObject("/Robot_diff/Main_frame")
robot_body = sim.getObject("/Robot_diff")


def angular_vel(velocity, radius):
    return velocity/radius


def turn_rate(velocity_l, velocity_r, wheel_sep):
    return (velocity_r-velocity_l)/wheel_sep


def forward_vel(velocity_l, velocity_r):
    return (velocity_l+velocity_r)/2


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


def controller_steer_to_point(goal_point, position, orientation, kh=1):
    diff = get_rel_angle(goal_point, position) - orientation

    steering_vel = kh*diff
    return steering_vel


def get_left_right_vel(forward_velocity, steering_velocity, wheel_dist):
    right_velocity = forward_velocity + (wheel_dist/2)*steering_velocity
    left_velocity = forward_velocity - (wheel_dist/2)*steering_velocity

    return left_velocity, right_velocity


R = 0.03
W = 0.122

sim.startSimulation()
robot_pos = get_xy(robot_object)
robot_ori = get_yaw_angle(robot_object)
goal_pos = get_xy(goal_object)

while True:
    linear_vel = controller_speed_proportional_to_dist(goal_pos, robot_pos, 0.1)
    steer_vel = controller_steer_to_point(goal_pos, robot_pos, robot_ori, 3)

    v_l, v_r = get_left_right_vel(linear_vel, steer_vel, W)
    w_l = angular_vel(v_l, R)
    w_r = angular_vel(v_r, R)

    sim.setJointTargetVelocity(left_f, w_l)
    sim.setJointTargetVelocity(left_b, w_l)
    sim.setJointTargetVelocity(right_f, -w_r)
    sim.setJointTargetVelocity(right_b, -w_r)

    robot_pos = get_xy(robot_object)
    robot_ori = get_yaw_angle(robot_object)
    goal_pos = get_xy(goal_object)
    sim.step()


