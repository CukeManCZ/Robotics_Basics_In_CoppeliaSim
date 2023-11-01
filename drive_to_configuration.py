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

goal_object = sim.getObject("/Ghost/Main_frame")
robot_object = sim.getObject("/Main_frame")


def get_xy(object_handle):
    pos = sim.getObjectPosition(object_handle, sim.handle_world)
    return pos[0], pos[1]


def get_yaw_angle(object_handle):
    orientation = sim.getObjectOrientation(object_handle, sim.handle_world)
    return orientation[2]


def get_pose(object_handle):
    pos = get_xy(object_handle)
    ori = get_yaw_angle(object_handle)
    return [pos, ori]


def get_rel_angle(p1, p2):
    return math.atan2(p1[1]-p2[1], p1[0]-p2[0])


def get_angle_robot_to_goal(goal_pose, _robot_pose):
    alpha = get_rel_angle(goal_pose[0], _robot_pose[0]) - _robot_pose[1]
    return alpha


def get_angle_goal_to_global(goal_pose, _robot_pose):
    beta = -_robot_pose[1] - get_angle_robot_to_goal(goal_pose, _robot_pose)
    return beta


def controller_speed_proportional_to_dist(goal_poit, robot_point, kv=0.05):
    distance = math.dist(goal_poit, robot_point)

    velocity = kv * distance
    return velocity


def controller_ang_speed_proportional_to_orientation(goal_pose, _robot_pose, ka=3, kb=-4):
    pa = ka * get_angle_robot_to_goal(goal_pose, _robot_pose)
    pb = kb * (goal_pose[1] + get_angle_goal_to_global(goal_pose, _robot_pose))
    ang_speed = pa + pb
    return ang_speed


# velocity, angular_speed, wheelbase
def get_steering(vel, ang_vel, l_base, max_steering=50):
    steering = math.atan2(ang_vel*l_base, vel)
    print("Steering: ", math.degrees(steering))

    max_steering = math.radians(max_steering)

    if steering > max_steering:
        steering = max_steering
    elif steering < -max_steering:
        steering = -max_steering
    return steering


def deg_p_s(linear_velocity, wheel_radius):
    w = (linear_velocity/wheel_radius)
    return w


# TODO: fix backing
#       check default speed
#       better controller constants

sim.startSimulation()
L = 0.075   # m
R = 0.05     # m

robot_pose = get_pose(robot_object)
end_pose = get_pose(goal_object)

alphaA = get_angle_robot_to_goal(end_pose, robot_pose)

backing = False
if math.radians(90) < alphaA < math.radians(270):
    backing = True

while True:
    robot_pose = get_pose(robot_object)
    end_pose = get_pose(goal_object)

    v = controller_speed_proportional_to_dist(end_pose[0], robot_pose[0])
    ang_v = controller_ang_speed_proportional_to_orientation(end_pose, robot_pose)
    gamma = get_steering(v, ang_v, L)

    if backing:
        v = -v
        gamma = -gamma
        print("backing")
    else:
        print("not backing")

    sim.setJointTargetVelocity(left_motor, deg_p_s(v, R))
    sim.setJointTargetVelocity(right_motor, deg_p_s(v, R))
    sim.setJointTargetPosition(steering_motor, gamma)
    sim.step()
