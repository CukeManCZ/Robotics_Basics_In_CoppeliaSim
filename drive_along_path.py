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

goal_path = "/Path"
robot_object = sim.getObject("/Main_frame")


def get_points_from_path(path_obj, point_name, num):
    points = []
    for i in range(num):
        point_addr = path_obj + '/' + point_name + '[' + str(i) + ']'
        point_obj = sim.getObject(point_addr)
        points.append(get_xy(point_obj))

    return points


def get_xy(object_handle):
    pos = sim.getObjectPosition(object_handle, sim.handle_world)
    return pos[0], pos[1]


def get_yaw_angle(object_handle):
    orientation = sim.getObjectOrientation(object_handle, sim.handle_world)
    return orientation[2]


def get_rel_angle(p1, p2):
    return math.atan2(p1[1]-p2[1], p1[0]-p2[0])


def controller_steer_to_point(goal_point, position, orientation, kh=1, max_steering=55):
    diff = get_rel_angle(goal_point, position) - orientation

    max_steering = math.radians(max_steering)

    if diff > max_steering:
        diff = max_steering
    elif diff < -max_steering:
        diff = -max_steering
    return kh * diff


def nearest_point(path, last_point, robo_obj):
    min_dist = -1
    j = 0
    robot_pos = get_xy(robo_obj)
    for i_point in range(last_point, len(path)):
        if min_dist == -1:
            min_dist = math.dist(path[i_point], robot_pos)
            continue
        distance = math.dist(path[i_point], robot_pos)
        if distance < min_dist:
            min_dist = distance
            j = i_point
    return j


def drive_along_path(path, robo_obj, d, vel):
    last_point = 0
    while True:
        # find the nearest point
        j = nearest_point(path, last_point, robo_obj)

        # check case if robot needs to go throughout points
        if j < last_point:
            j = last_point
        if j > last_point:
            last_point = j

        # find the first point k along the path that is at least a distance d ahead of j
        k = j
        for i_point in range(j+1, len(path)):
            distance = math.dist(path[i_point], path[j])
            if distance >= d:
                k = i_point
                break

        # drive to point k
        robot_pos = get_xy(robo_obj)
        robot_ori = get_yaw_angle(robo_obj)
        steering = controller_steer_to_point(path[k], robot_pos, robot_ori)

        sim.setJointTargetVelocity(left_motor, vel)
        sim.setJointTargetVelocity(right_motor, vel)
        sim.setJointTargetPosition(steering_motor, steering)

        sim.step()

        print("Current point: ", k, path[k])
        # check for finish
        if path[k] == path[-1]:
            if math.dist(path[k], robot_pos) < 0.01:
                sim.setJointTargetVelocity(left_motor, 0)
                sim.setJointTargetVelocity(right_motor, 0)
                sim.setJointTargetPosition(steering_motor, 0)
                break


sim.startSimulation()

goal_points = get_points_from_path(goal_path, "ctrlPt", 10)

drive_along_path(goal_points, robot_object, 0.01, 1.5)
sim.stopSimulation()
