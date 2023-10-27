import math
from Robotics_Basics_In_CoppeliaSim import sim

# Coppeliasim
#   function create_shape(ints, floats, strings, bytes)
#       sim.createPrimitiveShape(sim.primitiveshape_cuboid,{0.1,0.1,0.1},0)
#       return {}, {}, {}, ''
#   end
#p = sim.simxCallScriptFunction(id, "/Generator", sim.sim_scripttype_childscript, "create_shape", {}, {}, {}, "", sim.simx_opmode_oneshot_wait)


id = sim.simxStart("127.0.0.1", 19997, True, True, 5000, 5)

if id < 0:
    print("Connection failed.")
    exit()

print("Connected.")

_, left_motor = sim.simxGetObjectHandle(id, "Left_drive", sim.simx_opmode_oneshot_wait)
_, right_motor = sim.simxGetObjectHandle(id, "Right_drive", sim.simx_opmode_oneshot_wait)
_, steering_motor = sim.simxGetObjectHandle(id, "Steering_drive", sim.simx_opmode_oneshot_wait)

_, goal_object = sim.simxGetObjectHandle(id, "Goal_point", sim.simx_opmode_oneshot_wait)
_, robot_object = sim.simxGetObjectHandle(id, "Main_frame", sim.simx_opmode_oneshot_wait)

wheel_diameter = 0.1
wheel_base = 0.3


def get_xy(object_handle):
    _, pos = sim.simxGetObjectPosition(id, object_handle, -1, sim.simx_opmode_oneshot_wait)
    return pos[0], pos[1]


def compute_vel():
    kv = 0.7
    goal_pos = get_xy(goal_object)
    robot_pos = get_xy(robot_object)

    distance = math.dist(goal_pos, robot_pos)

    velocity = kv * distance
    return velocity


def get_rel_angle():
    goal_pos = get_xy(goal_object)
    robot_pos = get_xy(robot_object)
    return math.atan2(goal_pos[1]-robot_pos[1], goal_pos[0]-robot_pos[0], )


def get_yaw_angle(object_handle):
    _, orientation = sim.simxGetObjectOrientation(id, object_handle, -1, sim.simx_opmode_oneshot_wait)
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


sim.simxStopSimulation(id, sim.simx_opmode_oneshot_wait)
sim.simxStartSimulation(id, sim.simx_opmode_oneshot_wait)

goal_pos = get_xy(goal_object)
robot_pos = get_xy(robot_object)

while math.dist(goal_pos, robot_pos) > 0.08:
    v = compute_vel()
    gamma = compute_steering()

    sim.simxSetJointTargetVelocity(id, left_motor, v, sim.simx_opmode_oneshot_wait)
    sim.simxSetJointTargetVelocity(id, right_motor, v, sim.simx_opmode_oneshot_wait)
    sim.simxSetJointTargetPosition(id, steering_motor, gamma, sim.simx_opmode_oneshot_wait)

    goal_pos = get_xy(goal_object)
    robot_pos = get_xy(robot_object)

sim.simxPauseSimulation(id, sim.simx_opmode_oneshot_wait)