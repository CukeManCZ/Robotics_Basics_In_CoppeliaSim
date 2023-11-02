from enum import Enum
import math


class LocomotionTypes(Enum):
    DIFFERENTIAL = 1
    STEERING = 2


class Robot:
    def __init__(self, coppelia_sim, locomotion_type, main_frame, wheel_radius, wheel_separation,
                 max_steering, left_motors, right_motors, middle_motors):
        self.sim = coppelia_sim
        self.locomotion_type = locomotion_type
        self.main_frame = main_frame
        self.R = wheel_radius
        self.W = wheel_separation
        self.max_steering = math.radians(max_steering)
        self.left_motors = left_motors
        self.right_motors = right_motors
        self.middle_motors = middle_motors

        self.pose = None
        self.update_pose()

    def set_left_wheels(self, angular_velocity):
        for wheel in self.left_motors:
            self.sim.setJointTargetVelocity(wheel, angular_velocity)

    def set_right_wheels(self, angular_velocity):
        for wheel in self.right_motors:
            self.sim.setJointTargetVelocity(wheel, angular_velocity)

    def set_middle_wheels(self, angular_position):
        for wheel in self.middle_motors:
            self.sim.setJointTargetPosition(wheel, angular_position)

    def stop_robot(self):
        self.set_left_wheels(0)
        self.set_right_wheels(0)
        self.set_middle_wheels(0)

    def get_position(self):
        return self.pose[0]

    def get_2d_position(self):
        return [self.pose[0][0], self.pose[0][1]]

    def get_orientation(self):
        return self.pose[1]

    def get_pose(self):
        return self.pose

    def update_pose(self):
        position = self.sim.getObjectPosition(self.main_frame, self.sim.handle_world)
        orientation = self.sim.getObjectOrientation(self.main_frame, self.sim.handle_world)
        new_pose = [position, orientation]
        self.pose = new_pose

    def drive_to_point(self, point_xy, kv, kh):
        self.update_pose()
        if self.locomotion_type == LocomotionTypes.STEERING:
            v = controller_speed_proportional_to_dist(point_xy, self.get_2d_position(), kv)
            w = linear_to_angular_vel(v, self.R)

            gamma = controller_steer_to_point(point_xy, self.get_2d_position(), self.get_orientation()[2], kh)
            if gamma > self.max_steering:
                gamma = self.max_steering
            elif gamma < -self.max_steering:
                gamma = -self.max_steering

            self.set_left_wheels(w)
            self.set_right_wheels(w)
            self.set_middle_wheels(gamma)

        elif self.locomotion_type == LocomotionTypes.DIFFERENTIAL:
            linear_vel = controller_speed_proportional_to_dist(point_xy, self.get_2d_position(), kv)
            steer_vel = controller_steer_to_point(point_xy, self.get_2d_position(), self.get_orientation()[2], kh)

            v_l, v_r = get_left_right_vel(linear_vel, steer_vel, self.W)
            w_l = linear_to_angular_vel(v_l, self.R)
            w_r = linear_to_angular_vel(v_r, self.R)

            self.set_left_wheels(w_l)
            self.set_right_wheels(-w_r)

        else:
            print("Not implemented")

    def drive_to_line(self, line_abc, v, kd=0.5, kh=1):
        self.update_pose()
        if self.locomotion_type == LocomotionTypes.STEERING:
            gamma = controller_steer_to_line(line_abc, self.get_2d_position(), self.get_orientation()[2], kd, kh)
            w = linear_to_angular_vel(v, self.R)

            if gamma > self.max_steering:
                gamma = self.max_steering
            elif gamma < -self.max_steering:
                gamma = -self.max_steering

            self.set_left_wheels(w)
            self.set_right_wheels(w)
            self.set_middle_wheels(gamma)

        # TODO position update, currently drives asymmetrically
        elif self.locomotion_type == LocomotionTypes.DIFFERENTIAL:
            linear_vel = linear_to_angular_vel(v, self.R)
            steer_vel = controller_steer_to_line(line_abc, self.get_2d_position(), self.get_orientation()[2], kd, kh)

            v_l, v_r = get_left_right_vel(linear_vel, steer_vel, self.W)
            w_l = linear_to_angular_vel(v_l, self.R)
            w_r = linear_to_angular_vel(v_r, self.R)

            self.set_left_wheels(w_l)
            self.set_right_wheels(-w_r)
        else:
            print("Not implemented")

    def drive_to_pose(self, pose, kv, ka, kb):
        self.update_pose()
        if self.locomotion_type == LocomotionTypes.STEERING:
            pass
        elif self.locomotion_type == LocomotionTypes.DIFFERENTIAL:
            pass
        else:
            print("Not implemented")

    def drive_throughout_points(self, points, d, vel, kh):
        last_point = 0
        while True:
            self.update_pose()
            # find the nearest point
            j = nearest_point(points, last_point, self.get_2d_position())

            # check case if robot needs to go throughout points
            if j < last_point:
                j = last_point
            if j > last_point:
                last_point = j

            # find the first point k along the path that is at least a distance d ahead of j
            k = j
            for i_point in range(j + 1, len(points)):
                distance = math.dist(points[i_point], points[j])
                if distance >= d:
                    k = i_point
                    break

            if self.locomotion_type == LocomotionTypes.STEERING:
                w = linear_to_angular_vel(vel, self.R)
                gamma = controller_steer_to_point(points[k], self.get_2d_position(), self.get_orientation()[2], kh)
                if gamma > self.max_steering:
                    gamma = self.max_steering
                elif gamma < -self.max_steering:
                    gamma = -self.max_steering

                self.set_left_wheels(w)
                self.set_right_wheels(w)
                self.set_middle_wheels(gamma)
            elif self.locomotion_type == LocomotionTypes.DIFFERENTIAL:
                linear_vel = linear_to_angular_vel(vel, self.R)
                steer_vel = controller_steer_to_point(points[k], self.get_2d_position(), self.get_orientation()[2], kh)

                v_l, v_r = get_left_right_vel(linear_vel, steer_vel, self.W)
                w_l = linear_to_angular_vel(v_l, self.R)
                w_r = linear_to_angular_vel(v_r, self.R)

                self.set_left_wheels(w_l)
                self.set_right_wheels(-w_r)
            else:
                print("Not implemented")

            if points[k] == points[-1]:
                if math.dist(points[k], self.get_2d_position()) < 0.01:
                    self.stop_robot()
                    break
            self.sim.step()
        print("Done")


def linear_to_angular_vel(linear_velocity, radius):
    return linear_velocity/radius


# for differential locomotion
def get_left_right_vel(forward_velocity, steering_velocity, wheel_dist):
    right_velocity = forward_velocity + (wheel_dist / 2) * steering_velocity
    left_velocity = forward_velocity - (wheel_dist / 2) * steering_velocity

    return left_velocity, right_velocity


def get_rel_angle(p1, p2):
    return math.atan2(p1[1]-p2[1], p1[0]-p2[0])


def controller_speed_proportional_to_dist(goal_poit, robot_point, kv=0.7):
    distance = math.dist(goal_poit, robot_point)

    velocity = kv * distance
    return velocity


# ********************* point ********************
def controller_steer_to_point(goal_point, position, orientation, kh=1):
    diff = get_rel_angle(goal_point, position) - orientation
    return kh * diff


# ********************* line *********************
def normal_dist_from_line(line_abc, point):
    numerator = line_abc[0] * point[0] + line_abc[1]*point[1] + line_abc[2] * 1
    denominator = math.sqrt(line_abc[0]**2 + line_abc[1]**2)
    return numerator/denominator


def line_slope_angle(line_abc):
    return math.atan2(line_abc[0], -line_abc[1])


def create_line(point_a, point_b):
    k_a = point_b[1] - point_a[1]
    k_b = point_a[0] - point_b[0]
    k_c = - k_a * point_a[0] - k_b * point_a[1]
    return k_a, k_b, k_c


def controller_steer_to_line(line_abc, position, orientation, kd=0.5, kh=1):
    d = normal_dist_from_line(line_abc, position)
    gamma_x = line_slope_angle(line_abc)

    steering = kd*d + kh*(gamma_x - orientation)

    return steering


# ********************* points *********************
def nearest_point(path, last_point, robot_position):
    min_dist = -1
    j = 0
    for i_point in range(last_point, len(path)):
        if min_dist == -1:
            min_dist = math.dist(path[i_point], robot_position)
            continue
        distance = math.dist(path[i_point], robot_position)
        if distance < min_dist:
            min_dist = distance
            j = i_point
    return j
