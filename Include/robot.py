from enum import Enum
import math


class LocomotionTypes(Enum):
    DIFFERENTIAL = 1
    STEERING = 2


class Robot:
    def __init__(self, coppelia_sim, locomotion_type, main_frame, wheel_radius, wheel_base, wheel_separation,
                 max_steering, left_motors, right_motors, middle_motors):
        self.sim = coppelia_sim
        self.locomotion_type = locomotion_type
        self.main_frame = main_frame
        self.R = wheel_radius
        self.L = wheel_base
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
            def get_left_right_vel(forward_velocity, steering_velocity, wheel_dist):
                right_velocity = forward_velocity + (wheel_dist / 2) * steering_velocity
                left_velocity = forward_velocity - (wheel_dist / 2) * steering_velocity

                return left_velocity, right_velocity

            linear_vel = controller_speed_proportional_to_dist(point_xy, self.get_2d_position(), kv)
            steer_vel = controller_steer_to_point(point_xy, self.get_2d_position(), self.get_orientation()[2], kh)

            v_l, v_r = get_left_right_vel(linear_vel, steer_vel, self.W)
            w_l = linear_to_angular_vel(v_l, self.R)
            w_r = linear_to_angular_vel(v_r, self.R)

            self.set_left_wheels(w_l)
            self.set_right_wheels(-w_r)

        else:
            print("Not implemented")

    def drive_to_line(self, line_abc, kd, kh):
        self.update_pose()
        if self.locomotion_type == LocomotionTypes.STEERING:
            pass
        elif self.locomotion_type == LocomotionTypes.DIFFERENTIAL:
            pass
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

    def drive_throughout_points(self, points):
        self.update_pose()
        if self.locomotion_type == LocomotionTypes.STEERING:
            pass
        elif self.locomotion_type == LocomotionTypes.DIFFERENTIAL:
            pass
        else:
            print("Not implemented")


def get_rel_angle(p1, p2):
    return math.atan2(p1[1]-p2[1], p1[0]-p2[0])


def controller_speed_proportional_to_dist(goal_poit, robot_point, kv=0.7):
    distance = math.dist(goal_poit, robot_point)

    velocity = kv * distance
    return velocity


def controller_steer_to_point(goal_point, position, orientation, kh=1):
    diff = get_rel_angle(goal_point, position) - orientation
    return kh * diff


def linear_to_angular_vel(linear_velocity, radius):
    return linear_velocity/radius
