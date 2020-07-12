#!/usr/bin/env python3
# license removed for brevity
# Shmulik Edelman last update 15.6.20

import numpy as np
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64, Float64MultiArray
from geometry_msgs.msg import Twist
from math import pi, sqrt
import numpy as np
import scipy.interpolate as sc  # interp1d, CubicSpline, splprep, splev
import time

# All dimentions are in mm
# _dx = 0.05
# _dTheta = 0.1
_node_name = 'joy_listener'
_cmd_topic = "controlCMD"
_Hz = 100
_N = 1000
_joy_dead_zone = 0.12
_joint_limit = np.deg2rad(40)
_head_L = 0.2  # meter
_head_init_x = 160*9
_head_init_y = 0


class Node:
    rospy.init_node(_node_name, anonymous=True)
    rate = rospy.Rate(_Hz)

    def reset_sim(self):
        for i in range(self._link_N + 1):
            self.pub_vec[i].publish(Float64(data=0))
        self.theta_move = 0
        self.head_angle = 0
        # self.head_move = 0
        self.A_head = np.array([[1, 0, _head_init_x],
                                  [0, 1, _head_init_y],
                                  [0, 0, 1]])
        # self.path = np.array([np.linspace(-self._link_L * self._link_N * 2, 0, self._N, endpoint=True),
        #                         np.zeros(self._N),
        #                         np.ones(self._N)])
        self.path = np.array([np.linspace(-_head_init_x, _head_init_x, self._N, endpoint=True),
                              np.zeros(self._N),
                              np.ones(self._N)])

    def __init__(self):

        self._dtheta = 0.05
        self._dx = 10
        self._link_L = 160
        self._link_N = 10
        self._N = 200
        self.Kp_curve = 0.00041

        self.pub_vec = [rospy.Publisher('/snake_10/linear_position_controller/command', Float64, queue_size=10),
                        rospy.Publisher('/snake_10/joint1_position_controller/command', Float64, queue_size=10),
                        rospy.Publisher('/snake_10/joint2_position_controller/command', Float64, queue_size=10),
                        rospy.Publisher('/snake_10/joint3_position_controller/command', Float64, queue_size=10),
                        rospy.Publisher('/snake_10/joint4_position_controller/command', Float64, queue_size=10),
                        rospy.Publisher('/snake_10/joint5_position_controller/command', Float64, queue_size=10),
                        rospy.Publisher('/snake_10/joint6_position_controller/command', Float64, queue_size=10),
                        rospy.Publisher('/snake_10/joint7_position_controller/command', Float64, queue_size=10),
                        rospy.Publisher('/snake_10/joint8_position_controller/command', Float64, queue_size=10),
                        rospy.Publisher('/snake_10/joint9_position_controller/command', Float64, queue_size=10),
                        rospy.Publisher('/snake_10/joint10_position_controller/command', Float64, queue_size=10)]

        self.pub = rospy.Publisher(_cmd_topic, Twist, queue_size=10)
        rospy.Subscriber('/joy', Joy, self.joy_update)
        self.reset_sim()
        # self.joint_vals = Float32MultiArray()

        # self.joint_vals.data[0] = [0]
        # for i in range(self._link_N):
        #     self.joint_vals.data[i + 1] = 0

        # self.angle_range = np.linspace(-np.pi / 2, np.pi, _N)
        # self.head = Twist()
        # self.head.linear.x = 0
        # self.head.angular.z = 0
        # self.x_move = 0
        # self.theta_move = 0

        while not rospy.is_shutdown():
            # self.head.linear.x = 0
            # self.head.angular.z = 0

            # self.head.linear.x += self.x_move
            # self.head.angular.z += self.theta_move
            # self.pub.publish(self.head)
            #
            # self.pub_vec[0].publish(self.head.linear.x)
            # self.pub_vec[1].publish(self.head.angular.z)

            self.rate.sleep()

    def calc_angle(self, joint_pos):
        joint_angles = np.zeros(self._link_N)
        for i in range(1, self._link_N):
            # calculate 2 vectors from 3 points
            vec1 = (joint_pos[0:2, i] - joint_pos[0:2, i - 1])
            vec2 = (joint_pos[0:2, i + 1] - joint_pos[0:2, i])
            # calculate the angle between 2 vectors vec1 <-> vec2
            joint_angles[i - 1] = np.arctan2(np.cross(vec1, vec2), np.dot(vec1, vec2))
        return joint_angles

    def split_curve_2(self, path_x, path_y):
        # We will use the 'Bisection method' to find the joint position along the spline
        # https://en.wikipedia.org/wiki/Bisection_method

        counter = 0  # count iterations for log
        epsilon = 1e-3  # 0.001 mm
        joint_pos = np.zeros([2, self._link_N + 1])  # Create joint positions matrix

        # save the position of the true head
        joint_pos[0, 0] = path_x[-1]
        joint_pos[1, 0] = path_y[-1]

        # add head direction points for the spline
        head_dir = np.asarray([[500], [0], [1]])
        p_new = self.A_head * head_dir
        path_x_new = np.append(path_x, np.linspace(path_x[-1] + self._dx / 2, p_new[0, 0], 100))
        path_y_new = np.append(path_y, np.linspace(path_y[-1], p_new[1, 0], 100))

        # reverse the points to be end->start, so the spline u=0 will be at the head
        path_x_new = np.flip(path_x_new)
        path_y_new = np.flip(path_y_new)

        # rospy.loginfo("\npoints X new:\n")
        # rospy.loginfo(points_x_new)
        # rospy.loginfo(np.diff(points_x_new))
        # rospy.loginfo("\npoints Y new:\n")
        # rospy.loginfo(points_y_new)
        # rospy.loginfo(np.diff(points_y_new))
        # points_x_new = np.flip(points_x_new)
        # points_y_new = np.flip(points_y_new)

        # Create spline
        tck, U = sc.splprep([path_x_new, path_y_new], s=0)

        # a, b, c - 3 parameters we use for the 'Bisection method'
        a = c = 0
        b = 1
        u = 1
        for i in range(self._link_N-1):
            # rospy.loginfo("link {:d}".format(i))
            # reset parameters for each link
            error = error = self._link_L
            a = c
            b = 1
            while error >= epsilon:
                # rospy.loginfo(error)
                counter += 1
                c = (a + b) / 2
                p_temp = np.reshape(sc.splev(c, tck), 2)  # calc temp point along the spline
                dist = np.linalg.norm(
                    joint_pos[:, i] - p_temp)  # calc the distance from the temp point to the previous joint position
                error = abs(dist - self._link_L)  # Update links length error

                # Update a,b,c parameters
                if dist > self._link_L:
                    b = c
                else:
                    a = c

            rospy.loginfo(p_temp)
            joint_pos[:, i + 1] = p_temp
            time.sleep(0.2)
        rospy.loginfo(joint_pos)
        joint_pos = np.fliplr(joint_pos)  # flip the matrix on lef-right so the first joint will be at the beginning of the matrix
        rospy.loginfo(joint_pos)
        # Calculate the angles at the joints and the first linear rail distance
        # First angle
        vec1 = np.array([[1], [0]])
        vec2 = joint_pos[0:2, 1] - joint_pos[0:2, 0]
        # angle = np.math.acos(np.dot(v0, v1) / (np.linalg.norm(v0) * np.linalg.norm(v1)))
        first_angle = np.math.atan2(vec2[1], vec2[0]) - np.math.atan2(vec1[1], vec1[0])

        joint_angles = np.array([first_angle])
        joint_angles = np.append(joint_angles, self.calc_angle(joint_pos))

        joint_cmd = np.append(joint_pos[0, 0]*(1e-3), first_angle)      # (1e-3) convert from mm to meter
        joint_cmd = np.append(joint_cmd, joint_angles)
        rospy.loginfo("joint_cmd: {:.3f}".format(joint_cmd[0]))


        return joint_cmd

    def split_curve(self, points_x, points_y, link_N, link_L):
        counter = 0
        epsilon = 1e-2  # Allowed error = 0.01 mm
        e = link_L
        px0 = points_x[-1]
        py0 = points_y[-1]
        px1 = py1 = 0
        tck, u = sc.splprep([points_x, points_y], k=2, s=0)
        joints_points = [px0, py0, 1]
        u = 1
        for i in range(link_N):
            e = 1
            while abs(e) > epsilon:
                px1, py1 = sc.splev(u, tck)
                L_current = sqrt((px1 - px0) ** 2 + (py1 - py0) ** 2)
                e = link_L - L_current
                u -= e * self.Kp_curve
                counter += 1

            u -= 0.001
            self.Kp_curve *= 0.999
            vec = [px1, py1, 1]
            joints_points = np.vstack([joints_points, vec])
            px0 = px1
            py0 = py1

        # print("Counter {:.2f}".format(counter/link_N))
        joints_points = joints_points[::-1]

        # First angle
        v0 = [1, 0]
        v1 = joints_points[1, 0:2] - joints_points[0, 0:2]
        # angle = np.math.acos(np.dot(v0, v1) / (np.linalg.norm(v0) * np.linalg.norm(v1)))
        angle = np.math.atan2(v1[1], v1[0]) - np.math.atan2(v0[1], v0[0])
        joints_angles = np.asarray([angle])

        for i in range(1, link_N):
            v0 = (joints_points[i, 0:2] - joints_points[i - 1, 0:2])
            v1 = (joints_points[i + 1, 0:2] - joints_points[i, 0:2])
            angle = np.arctan2(np.cross(v0, v1), np.dot(v0, v1))
            joints_angles = np.concatenate((joints_angles, [angle]))

        joints_points = (joints_points + 1600) / 1000
        return joints_points[:, 0], joints_angles, tck

    def stretch(self, x, in_min, in_max, out_min, out_max):
        x, in_min, in_max, out_min, out_max = map(float, (x, in_min, in_max, out_min, out_max))
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def Rz(self, x, theta):
        return np.asarray([[np.cos(theta), -np.sin(theta), np.cos(theta) * x],
                           [np.sin(theta), np.cos(theta), np.sin(theta) * x],
                           [0, 0, 1]])

    def head_turn(self, state):
        if abs(state) > _joy_dead_zone and abs(self.head_angle + self._dtheta * (-state)) < _joint_limit:
            self.theta_move = self._dtheta * (-state)
            self.head_angle += self.theta_move
            vec_view = [[500], [0], [1]]
            # self.R_z = self.Rz(0, self.theta_move)
            self.A_head = np.dot(self.A_head, self.Rz(0, self.theta_move))  # Update A_hed matrix
            # head_direction = np.dot(self.A_head, vec_view)  # Get new head Direction
            # Start section print and plot of the snake:
            # Remove old plot elements

            self.pub_vec[self._link_N].publish(Float64(data=self.head_angle))

    def joystick_Left_X(self, state):
        print(state)

    def head_move(self, state):
        if state > _joy_dead_zone:
            # x_move = self._dx * self.stretch(state, 0, 2**15, 0, 1)
            x_move = self._dx * state
            vec0 = [[0], [0], [1]]
            vec_view = [[500], [0], [1]]
            self.R_z = np.array(
                [[np.cos(self.theta_move), -np.sin(self.theta_move), np.cos(self.theta_move) * x_move],
                 [np.sin(self.theta_move), np.cos(self.theta_move), np.sin(self.theta_move) * x_move],
                 [0, 0, 1]])
            self.A_head = np.dot(self.A_head, self.R_z)  # Update A_hed matrix

            self.head_point_new = np.dot(self.A_head, vec0)  # Get new head position
            # head_direction = np.dot(self.A_head, vec_view)          # Get new head Direction



            self.path = np.concatenate((self.path, self.head_point_new), axis=1)  # Add new head position to path
            x = np.squeeze(np.asarray(self.path[0, :]))
            y = np.squeeze(np.asarray(self.path[1, :]))

            joint_cmd = self.split_curve_2(x, y)
            self.theta_move = 0


            print("Origin pos: {:.3f}".format(joint_cmd[0]))
            rospy.loginfo(joint_cmd)
            rospy.loginfo(joint_cmd.shape)
            for i in range(self._link_N + 1):
                rospy.loginfo("joint {:d}\t{:.3f}".format(i, joint_cmd[i]))
                self.pub_vec[i].publish(Float64(data=joint_cmd[i]))


    def joy_update(self, data):
        """callback for /joy
        * sends commands to the simulation
        Args:
            data (Joy): the joystick's state
            Left stick - right left - data.axes[0]
            Left stick - up down - data.axes[1]
        """
        # print("joy_update: {:.2f}".format(data.axes[1]))
        self.head_turn(-data.axes[0])
        self.head_move(data.axes[7])
        # self.head_move(data.axes[1])
        if data.buttons[2] == 1:
            print("Reset Simulation")
            self.reset_sim()
        # if pygame.joystick.Joystick(joy).get_button(0) == 1:
        #     run = False
        # self.x_move = _dx * data.axes[1]
        # self.theta_move = _dTheta * data.axes[0]


if __name__ == '__main__':
    try:
        Node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
