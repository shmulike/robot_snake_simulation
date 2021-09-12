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


        while not rospy.is_shutdown():


            self.rate.sleep()



    def headTurn(self, state):



    def joy_update(self, data):
        """callback for /joy
        * sends commands to the simulation
        Args:
            data (Joy): the joystick's state
            Left stick - right left - data.axes[0]
            Left stick - up down - data.axes[1]
            Button - 'X' - data.burrons[2]
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
