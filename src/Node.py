#!/usr/bin/env python3
# license removed for brevity
# Shmulik Edelman last update

import numpy as np
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64, Float64MultiArray
from geometry_msgs.msg import Twist
import numpy as np
from Robot import Robot
import matplotlib.pyplot as plt
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

        self.robot = Robot()

        fig = plt.figure(1)

        self.ax = fig.add_subplot(111, projection='3d')

        # plt.ion()
        # self.ax = fig.gca(projection='3d', adjustable='box')
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')

        self.ax.set_xlim([-400, 400])
        self.ax.set_ylim([-400, 400])
        self.ax.set_zlim([-400, 400])

        # self.plotPath()
        # self.plotHead()
        self.ax.plot3D([0, 10], [0, 10], [0, 10], linewidth=10)
        plt.draw()
        plt.pause(0.01)
        plt.show(block=False)
        # time.sleep(0.001)
        '''
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
        '''
        rospy.Subscriber('/joy', Joy, self.joy_update)
        # self.reset_sim()
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


    def joystick_Left_X(self, state):
        print(state)


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
        if (data.axes[1]!=0 or data.axes[0]!=0):
            print("move\t{:.2f}:{:.2f}".format(data.axes[1], data.axes[0]))

            # plt.cla()
            # self.ax.clear()

            self.robot.turnHead(thetaY=data.axes[1], thetaZ=-data.axes[0])
            path = self.robot.path
            self.ax.plot(path[:, 0], path[:, 1], path[:, 2], 'b.', markersize=5)
            #
            # self.plotHead()
            headXAxis, headYAxis, headZAxis = self.robot.getHead()
            self.ax.plot(headXAxis[0, :], headXAxis[1, :], headXAxis[2, :], 'r', linewidth=3)
            self.ax.plot(headYAxis[0, :], headYAxis[1, :], headYAxis[2, :], 'g', linewidth=3)
            self.ax.plot(headZAxis[0, :], headZAxis[1, :], headZAxis[2, :], 'b', linewidth=3)

            self.ax.set_xlim([-400, 400])
            self.ax.set_ylim([-400, 400])
            self.ax.set_zlim([-400, 400])

            plt.draw()
            plt.pause(0.00001)

        # self.head_move(data.axes[1])
        if data.buttons[2] == 1:
            print("Reset Simulation")
            self.reset_sim()
        # if pygame.joystick.Joystick(joy).get_button(0) == 1:
        #     run = False
        # self.x_move = _dx * data.axes[1]
        # self.theta_move = _dTheta * data.axes[0]

    def plotPath(self):
        path = self.robot.getPath()
        # self.ax.autoscale(enable=True, axis='both', tight=True)
        # plt.plot(self.path[:, 0], self.path[:, 1], self.path[:, 2], 'b.', markersize=1)
        self.ax.plot3D(path[:, 0], path[:, 1], path[:, 2], 'b.', markersize=5)

        # plt.draw()
        # plt.pause(0.001)

    def plotHead(self):
        headXAxis, headYAxis, headZAxis = self.robot.getHead()
        print(headXAxis)
        # print(headXAxis)
        print(type(headXAxis))
        self.ax.plot3D(headXAxis[0, :], headXAxis[0, :], headXAxis[0, :], 'r', markersize=5)
        # self.ax.plot3D(headYAxis[1, :], headYAxis[1, :], headYAxis[1, :], 'g', markersize=5)
        # self.ax.plot3D(headZAxis[1, :], headZAxis[1, :], headZAxis[1, :], 'b', markersize=5)
        # plt.draw()
        # plt.pause(0.001)



if __name__ == '__main__':
    try:
        node = Node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass



