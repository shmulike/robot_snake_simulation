#!/usr/bin/env python3
# Shmulik Edelman last update

import numpy as np
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import Twist, Transform, Wrench
# from sensor_msgs.msg import MultiDOFJointState
# from geometry_msgs.msg import Transform
import numpy as np
from Robot_singleRevJoint import Robot
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation
# from scipy.linalg import norm

# All dimentions are in mm
_node_name = 'joy_listener'
# _cmd_topic = "controlCMD"
_Hz = 100
# _N = 100
_PLOT = True


class Node:
    rospy.init_node(_node_name, anonymous=True)
    rate = rospy.Rate(_Hz)

    def __init__(self):
        self._dtheta = 0.01  # Dimention in rad
        self._dx = 0.1  # Dimention in mm
        self._link_L = 160
        self._link_N = 10
        # self._N = 200
        self._head_init_x = self._link_L * (self._link_N - 1)
        self._head_init_y = 0
        self._joint_limit = np.deg2rad(40)
        self._joy_dead_zone = 0.12
        self._linear_cmd_offset = self._link_L*(self._link_N-1)
        self.reset_sim()

        rospy.Subscriber('/joy', Joy, self.joy_update)
        # self.pub_joint_cmd = rospy.Publisher('/robot_snake_sim/joint_ang_cmd/', MultiDOFJointState, queue_size=10)
        self.pub_joint_cmd = rospy.Publisher('/robot_snake_10/joint_cmd/', Float32MultiArray, queue_size=10)
        self.pub_linear_cmd = rospy.Publisher('/robot_snake_10/linear_cmd/', Float32, queue_size=10)

        # initiate plot
        fig = plt.figure()
        ax = p3.Axes3D(fig)
        ax.view_init(elev=35, azim=-150)

        headAxis_1 = self.robot.head_axis_1
        headAxis_2 = self.robot.head_axis_2
        # Creating fifty line objects.
        # NOTE: Can't pass empty arrays into 3d version of plot()
        axes_color = ["red", "green", "blue"]
        head_lines_1 = [ax.plot(dat[0, 0:1], dat[1, 0:1], dat[2, 0:1])[0] for dat in headAxis_1]
        head_lines_2 = [ax.plot(dat[0, 0:1], dat[1, 0:1], dat[2, 0:1])[0] for dat in headAxis_2]
        for color, line_1, line_2 in zip(axes_color, head_lines_1, head_lines_2):
            line_1.set_color(color)
            line_2.set_color(color)

        # for color, line_1 in zip(axes_color, head_lines_1):
        #     line_1.set_color(color)
        # for color, line_2 in zip(axes_color, head_lines_2):
        #     line_2.set_color(color)

        path_lines, = ax.plot(self.robot.path[0, :], self.robot.path[1, :], self.robot.path[2, :], 'k.', markersize=0.01)
        joint_lines, = ax.plot(self.robot.path[0, :], self.robot.path[1, :], self.robot.path[2, :], '--or')
        joint_lines_odd, = ax.plot(self.robot.joint_pos[0, :], self.robot.joint_pos[1, :], self.robot.joint_pos[2, :], '--og')
        joint_lines_even, = ax.plot(self.robot.joint_pos[0, :], self.robot.joint_pos[1, :], self.robot.joint_pos[2, :], '--og')
        # joint_recon_lines, = ax.plot(path[0, :], path[1, :], path[2, :], 'sg')

        ax.set_xlim3d([0, 2000])
        ax.set_ylim3d([-1000, 1000])
        ax.set_zlim3d([-1000, 1000])
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        ax.set_title('3D Test')

        print("start")
        if _PLOT:
            line_ani = animation.FuncAnimation(fig, func=self.update_lines, frames=100,
                                               fargs=(
                                               head_lines_1, head_lines_2, path_lines, joint_lines, joint_lines_odd,
                                               joint_lines_even), interval=5,
                                               blit=False)
            plt.show()

        while not rospy.is_shutdown():
            self.pub_joint_cmd.publish(Float32MultiArray(data=self.robot.joint_cmd[1:]))
            self.pub_linear_cmd.publish(Float32(data=(self.robot.joint_cmd[0] + self._linear_cmd_offset)))
            self.rate.sleep()

    def reset_sim(self):
        """Reset simulation"""
        self.robot = Robot(self._link_N, self._link_L, self._dtheta, self._dtheta, self._dx)


    def joy_update(self, data):
        """callback for /joy
        * sends commands to the simulation
        Args:
            data (Joy): the joystick's state
            Left stick - right left - data.axes[0]
            Left stick - up down - data.axes[1]
            Button - 'X' - data.buttons[2]
        """
        # On joystick move send the command to to robot.move_head
        if data.axes[0] != 0 or data.axes[1] != 0 or data.axes[4] != 0:
            self.robot.move_head(thetaY=data.axes[1], thetaZ=data.axes[0], forward=data.axes[4])

        # Reset the robot on press 'X' button
        if data.buttons[2] == 1:
            print("Reset Simulation")
            self.reset_sim()

        if _PLOT:
            """
            Plot and animation are blocking the code
            If plot is True that publish the angles here
            Otherwise publish in: __init__ -> while not rospy.is_shutdown
            """
            self.pub_joint_cmd.publish(Float32MultiArray(data=self.robot.joint_cmd[1:]))
            self.pub_linear_cmd.publish(Float32(data=(self.robot.joint_cmd[0] + self._linear_cmd_offset)))

    def update_lines(self, frames, head_lines_1, head_lines_2, path_lines, joint_lines, joint_lines_odd, joint_lines_even):
        # Plot head_1 (back) coordinate system
        dataLines_1 = self.robot.head_axis_1
        for line_1, data_1 in zip(head_lines_1, dataLines_1):
            line_1.set_data(data_1[0:2, :])
            line_1.set_3d_properties(data_1[2, :])

        # Plot head_2 (Front) coordinate system
        dataLines_2 = self.robot.head_axis_2
        for line_2, data_2 in zip(head_lines_2, dataLines_2):
            line_2.set_data(data_2[:2, :])
            line_2.set_3d_properties(data_2[2, :])

        # Plot the robot path
        path_lines.set_data(self.robot.path[0:2, :])
        path_lines.set_3d_properties(self.robot.path[2, :])

        # Plot links with lines
        joint_lines.set_data(self.robot.joint_pos[:2, :])
        joint_lines.set_3d_properties(self.robot.joint_pos[2, :])

        # Plot the two links in one line,
        joint_lines_odd.set_data(self.robot.joint_pos[:2, ::2])
        joint_lines_odd.set_3d_properties(self.robot.joint_pos[2, ::2])
        joint_lines_even.set_data(self.robot.joint_pos[:2, 1::2])
        joint_lines_even.set_3d_properties(self.robot.joint_pos[2, 1::2])

        # Plot the reconstructed joints with forward kinematics, to check the angle calculation
        # joint_pos_recon = self.robot.joint_pos_recon
        # joint_recon_lines.set_data(joint_pos_recon[0:2, :])
        # joint_recon_lines.set_3d_properties(joint_pos_recon[2, :])
        return head_lines_1, head_lines_2, path_lines


if __name__ == '__main__':
    try:
        node = Node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
