#!/usr/bin/env python3
# license removed for brevity
# Shmulik Edelman last update

import numpy as np
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64, Float64MultiArray
from geometry_msgs.msg import Twist, Transform, Wrench
from sensor_msgs.msg import MultiDOFJointState
#from geometry_msgs.msg import Transform
import numpy as np
from Robot_v2 import Robot
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation
from scipy.linalg import norm

# All dimentions are in mm
# _dx = 0.05
# _dTheta = 0.1
_node_name = 'joy_listener'
_cmd_topic = "controlCMD"
_Hz = 100
_N = 100
_joy_dead_zone = 0.12
_joint_limit = np.deg2rad(40)
_head_L = 0.2  # meter
_head_init_x = 160 * 9
_head_init_y = 0


class Node:
    rospy.init_node(_node_name, anonymous=True)
    rate = rospy.Rate(_Hz)

    def reset_sim(self):

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

        self.robot = Robot()
        self.joint_cmd = MultiDOFJointState()

    def __init__(self):
        np.random.seed(19680801)

        self._dtheta = 0.05
        self._dx = 10
        self._link_L = 160
        self._link_N = 12
        self._N = 200
        # self.Kp_curve = 0.00041

        # moved to reset_sim()
        # self.robot = Robot()
        # self.joint_cmd = MultiDOFJointState()
        self.reset_sim()

        rospy.Subscriber('/joy', Joy, self.joy_update)
        self.pub_joint_cmd = rospy.Publisher('/robot_snake_sim/joint_ang_cmd/', MultiDOFJointState, queue_size=10)

        ### initiate plot
        fig = plt.figure()
        ax = p3.Axes3D(fig)
        ax.view_init(elev=35, azim=-150)
        # data = [self.Gen_RandLine(25, 3)]
        # print(data)


        data = self.robot.head_axis
        # Creating fifty line objects.
        # NOTE: Can't pass empty arrays into 3d version of plot()
        axes_color = ["red", "green", "blue"]
        head_lines = [ax.plot(dat[0, 0:1], dat[1, 0:1], dat[2, 0:1])[0] for dat in data]
        for color, line in zip(axes_color, head_lines):
            line.set_color(color)
        path = self.robot.path

        path_lines, = ax.plot(path[0, :], path[1, :], path[2, :], 'k.', markersize=0.1)
        joint_lines, = ax.plot(path[0, :], path[1, :], path[2, :], '--or')
        # joint_recon_lines, = ax.plot(path[0, :], path[1, :], path[2, :], 'sg')

        ax.set_xlim3d([0, 2000])
        ax.set_ylim3d([-1000, 1000])
        ax.set_zlim3d([-1000, 1000])
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        ax.set_title('3D Test')

        # line_ani = animation.FuncAnimation(fig, func=self.update_lines, frames=100, fargs=(head_lines, path_lines, joint_lines, joint_recon_lines), interval=1, blit=False)
        # line_ani = animation.FuncAnimation(fig, func=self.update_lines, frames=100, fargs=(head_lines, path_lines, joint_lines), interval=5, blit=False)
        # plt.show()

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
            Button - 'X' - data.buttons[2]
        """
        # print("joy_update: {:.2f}".format(data.axes[1]))

        if (data.axes[0] != 0 or data.axes[1] != 0 or data.axes[4] != 0):
            # self.robot.move_head(thetaY=data.axes[1], thetaZ=data.axes[0], forward=data.axes[4])
            # self.robot.move_head(thetaY=data.axes[1], thetaZ=data.axes[0], forward=data.axes[4])
            self.robot.move_head(thetaY=data.axes[1], thetaZ=data.axes[0], forward=data.axes[4])

            CMD = MultiDOFJointState()

            trans_i = Transform()
            trans_i.translation.x = self.robot.joint_cmd[0]
            twist_i = Twist()
            wrench_i = Wrench()
            CMD.transforms = [trans_i]
            CMD.twist = [twist_i]
            CMD.wrench = [wrench_i]

            for i in range(self._link_N):
                trans_i = Transform()
                trans_i.rotation.z = self.robot.joint_ang[i*2]
                trans_i.rotation.y = self.robot.joint_ang[i*2+1]
                CMD.transforms.append(trans_i)

                twist_i = Twist()
                CMD.twist.append(twist_i)

                wrench_i = Wrench()
                CMD.wrench.append(wrench_i)
            self.pub_joint_cmd.publish(CMD)



        if data.buttons[2] == 1:
            print("Reset Simulation")
            self.reset_sim()
        # if pygame.joystick.Joystick(joy).get_button(0) == 1:
        #     run = False
        # self.x_move = _dx * data.axes[1]
        # self.theta_move = _dTheta * data.axes[0]

    def link_body(self):
        # p0 = np.array([1, 3, 2])
        # p1 = np.array([8, 5, 9])
        p0 = self.robot.joint_pos[0:3, -1]
        p1 = self.robot.joint_pos[0:3, -2]
        R = 60/2
        # vector in direction of axis
        v = p1 - p0
        # find magnitude of vector
        mag = norm(v)
        # unit vector in direction of axis
        v = v / mag
        # make some vector not in the same direction as v
        not_v = np.array([1, 0, 0])
        if (v == not_v).all():
            not_v = np.array([0, 1, 0])
        # make vector perpendicular to v
        n1 = np.cross(v, not_v)
        # normalize n1
        n1 /= norm(n1)
        # make unit vector perpendicular to v and n1
        n2 = np.cross(v, n1)
        # surface ranges over t from 0 to length of axis and 0 to 2*pi
        t = np.linspace(0, mag, 100)
        theta = np.linspace(0, 2 * np.pi, 100)
        # use meshgrid to make 2d arrays
        t, theta = np.meshgrid(t, theta)
        # generate coordinates for surface
        X, Y, Z = [p0[i] + v[i] * t + R * np.sin(theta) * n1[i] + R * np.cos(theta) * n2[i] for i in [0, 1, 2]]
        # ax.plot_surface(X, Y, Z)
        return X,Y,Z

    # def update_lines(self, frames, head_lines, path_lines, joint_lines, joint_recon_lines):
    def update_lines(self, frames, head_lines, path_lines, joint_lines):
        dataLines = self.robot.head_axis
        for line, data in zip(head_lines, dataLines):
            line.set_data(data[0:2, :])
            line.set_3d_properties(data[2, :])

        path = self.robot.path
        path_lines.set_data(path[0:2, :])
        path_lines.set_3d_properties(path[2, :])

        joint_pos = self.robot.joint_pos
        joint_lines.set_data(joint_pos[0:2, :])
        joint_lines.set_3d_properties(joint_pos[2, :])

        # joint_pos_recon = self.robot.joint_pos_recon
        # joint_recon_lines.set_data(joint_pos_recon[0:2, :])
        # joint_recon_lines.set_3d_properties(joint_pos_recon[2, :])


        return head_lines, path_lines



if __name__ == '__main__':
    try:
        node = Node()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
