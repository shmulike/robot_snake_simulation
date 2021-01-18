'''
Shmulik Edelman
shmulike@post.bgu.ac.il
'''
import numpy as np
from math import pi, sqrt
import scipy.interpolate as sc  # interp1d, CubicSpline, splprep, splev


class Robot:
    def __init__(self, link_N = 12, link_L = 160, thetaYstep=1, thetaZstep=1, forwardStep=2):
        self.link_N = link_N
        self.link_L = link_L
        self.thetaYstep = thetaYstep
        self.thetaZstep = thetaZstep
        self.forwardStep = forwardStep
        self.epsilon = 1

        self.v0 = np.array([[0], [0], [0], [1]])
        self.v_end = np.array([[self.link_L], [0], [0], [1]])
        self.A_head = np.eye(4)
        self.head_axis = []
        self.joint_pos = []
        self.update_head_axis()


        # create initial path
        x = np.array([np.linspace(-self.link_N*self.link_L, 0, 100)])
        y = z = np.zeros((1, x.shape[1]))
        self.path = np.vstack((x, y, z))

        # Update joint poses
        self.split_curve()


    def turnHead(self, thetaY=0, thetaZ=0):
        thetaY *= self.thetaYstep
        thetaZ *= self.thetaZstep
        # print("move\t{:.2f}:{:.2f}".format(thetaY, thetaZ))
        self.A_head = np.dot(self.A_head, self.RyRzRd(thetaY, thetaZ, 0))
        self.update_head_axis()


    def move_head(self, thetaY=0, thetaZ=0, forward=0):
        thetaY *= self.thetaYstep
        thetaZ *= self.thetaZstep

        if (forward > 0):
            forward *= self.forwardStep
        else:
            forward = 0

        self.A_head = np.dot(self.A_head, self.RyRzRd(thetaY, thetaZ, forward))

        head_origin = self.update_head_axis()

        if (forward>0):
            self.path = np.hstack((self.path, head_origin))
            self.split_curve()

    def update_head_axis(self):
        head_X_size = 200
        head_YZ_size = 100
        origin = np.dot(self.A_head, np.array([[0], [0], [0], [1]]))
        x_p = np.dot(self.A_head, np.array([[head_X_size], [0], [0], [1]]))
        y_p = np.dot(self.A_head, np.array([[0], [head_YZ_size], [0], [1]]))
        z_p = np.dot(self.A_head, np.array([[0], [0], [head_YZ_size], [1]]))

        headXAxis = np.hstack((origin, x_p))
        headYAxis = np.hstack((origin, y_p))
        headZAxis = np.hstack((origin, z_p))

        self.head_axis = np.stack((headXAxis, headYAxis, headZAxis))[0:3, :]
        return origin[0:3, :]


    def print_head(self):
        p_head = np.dot(self.A_head, self.v0)
        # print("Robot head position: {}".format(p_head))


    def RyRzRd(self, y, z, d):
        y = np.deg2rad(y)
        z = np.deg2rad(z)
        R = np.array([[np.cos(y)*np.cos(z), -np.cos(y)*np.sin(z), np.sin(y), d*np.cos(y)*np.cos(z)],
                      [np.sin(z), np.cos(z), 0, d*np.sin(z)],
                      [-np.sin(y)*np.cos(z), np.sin(y)*np.sin(z), np.cos(y), -d*np.sin(y)*np.cos(z)],
                      [0, 0, 0, 1]])

        # print(R)
        return R

    def getPath(self):
        # print(type(self.path))
        return self.path

    def getHead(self):
        # print(type(self.head_axis))
        return self.head_axis

    def split_curve(self):
        # Flip path = end -> start
        path = np.fliplr(self.path)
        self.joint_pos = path[:, 0].reshape(3, 1)

        tck, u = sc.splprep(path, k=2, s=0)

        iter_count = 0
        b = 1
        a = 0
        prev_pos = self.joint_pos[:, 0].reshape(3, 1)
        for i in range(self.link_N):
            b = 1
            e = self.epsilon + 1
            while (np.abs(e) > self.epsilon):
                iter_count += 1
                c = (a+b)/2
                temp_pos = np.asarray([sc.splev(c, tck)]).T
                e = self.link_L - np.linalg.norm(temp_pos - prev_pos)
                if (e > 0):
                    a = c
                else:
                    b = c
            u = c
            # new_joint_pos = np.asarray([sc.splev(u, tck)]).T
            self.joint_pos = np.hstack((self.joint_pos, temp_pos))
            prev_pos = temp_pos

        # Add the end link position to the plot
        self.joint_pos = np.fliplr(self.joint_pos)
        end_effctor_pos = np.dot(self.A_head, self.v_end)[0:3, :]
        self.joint_pos = np.hstack((self.joint_pos, end_effctor_pos))
        # link_len = np.diff(self.joint_pos, axis=1)
        # link_len = np.linalg.norm(link_len, axis=0)
        # print("Debug: split_curve: link mean length: {:.4f}\tAverage iterations: {:.1f}.".format(link_len.mean(), iter_count/self.link_N))

    def calc_joint_angles(self):
        A = self.RyRzRd(0, 0, self.joint_pos[0, 0])
        for i in range(1, self.link_N):
            thetaZ = np.arctan2(self.joint_pos[1, i], self.joint_pos[0, i])
            thetaY = np.arctan2(self.joint_pos[2, i], np.linalg.norm(self.joint_pos[0:2, i]))
            print("Angles: {:.2f}\t{:.2f}".format(np.rad2deg(thetaY),np.rad2deg(thetaZ)))


