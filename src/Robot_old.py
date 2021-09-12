'''
Shmulik Edelman
shmulike@post.bgu.ac.il
'''
import numpy as np
from math import pi, sqrt
import scipy.interpolate as sc  # interp1d, CubicSpline, splprep, splev
from timeit import default_timer as timer

# add angle limit to head move
# publish joint positions(12+1) and angles (12*2+1)

class Robot:
    def __init__(self, link_N = 12, link_L = 160, thetaYstep=.01, thetaZstep=.01, forwardStep=2):
        self.link_N = link_N
        self.link_L = link_L
        self.thetaYstep = thetaYstep
        self.thetaZstep = thetaZstep
        self.forwardStep = forwardStep
        self.epsilon = 0.1
        self.iter_max = 25

        self.v0 = np.array([[0], [0], [0], [1]])
        self.v_end = np.array([[self.link_L], [0], [0], [1]])
        self.A_head = np.eye(4)
        self.head_axis = []

        self.vec_len = 1000
        self.vx = np.array([[self.vec_len], [0], [0], [1]])
        self.vy = np.array([[0], [self.vec_len], [0], [1]])
        self.vz = np.array([[0], [0], [self.vec_len], [1]])

        # create initial path
        self.x_start = -(self.link_N-1)*self.link_L
        x = np.array([np.linspace(self.x_start, 0, 100)])
        y = z = np.zeros((1, x.shape[1]))
        self.path = np.vstack((x, y, z))
        self.path = np.fliplr(self.path)

        self.joint_cmd = []
        self.joint_pos_recon = []
        self.update_head_axis()


        # Update joint poses
        self.joint_pos = np.arange(self.x_start, self.link_L+1, self.link_L)
        self.split_curve()
        self.calc_joint_angles()
        # self.recontract_joints_pos()

    # def turnHead(self, thetaY=0, thetaZ=0):
    #     thetaY *= self.thetaYstep
    #     thetaZ *= self.thetaZstep
    #     # print("move\t{:.2f}:{:.2f}".format(thetaY, thetaZ))
    #     self.A_head = np.dot(self.A_head, self.RyRzRd(thetaY, thetaZ, 0))
    #     self.update_head_axis()


    def move_head(self, thetaY=0, thetaZ=0, forward=0):
        thetaY *= self.thetaYstep
        thetaZ *= self.thetaZstep

        if (forward > 0):
            forward *= self.forwardStep
        else:
            forward = 0

        self.A_head = np.dot(self.A_head, self.RzRyRd(z=thetaZ, y=thetaY, d=forward))

        head_origin = self.update_head_axis()

        if (forward>0):
            self.path = np.hstack((head_origin, self.path))
            # time_start_1 = timer()

            # time_start_2 = timer()
            self.calc_joint_angles()
            # print(self.joint_cmd)
            # time_start_3 = timer()
            # self.recontract_joints_pos()
            # time_end = timer()
            # print("Time: curve {:3.3f}\tangles {:3.3f}\trecontract {:3.3f}".format((time_start_2-time_start_1)*10e3, (time_start_3-time_start_2)*10e3, (time_end-time_start_3)*10e3))
            # print(self.joint_pos_recon[0, :])
        self.split_curve()
        self.calc_joint_angles()
        
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


    # def RyRzRd(self, y, z, d):
    #     # y = np.deg2rad(y)
    #     # z = np.deg2rad(z)
    #     R = np.array([[np.cos(y)*np.cos(z), -np.cos(y)*np.sin(z), np.sin(y), d*np.cos(y)*np.cos(z)],
    #                   [np.sin(z), np.cos(z), 0, d*np.sin(z)],
    #                   [-np.sin(y)*np.cos(z), np.sin(y)*np.sin(z), np.cos(y), -d*np.sin(y)*np.cos(z)],
    #                   [0, 0, 0, 1]])
    #     return R

    def RzRyRd(self, z, y, d):
        sz = np.sin(z)
        cz = np.cos(z)
        sy = np.sin(y)
        cy = np.cos(y)
        R = np.array([[cy * cz, -sz, sy * cz,  d * cy * cz],
                      [cy * sz,  cz, sy * sz,  d * cy * sz],
                      [-sy,      0,  cy,      -d * sy],
                      [0,        0,  0,        1]])
        return R


    def split_curve(self):
        self.joint_pos = self.path[:, 0].reshape(3, 1)
        tck, u = sc.splprep(self.path, k=2, s=0)

        # b = 1
        a = 0
        # error_avg = 0
        prev_pos = self.joint_pos
        for i in range(self.link_N-1):
            iter_count = 0
            b = 1
            error = self.epsilon + 1
            while np.abs(error) > self.epsilon and iter_count < self.iter_max:
                iter_count += 1
                c = (a+b)/2
                # temp_pos = np.asarray([sc.splev(c, tck)]).T
                temp_pos = sc.splev(c, tck)

                # error = self.link_L - self.norm(temp_pos - prev_pos)
                error = self.link_L - self.norm2(temp_pos, prev_pos)
                if error > 0:
                    a = c
                else:
                    b = c
            a = c

            temp_pos = np.asarray([temp_pos]).T
            self.joint_pos = np.hstack((self.joint_pos, temp_pos))
            prev_pos = temp_pos
            # error_avg += np.abs(error)
        # print("Average error: {:.7f}".format(error_avg/(self.link_N-1)))

        # Add the end link position to the plot
        self.joint_pos = np.fliplr(self.joint_pos)
        end_effctor_pos = np.dot(self.A_head, self.v_end)[0:3, :]
        self.joint_pos = np.hstack((self.joint_pos, end_effctor_pos))
        # link_len = np.diff(self.joint_pos, axis=1)
        # link_len = np.linalg.norm(link_len, axis=0)
        # print("Debug: split_curve: link mean length: {:.4f}\tAverage iterations: {:.1f}.".format(link_len.mean(), iter_count/self.link_N))

        # print( (timer()-time1)*10e3)

    def split_curve2(self):
        self.joint_pos = self.path[:, 0].reshape(3, 1)
        tck, u = sc.splprep(self.path, k=2, s=0)

        # b = 1
        a = 0
        # error_avg = 0
        prev_pos = self.joint_pos
        for i in range(self.link_N-1):
            iter_count = 0
            b = 1
            error = self.epsilon + 1
            while np.abs(error) > self.epsilon and iter_count < self.iter_max:
                iter_count += 1
                c = (a+b)/2
                # temp_pos = np.asarray([sc.splev(c, tck)]).T
                temp_pos = sc.splev(c, tck)

                # error = self.link_L - self.norm(temp_pos - prev_pos)
                error = self.link_L - self.norm2(temp_pos, prev_pos)
                if error > 0:
                    a = c
                else:
                    b = c
            a = c

            temp_pos = np.asarray([temp_pos]).T
            self.joint_pos = np.hstack((self.joint_pos, temp_pos))
            prev_pos = temp_pos
            # error_avg += np.abs(error)
        # print("Average error: {:.7f}".format(error_avg/(self.link_N-1)))

        # Add the end link position to the plot
        self.joint_pos = np.fliplr(self.joint_pos)
        end_effctor_pos = np.dot(self.A_head, self.v_end)[0:3, :]
        self.joint_pos = np.hstack((self.joint_pos, end_effctor_pos))
        # link_len = np.diff(self.joint_pos, axis=1)
        # link_len = np.linalg.norm(link_len, axis=0)
        # print("Debug: split_curve: link mean length: {:.4f}\tAverage iterations: {:.1f}.".format(link_len.mean(), iter_count/self.link_N))

        # print( (timer()-time1)*10e3)


    def calc_joint_angles(self):
        # self.joint_cmd = self.joint_pos[0, 0] ##- self.x_start
        self.joint_ang = []
        R = self.RzRyRd(y=0, z=0, d=self.joint_pos[0, 0])
        # vec_len = 1000
        # vx = np.array([[vec_len], [0], [0], [1]])
        # vy = np.array([[0], [vec_len], [0], [1]])
        # vz = np.array([[0], [0], [vec_len], [1]])

        for i in range(self.link_N):
            # origin = np.dot(R, self.v0)[0:3]
            origin = R[0:3, -1].reshape(3, 1)
            # x_hat = (np.dot(R, self.vx)[0:3]-origin) / self.vec_len
            x_hat = (np.dot(R, self.vx)[0:3]-origin) / 1000
            y_hat = (np.dot(R, self.vy)[0:3]-origin) / 1000
            z_hat = (np.dot(R, self.vz)[0:3]-origin) / 1000

            next_joint_pose = self.joint_pos[:, i+1].reshape((3, 1))
            new_vec = next_joint_pose - origin


            x_val = np.vdot(new_vec, x_hat)
            y_val = np.vdot(new_vec, y_hat)
            z_val = np.vdot(new_vec, z_hat)

            thetaZ = np.arctan2(y_val, x_val)
            thetaY = -np.arctan2(z_val, sqrt(x_val**2 + y_val**2))

            R = np.dot(R, self.RzRyRd(z=thetaZ, y=thetaY, d=self.link_L))
            self.joint_ang = np.append(self.joint_ang, [thetaZ, thetaY])

        self.joint_cmd = np.append(self.joint_pos[0, 0], self.joint_ang)
            # print("Angles: Y {:.2f}\tZ {:.2f}".format(np.rad2deg(thetaY), np.rad2deg(thetaZ)))


    def recontract_joints_pos(self):
        R = self.RzRyRd(z=0, y=0, d=self.joint_cmd[0])
        # recon_joints = np.dot(R, self.v0)
        recon_joints = R[:, -1].reshape(4, 1)

        for i in range(1, self.link_N):
            # joint_cmd_ind_Y = i*2-1
            # joint_cmd_ind_Z = i*2
            R = np.dot(R, self.RzRyRd(z=self.joint_cmd[i*2-1], y=self.joint_cmd[i*2], d=self.link_L))
            # new_joint_pos = np.dot(R, self.v0)
            new_joint_pos = R[:, -1].reshape(4, 1)
            recon_joints = np.hstack((recon_joints, new_joint_pos))

        # calculate the difference between the planed joint pose
        recon_joints = recon_joints[0:3, :]
        self.joint_pos_recon = recon_joints[0:3, :]

        # debug
        # d = recon_joints - self.joint_pos[:, :-1]
        # d = np.linalg.norm(d, axis=0)
        # print("Min-Average-Max: {:.6f}-{:.6f}-{:.6f}".format(d.min(), d.mean(), d.max()))

    def norm(self, x):
        return sqrt(x[0] ** 2 + x[1] ** 2 + x[2] ** 2)

    def norm2(self, x1, x2):
        return sqrt((x1[0]-x2[0]) ** 2 + (x1[1]-x2[1]) ** 2 + (x1[2]-x2[2]) ** 2)

    def vdot(self, x1, x2):
        res = x1[0]*x2[0] + x1[1]*x2[1] + x1[2]*x2[2]
        return res[0]


