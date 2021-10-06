'''
Shmulik Edelman
shmulike@post.bgu.ac.il
'''
import numpy as np
from math import pi, sqrt
import scipy.interpolate as sc  # interp1d, CubicSpline, splprep, splev
from datetime import datetime
from timeit import default_timer as timer

# add angle limit to head move
# publish joint positions(12+1) and angles (12*2+1)

deadzone_ = 0.12

class Robot:
    def __init__(self, link_N = 10, link_L = 160, thetaYstep=.01, thetaZstep=.01, forwardStep=2, backStep=3, rider_max=1000):
        self.link_N = link_N
        self.link_L = link_L
        self.path_nodes_in_link = 10
        self.thetaYstep = thetaYstep
        self.thetaZstep = thetaZstep
        self.forwardStep = forwardStep
        self.backSteps = backStep
        self.epsilon = 0.01
        self.iter_max = 30
        self.rider_max = rider_max

        self.v0 = np.array([[0], [0], [0], [1]])
        self.v_end = np.array([[self.link_L], [0], [0], [1]])
        self.A_head_1 = self.RzRyRd()
        self.A_head_2 = self.RzRyRd(d=self.link_L)
        # self.A_head_2 = np.dot(self.A_head_1, self.RzRyRd(d=self.link_L))
        self.head_axis_1 = []
        self.head_axis_2 = []
        self.thetaZ_record = 0

        self.vec_len = 1000
        self.vx = np.array([[self.vec_len], [0], [0], [1]])
        self.vy = np.array([[0], [self.vec_len], [0], [1]])
        self.vz = np.array([[0], [0], [self.vec_len], [1]])

        # create initial path
        self.x_start = -(self.link_N-1)*self.link_L
        x = np.array([np.linspace(self.x_start, 0, self.path_nodes_in_link*self.link_N)])
        y = z = np.zeros((1, x.shape[1]))
        self.path = np.vstack((x, y, z))
        self.path_back = np.vstack((x, y, z))
        self.path_head = []
        # self.path = np.fliplr(self.path)

        self.joint_cmd = []
        self.joint_ang = np.zeros((1, self.link_N))
        self.joint_pos_recon = []
        self.update_head_axis()

        # Update joint poses
        # self.joint_pos = [np.arange(self.x_start, self.link_L+1, self.link_L),
        #                   np.zeros((1, self.link_N+1)),
        #                   np.zeros((1, self.link_N+1))]
        self.joint_pos = self.split_curve_4(self.path, self.link_N, self.link_L, self.A_head_2)
        # self.joint_ang, self.joint_cmd = self.calc_joint_angles(self.joint_pos)
        #self.recontract_joints_pos()

    # def turnHead(self, thetaY=0, thetaZ=0):
    #     thetaY *= self.thetaYstep
    #     thetaZ *= self.thetaZstep
    #     # print("move\t{:.2f}:{:.2f}".format(thetaY, thetaZ))
    #     self.A_head = np.dot(self.A_head, self.RyRzRd(thetaY, thetaZ, 0))
    #     self.update_head_axis()




    def move_head(self, thetaY=0, thetaZ=0, forward=0):
        # Scale the joysticl movement by scale factor: thetaYstep, thetaZstep
        thetaY *= self.thetaYstep
        self.thetaZ_record += thetaZ * self.thetaZstep
        # just turn the head

        if forward >= 0 and self.joint_pos[0][0] <= 0:
            forward *= self.forwardStep
            # self.A_head_1 = np.dot(self.A_head_1, self.RzRyRd(y=thetaY))
            self.A_head_1 = self.A_head_1 @ self.RzRyRd(y=thetaY)
            # self.A_head_2 = np.dot(self.A_head_1, np.dot(self.RzRyRd(d=self.link_L), self.RzRyRd(z=self.thetaZ_record, d=forward)))
            self.A_head_2 = self.A_head_1 @ self.RzRyRd(d=self.link_L) @ self.RzRyRd(z=self.thetaZ_record, d=forward)

            head_origin_2_temp = self.A_head_1 @ self.RzRyRd(d=self.link_L) @ self.RzRyRd(z=self.thetaZ_record) @ self.v0
            head_origin_1, head_origin_2 = self.update_head_axis()
            if forward > 0:
                # Head path only
                path_head = np.vstack((np.append(np.linspace(head_origin_1[0, 0], head_origin_2_temp[0, 0], self.path_nodes_in_link), np.linspace(head_origin_2_temp[0, 0], head_origin_2[0, 0], 2)[1:]),
                                       np.append(np.linspace(head_origin_1[1, 0], head_origin_2_temp[1, 0], self.path_nodes_in_link), np.linspace(head_origin_2_temp[1, 0], head_origin_2[1, 0], 2)[1:]),
                                       np.append(np.linspace(head_origin_1[2, 0], head_origin_2_temp[2, 0], self.path_nodes_in_link), np.linspace(head_origin_2_temp[2, 0], head_origin_2[2, 0], 2)[1:])))
                self.joint_pos = self.split_curve_4(np.hstack((self.path, path_head[:, 1:])), self.link_N, self.link_L, self.A_head_2)
                self.path = np.hstack((self.path, self.joint_pos[:, -3].reshape(3, 1)))

        elif forward < 0:
            # forward < 0 == Going backwards
            # How many "steps" (points along the path) go back
            backwards = int(np.floor(forward * self.backSteps))
            self.path = self.path[:, :backwards]
            # self.A_head = np.eye(4)
            # print(self.path.shape)
            # self.A_head[0:3, -1] = self.path[:, -1]
            # head_origin = self.update_head_axis()
            self.findHeadNewPos(self.joint_pos)
            # self.firstOrderDerivative(self.path, self.joint_pos)
            # print(head_origin)
            # update head position and orientation
            # self.A_head = self.path[:, -1].reshape((4,1 ))

        self.joint_ang, self.joint_cmd = self.calc_joint_angles(self.joint_pos)
        #print("head1:\n", np.round(self.A_head_1, 2), "\nhead2:\n", np.round(self.A_head_2, 2), "\n")
        # Update head position relative to world-system

        # Continuous msg publish


        # if forward > 0:
        #     self.joint_ang, self.joint_cmd = self.calc_joint_angles(self.joint_pos)
        # else:
        #     temp = 0
        #     # print(self.head_axis)
        #     # head_pos
        #     # self.calc_joint_angles(np.hstack((self.joint_pos, np.dot())))
        # # self.recontract_joints_pos()

    def update_head_axis(self):
        # updated to single revolve joints
        head_X_size = 400
        head_YZ_size = 200

        # this is the origin of head 1 = Y rotation
        origin_1 = np.dot(self.A_head_1, np.array([[0], [0], [0], [1]]))
        x_p_1 = np.dot(self.A_head_1, np.array([[head_X_size], [0], [0], [1]]))
        y_p_1 = np.dot(self.A_head_1, np.array([[0], [head_YZ_size], [0], [1]]))
        z_p_1 = np.dot(self.A_head_1, np.array([[0], [0], [head_YZ_size], [1]]))
        headXAxis_1 = np.hstack((origin_1, x_p_1))  # Vector of head X axis coordinate system
        headYAxis_1 = np.hstack((origin_1, y_p_1))  # Vector of head Y axis coordinate system
        headZAxis_1 = np.hstack((origin_1, z_p_1))  # Vector of head Z axis coordinate system
        self.head_axis_1 = np.stack((headXAxis_1, headYAxis_1, headZAxis_1))[0:3, :]

        # this is the origin of head 2 = Z rotation
        origin_2 = np.dot(self.A_head_2, np.array([[0], [0], [0], [1]]))
        x_p_2 = np.dot(self.A_head_2, np.array([[head_X_size], [0], [0], [1]]))
        y_p_2 = np.dot(self.A_head_2, np.array([[0], [head_YZ_size], [0], [1]]))
        z_p_2 = np.dot(self.A_head_2, np.array([[0], [0], [head_YZ_size], [1]]))
        headXAxis_2 = np.hstack((origin_2, x_p_2))  # Vector of head X axis coordinate system
        headYAxis_2 = np.hstack((origin_2, y_p_2))  # Vector of head Y axis coordinate system
        headZAxis_2 = np.hstack((origin_2, z_p_2))  # Vector of head Z axis coordinate system
        self.head_axis_2 = np.stack((headXAxis_2, headYAxis_2, headZAxis_2))[0:3, :]
        #print("Head 1 origin:\n", origin_1, "\nHead 2 origin:\n", origin_2)
        return origin_1[0:3, :], origin_2[0:3, :]

    def print_head(self):
        p_head = np.dot(self.A_head, self.v0)
        # print("Robot head position: {}".format(p_head))

    def RzRyRd(self, z=0, y=0, d=0):
        sz = np.sin(z)
        cz = np.cos(z)
        sy = np.sin(y)
        cy = np.cos(y)
        R = np.array([[cy * cz, -sz, sy * cz,  d * cy * cz],
                      [cy * sz,  cz, sy * sz,  d * cy * sz],
                      [-sy,      0,  cy,      -d * sy],
                      [0,        0,  0,        1]])
        return R


    def split_curve_4(self, path, segment_N, segment_L, head_mat):
        # Secant method
        v_end = np.array([[segment_L], [0], [0], [1]])
        path = np.fliplr(path)                  # reverse the path matrix to be "End to Start"

        joints_pos = path[:, 0].reshape(3, 1)
        tck, u = sc.splprep(path, k=2, s=0)

        x0 = x2 = x1_pre = 0
        x1 = 1
        error_avg = 0
        prev_pos = joints_pos
        iter_count_avrg = 0
        for i in range(self.link_N-1):

            iter_count = 0
            error = self.epsilon + 1
            while np.abs(error) > self.epsilon and iter_count < self.iter_max:
                iter_count += 1
                f_x0 = self.link_L - self.norm2(sc.splev(x0, tck), prev_pos)
                f_x1 = self.link_L - self.norm2(sc.splev(x1, tck), prev_pos)
                x2 = x1 - f_x1 * (x1 - x0) / (f_x1 - f_x0)
                x0, x1 = x1, x2

                temp_pos = sc.splev(x2, tck)
                error = self.link_L - self.norm2(temp_pos, prev_pos)
            x0 = x2
            x1 = x2 + 3*(x2-x1_pre)
            x1_pre = x2

            temp_pos = np.asarray([temp_pos]).T
            joints_pos = np.hstack((joints_pos, temp_pos))
            prev_pos = temp_pos
            error_avg += np.abs(error)

            iter_count_avrg += iter_count
            # print("{}, ".format(iter_count), end='')        # Debug: Print number iterations for each joint position
        iter_count_avrg /= segment_N-1
        error_avg /= segment_N - 1
        # print(datetime.now(), " ", end='')
        print("avrg: {:.2f}\tAverage error: {:.7f}".format(iter_count_avrg, error_avg))     # Debug: Print average numberof  iterations and average link length error

        # Add the end link position to the plot
        joints_pos = np.fliplr(joints_pos)
        end_effctor_pos = np.dot(head_mat, v_end)[0:3, :]
        joints_pos = np.hstack((joints_pos, end_effctor_pos))
        # link_len = np.diff(self.joint_pos, axis=1)
        # link_len = np.linalg.norm(link_len, axis=0)
        # print("Debug: split_curve: link mean length: {:.4f}\tAverage iterations: {:.1f}.".format(link_len.mean(), iter_count/self.link_N))

        # print( (timer()-time1)*10e3)
        return joints_pos

    def calc_joint_angles(self, joint_pos):
        # self.joint_cmd = self.joint_pos[0, 0] ##- self.x_start
        joint_ang = []                                     # Create empty vector of angles
        # R = self.RzRyRd(y=0, z=0, d=self.joint_pos[0, 0])       # Generate the first transformation matrix to be placed at the first-back joint, we know its position: joint_pos[0,0]
        R = self.RzRyRd(d=joint_pos[0, 0])       # Generate the first transformation matrix to be placed at the first-back joint, we know its position: joint_pos[0,0]

        # Run on every link:
        for i in range(self.link_N-1):
            # origin = np.dot(R, self.v0)[0:3]
            origin = R[0:3, -1].reshape(3, 1)                           # take the coordinate xyz of this joints
            # x_hat = (np.dot(R, self.vx)[0:3]-origin) / self.vec_len
            x_hat = (np.dot(R, self.vx)[0:3]-origin) / 1000             # Create direction vector X for this joints coordinate system
            y_hat = (np.dot(R, self.vy)[0:3]-origin) / 1000
            z_hat = (np.dot(R, self.vz)[0:3]-origin) / 1000

            next_joint_pose = joint_pos[:, i+2].reshape((3, 1))    # Get the next joints position XYZ only
            new_vec = next_joint_pose - origin                          # Vector for the link

            x_val = np.vdot(new_vec, x_hat)                             # X_het projection on the link vector
            y_val = np.vdot(new_vec, y_hat)
            z_val = np.vdot(new_vec, z_hat)

            if i % 2 == 0:
                # Odd joints 0,2,4,.. = Rotation axis is Y
                theta = -np.arctan2(z_val, x_val)
                theta = self.wrap2pi(theta)
                R = np.dot(R, self.RzRyRd(y=theta, d=self.link_L))  # Using forward kinematics and the calculated angles, go to the next joint
            else:
                # Even joints 1,3,5,.. = Rotation axis is Z
                theta = np.arctan2(y_val, x_val)
                theta = self.wrap2pi(theta)
                R = np.dot(R, self.RzRyRd(z=theta, d=self.link_L))  # Using forward kinematics and the calculated angles, go to the next joint

            if i == self.link_N-2:
                self.A_head_1 = np.dot(R, self.RzRyRd(d=-self.link_L))

            joint_ang = np.append(joint_ang, theta)     # Update the joint_ang vector with the calculated angles

        joint_cmd = np.hstack((joint_pos[0, 0], joint_ang, self.thetaZ_record))    # Generate vector of command values which include linear command followed bt the angle command
        return joint_ang, joint_cmd

    def recontract_joints_pos(self):
        R = self.RzRyRd(z=0, y=0, d=self.joint_cmd[0])
        recon_joints = R[:, -1].reshape(4, 1)

        for i in range(1, self.link_N):
            # Reminder: joint_cmd_ind_Y = i*2-1
            # Reminder: joint_cmd_ind_Z = i*2
            R = np.dot(R, self.RzRyRd(z=self.joint_cmd[i*2-1], y=self.joint_cmd[i*2], d=self.link_L))
            new_joint_pos = R[:, -1].reshape(4, 1)
            recon_joints = np.hstack((recon_joints, new_joint_pos))

        # calculate the difference between the planed joint pose
        # recon_joints = recon_joints[0:3, :]
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

    def findHeadNewPos(self, jointPos):
        joint_ang, joint_cmd = self.calc_joint_angles(jointPos)
        newHead_R = self.RzRyRd(z=0, y=0, d=joint_cmd[0])
        for i in range(1, self.link_N):
            # print("i:{}\t x:{:.2f}".format(i, newHead_R[0, -1]))
            newHead_R = np.dot(newHead_R, self.RzRyRd(z=joint_cmd[i * 2 - 1], y=joint_cmd[i * 2], d=self.link_L))
        self.A_head = newHead_R
        origin = self.update_head_axis()

    def wrap2pi(self, theta):
        # Return (theta + np.pi) % (2 * np.pi) - np.pi
        if theta > 2*np.pi:
            return theta - 2*np.pi
        elif theta < -2*np.pi:
            return theta + 2*np.pi
        else:
            return theta
