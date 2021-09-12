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
    def __init__(self, link_N = 12, link_L = 160, thetaYstep=.01, thetaZstep=.01, forwardStep=2, backStep=3, rider_max=1000):
        self.link_N = link_N
        self.link_L = link_L
        self.thetaYstep = thetaYstep
        self.thetaZstep = thetaZstep
        self.forwardStep = forwardStep
        self.backSteps = backStep
        self.epsilon = 0.01
        self.iter_max = 30
        self.rider_max = rider_max

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
        # self.path = np.fliplr(self.path)

        self.joint_cmd = []
        self.joint_pos_recon = []
        self.update_head_axis()

        # Update joint poses
        self.joint_pos = np.arange(self.x_start, self.link_L+1, self.link_L)
        self.split_curve_3()
        self.joint_ang, self.joint_cmd = self.calc_joint_angles(self.joint_pos)
        self.recontract_joints_pos()

    # def turnHead(self, thetaY=0, thetaZ=0):
    #     thetaY *= self.thetaYstep
    #     thetaZ *= self.thetaZstep
    #     # print("move\t{:.2f}:{:.2f}".format(thetaY, thetaZ))
    #     self.A_head = np.dot(self.A_head, self.RyRzRd(thetaY, thetaZ, 0))
    #     self.update_head_axis()


    def move_head(self, thetaY=0, thetaZ=0, forward=0):
        # Scale the joysticl movement by scale factor: thetaYstep, thetaZstep
        thetaY *= self.thetaYstep
        thetaZ *= self.thetaZstep

        # print(self.joint_pos[0][0])
        head_origin = None
        if (forward >= 0 and self.joint_pos[0][0] <= 0):
            forward *= self.forwardStep
            self.A_head = np.dot(self.A_head, self.RzRyRd(z=thetaZ, y=thetaY, d=forward))
            head_origin = self.update_head_axis()
            if (forward > 0):
                self.path = np.hstack((self.path, head_origin))
        else:
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

        print("head:\n", np.round(self.A_head, 2), "\n")
        # Update head position relative to world-system

        # if (forward>0):
        #     # self.path = np.hstack((head_origin, self.path))
        #     self.path = np.hstack((self.path, head_origin))

        # Continuous msg publish
        self.split_curve_3()
        if forward >= 0:
            self.joint_ang, self.joint_cmd = self.calc_joint_angles(self.joint_pos)
        else:
            temp = 0
            # print(self.head_axis)
            # head_pos
            # self.calc_joint_angles(np.hstack((self.joint_pos, np.dot())))
        self.recontract_joints_pos()

    def update_head_axis(self):
        head_X_size = 400
        head_YZ_size = 400
        origin = np.dot(self.A_head, np.array([[0], [0], [0], [1]]))
        x_p = np.dot(self.A_head, np.array([[head_X_size], [0], [0], [1]]))
        y_p = np.dot(self.A_head, np.array([[0], [head_YZ_size], [0], [1]]))
        z_p = np.dot(self.A_head, np.array([[0], [0], [head_YZ_size], [1]]))

        headXAxis = np.hstack((origin, x_p))        # Vector of head X axis coordinate system
        headYAxis = np.hstack((origin, y_p))        # Vector of head Y axis coordinate system
        headZAxis = np.hstack((origin, z_p))        # Vector of head Z axis coordinate system

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

    def split_curve_3(self):
        # Secant method
        # reverse the path matrix to be end o start
        path = np.fliplr(self.path)

        self.joint_pos = path[:, 0].reshape(3, 1)
        tck, u = sc.splprep(path, k=2, s=0)

        x0 = x2 = x1_pre = 0
        x1 = 1
        error_avg = 0
        prev_pos = self.joint_pos
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
            self.joint_pos = np.hstack((self.joint_pos, temp_pos))
            prev_pos = temp_pos
            error_avg += np.abs(error)

            iter_count_avrg += iter_count
            # print("{}, ".format(iter_count), end='')        # Debug: Print number iterations for each joint position
        iter_count_avrg /= self.link_N-1
        error_avg /= self.link_N - 1
        # print(datetime.now(), " ", end='')
        print("avrg: {:.2f}\tAverage error: {:.7f}".format(iter_count_avrg, error_avg))     # Debug: Print average numberof  iterations and average link length error

        # Add the end link position to the plot
        self.joint_pos = np.fliplr(self.joint_pos)
        end_effctor_pos = np.dot(self.A_head, self.v_end)[0:3, :]
        self.joint_pos = np.hstack((self.joint_pos, end_effctor_pos))
        # link_len = np.diff(self.joint_pos, axis=1)
        # link_len = np.linalg.norm(link_len, axis=0)
        # print("Debug: split_curve: link mean length: {:.4f}\tAverage iterations: {:.1f}.".format(link_len.mean(), iter_count/self.link_N))

        # print( (timer()-time1)*10e3)

    def calc_joint_angles(self, jointPos):
        # self.joint_cmd = self.joint_pos[0, 0] ##- self.x_start
        joint_ang = []                                     # Create empty vector of angles
        # R = self.RzRyRd(y=0, z=0, d=self.joint_pos[0, 0])       # Generate the first transformation matrix to be placed at the first-back joint, we know its position: joint_pos[0,0]
        R = self.RzRyRd(y=0, z=0, d=jointPos[0, 0])       # Generate the first transformation matrix to be placed at the first-back joint, we know its position: joint_pos[0,0]
        # vec_len = 1000
        # vx = np.array([[vec_len], [0], [0], [1]])
        # vy = np.array([[0], [vec_len], [0], [1]])
        # vz = np.array([[0], [0], [vec_len], [1]])

        # Run on every link:
        for i in range(self.link_N):
            # origin = np.dot(R, self.v0)[0:3]
            origin = R[0:3, -1].reshape(3, 1)                           # take the coordinate xyz of this joints
            # x_hat = (np.dot(R, self.vx)[0:3]-origin) / self.vec_len
            x_hat = (np.dot(R, self.vx)[0:3]-origin) / 1000             # Create direction vector X for this joints coordinate system
            y_hat = (np.dot(R, self.vy)[0:3]-origin) / 1000
            z_hat = (np.dot(R, self.vz)[0:3]-origin) / 1000

            next_joint_pose = jointPos[:, i+1].reshape((3, 1))    # Get the next joints position XYZ only
            new_vec = next_joint_pose - origin                          # Vector for the link

            x_val = np.vdot(new_vec, x_hat)                             # X_het projection on the link vector
            y_val = np.vdot(new_vec, y_hat)
            z_val = np.vdot(new_vec, z_hat)

            thetaZ = np.arctan2(y_val, x_val)                               # Calculate the joint Z rotation angle
            if abs(thetaZ) == np.pi:
                thetaZ = 0

            thetaY = -np.arctan2(z_val, sqrt(x_val**2 + y_val**2))          # Calculate the joint Y rotation angle
            if abs(thetaY) == np.pi:
                thetaY = 0

            joint_ang = np.append(joint_ang, [thetaZ, thetaY])    # Update the joint_ang vector with the calculated angles

            R = np.dot(R, self.RzRyRd(z=thetaZ, y=thetaY, d=self.link_L))   # Using forward kinematics and the calculated angles, go to the next joint

        joint_cmd = np.append(jointPos[0, 0], joint_ang)    # Generate vector of command values which include linear command followed bt the angle command
        # print("Angles: Y {:.2f}\tZ {:.2f}".format(np.rad2deg(thetaY), np.rad2deg(thetaZ)))
        return joint_ang, joint_cmd

    def calc_head_angles(self):
        print("r")
        # Recalculate head movement only

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

    def firstOrderDerivative(self, path, jointPos):
        # dfx = (3*fx_i - 4*fx_i-1 + fx_i-2) / 2h
        # dx = (3*path[0, -1] - 4*path[0, -2] + path[0, -3]) / abs(path[0, -3] - path[0, -1])
        h = abs(path[0, -3] - path[0, -1])
        x0 = path[0, -1]
        y0 = path[1, -1]
        z0 = path[2, -1]
        my = (3*path[1, -1] - 4*path[1, -2] + path[1, -3]) / (2*h)
        mz = (3*path[2, -1] - 4*path[2, -2] + path[2, -3]) / (2*h)

        ny = y0-my*x0
        nz = z0-mz*x0

        A = 1 + my**2 + mz**2
        B = -2*x0 + 2*my*(ny-y0) + 2*mz*(nz-z0)
        C = x0**2 + (ny-y0)**2 + (nz-z0)**2 - self.link_L**2

        # x1 = (-B + np.sqrt(B**2 - 4*A*C)) / (2*A)
        x1 = (-B - np.sqrt(B**2 - 4*A*C)) / (2*A)
        y1 = my*x1 + ny
        z1 = mz*x1 + nz

        print("x1:{:.2f}\ty1:{:.2f}\tz1:{:.2f}".format(x1, y1, z1))

        # print("ny:{:.2f}\tnz:{:.2f}".format(ny, nz))
        # print("x1:{:.2f}\tx2:{:.2f}".format(x1, x2))
        # print("sqrt:{:.2f}".format(B**2 - 4*A*C))
        # joint_ang, joint_cmd = self.calc_joint_angles(np.hstack((path, [[x1], [y1], [z1]]) ))
        joint_ang, joint_cmd = self.calc_joint_angles(jointPos)

        # R = self.RzRyRd(z=0, y=0, d=self.joint_pos[0, 0])
        R = self.RzRyRd(z=0, y=0, d=joint_cmd[0])
        for i in range(1, self.link_N):
            print("i:{}\t x:{:.2f}".format(i, R[0, -1]))
            R = np.dot(R, self.RzRyRd(z=joint_cmd[i * 2 - 1], y=joint_cmd[i * 2], d=self.link_L))

        self.A_head = R
        origin = self.update_head_axis()

    def findHeadNewPos(self, jointPos):
        joint_ang, joint_cmd = self.calc_joint_angles(jointPos)
        newHead_R = self.RzRyRd(z=0, y=0, d=joint_cmd[0])
        for i in range(1, self.link_N):
            print("i:{}\t x:{:.2f}".format(i, newHead_R[0, -1]))
            newHead_R = np.dot(newHead_R, self.RzRyRd(z=joint_cmd[i * 2 - 1], y=joint_cmd[i * 2], d=self.link_L))
        self.A_head = newHead_R
        origin = self.update_head_axis()



