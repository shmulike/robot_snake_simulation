import numpy as np
import scipy as sc

class Robot:
    def __init__(self, link_N = 12, link_L = 160, thetaYstep=5, thetaZstep=5, forwardStep=10):
        self.link_N = link_N
        self.link_L = link_L
        self.thetaYstep = thetaYstep
        self.thetaZstep = thetaZstep
        self.forwardStep = forwardStep

        self.v0 = np.array([[0], [0], [0], [1]])
        self.A_head = np.eye(4)
        self.headXAxis = []
        self.headYAxis = []
        self.headZAxis = []
        self.head_axis = []
        self.update_head_axis()


        # create initial path
        x = np.array([np.linspace(-self.link_N*self.link_L, 0, 100)])
        y = z = np.zeros((1, x.shape[1]))
        self.path = np.vstack((x, y, z))



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

        if (forward>0):
            new_path_point = np.dot(self.A_head, self.v0)[0:3, :]
            self.path = np.hstack((self.path, new_path_point))

        self.update_head_axis()


    def update_head_axis(self):
        head_X_size = 200
        head_YZ_size = 100
        origin = np.dot(self.A_head, np.array([[0], [0], [0], [1]]))
        x_p = np.dot(self.A_head, np.array([[head_X_size], [0], [0], [1]]))
        y_p = np.dot(self.A_head, np.array([[0], [head_YZ_size], [0], [1]]))
        z_p = np.dot(self.A_head, np.array([[0], [0], [head_YZ_size], [1]]))

        self.headXAxis = np.hstack((origin, x_p))
        self.headYAxis = np.hstack((origin, y_p))
        self.headZAxis = np.hstack((origin, z_p))

        self.head_axis = np.stack((self.headXAxis, self.headYAxis, self.headZAxis))[0:3, :]



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


