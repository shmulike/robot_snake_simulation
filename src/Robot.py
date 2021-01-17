import numpy as np
import scipy as sc

class Robot:
    def __init__(self, link_N = 12, link_L = 160, thetaYstep=10, thetaZstep=10, Dstep=10):
        self.link_N = link_N
        self.link_L = link_L
        self.thetaYstep = thetaYstep
        self.thetaZstep = thetaZstep
        self.Dstep = Dstep

        self.v0 = np.array([[0], [0], [0], [1]])
        self.A_head = np.eye(4)
        self.headXAxis = []
        self.headYAxis = []
        self.headZAxis = []
        self.update_head_axis()


        # create initial path
        x = np.array([np.linspace(-self.link_N*self.link_L, 0, 20)]).T
        y = z = np.zeros((len(x), 1))
        self.path = np.hstack((x, y, z))




        # fig.canvas.draw()

    def turnHead(self, thetaY=0, thetaZ=0):
        thetaY *= self.thetaYstep
        thetaZ *= self.thetaZstep
        print("move\t{:.2f}:{:.2f}".format(thetaY, thetaZ))
        self.A_head = np.dot(self.A_head, self.RyRzRd(thetaY, thetaZ, 0))
        self.update_head_axis()
        # self.plot_head()


    def move_head(self, D=0):
        print("move")
        self.A_head = np.dot(self.A_head, self.RyRzRd(0, 0, D))
        # self.plot_head()

    def update_head_axis(self):
        head_axis_size = 200
        origin = np.dot(self.A_head, np.array([[0], [0], [0], [1]]))
        x_p = np.dot(self.A_head, np.array([[head_axis_size], [0], [0], [1]]))
        y_p = np.dot(self.A_head, np.array([[0], [head_axis_size], [0], [1]]))
        z_p = np.dot(self.A_head, np.array([[0], [0], [head_axis_size], [1]]))

        self.headXAxis = np.hstack((origin, x_p))
        self.headYAxis = np.hstack((origin, y_p))
        self.headZAxis = np.hstack((origin, z_p))

        # return (x_axis, y_axis, z_axis)
        # plt.clf()
        # self.ax.plot3D(x_axis[0], x_axis[1], x_axis[2], 'r', linewidth=2)
        # self.ax.plot3D(y_axis[0], y_axis[1], y_axis[2], 'g', linewidth=2)
        # self.ax.plot3D(z_axis[0], z_axis[1], z_axis[2], 'b', linewidth=2)

        # self.hl_headYaxes.set_xdata([origin[0], x_axis[0]])
        # self.hl_headYaxes.set_ydata([origin[1], x_axis[1]])
        # self.hl_headYaxes.set_3d_properties([origin[2], x_axis[2]])
        #
        # self.hl_headZaxes.set_xdata([origin[0], x_axis[0]])
        # self.hl_headZaxes.set_zdata([origin[1], x_axis[1]])
        # self.hl_headZaxes.set_3d_properties([origin[2], x_axis[2]])

        # plt.clf()
        # plt.draw()
        # plt.pause(0.01)


    def print_head(self):
        p_head = self.A_head * self.v0
        # print("Robot head position: {}".format(p_head))

    # def plot_path(self, f=1):
    #
    #     # self.ax.autoscale(enable=True, axis='both', tight=True)
    #     # plt.plot(self.path[:, 0], self.path[:, 1], self.path[:, 2], 'b.', markersize=1)
    #     self.ax.plot3D(self.path[:, 0], self.path[:, 1], self.path[:, 2], 'b.', markersize=5)
    #
    #     plt.draw()
    #     # plt.pause(1)


    # def update_line(hl, path):
    #     hl.set_xdata(path[:, 0])
    #     hl.set_ydata(path[:, 0])
    #     hl.set_3d_properties(path[:, 0])
    #     plt.draw()
    #     # plt.show()


    def RyRzRd(self, y, z, d):
        y = np.deg2rad(y)
        z = np.deg2rad(z)
        R = np.array([[np.cos(y)*np.cos(z), -np.cos(y)*np.sin(z), np.sin(y), d*np.cos(y)*np.cos(z)],
                      [np.sin(z), np.cos(z), 0, d*np.sin(z)],
                      [-np.sin(y)*np.cos(z), np.sin(y)*np.sin(z), np.cos(y), -d*np.sin(y)*np.cos(z)],
                      [0, 0, 0, 1]])

        # R = np.array([[np.cos(y), 0, np.sin(y), d*np.cos(y)],
        #               [np.sin(x)*np.sin(y), np.cos(x), -np.cos(y)*np.sin(x), d*np.sin(x)*np.sin(y)],
        #               [-np.cos(x)*np.sin(y), np.sin(x), np.cos(x)*np.cos(y), -d*np.cos(x)*np.sin(y)],
        #               [0, 0, 0, 1]])

        # print(R)
        return R

    def getPath(self):
        return self.path

    def getHead(self):
        # print(type(self.headXAxis))
        return self.headXAxis, self.headYAxis, self.headZAxis


