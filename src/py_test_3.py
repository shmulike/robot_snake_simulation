import numpy as np
import scipy.interpolate as sc
from timeit import default_timer as timer
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from math import sqrt, log
import math

n = 10000
x = np.linspace(0, 20*np.pi, n)
y = np.sin(x/4)*10
z = np.cos(x/4)*10
path = np.vstack((x, y, z))
path = np.fliplr(path)
link_N = 12
link_L = 8
epsilon = 0.005
iter_max = 100
v_end = np.array([[link_L], [0], [0], [1]])


def split_curve1(path):
    joint_pos = path[:, 0].reshape(3, 1)
    tck, u = sc.splprep(path, k=2, s=0)

    # b = 1
    a = 0
    # error_avg = 0
    prev_pos = joint_pos
    sum_iter_count = 0
    for i in range(link_N):
        iter_count = 0
        b = 1
        error = epsilon + 1
        while np.abs(error) > epsilon and iter_count < iter_max:
            iter_count += 1
            c = (a + b) / 2
            # temp_pos = np.asarray([sc.splev(c, tck)]).T
            temp_pos = sc.splev(c, tck)

            # error = self.link_L - self.norm(temp_pos - prev_pos)
            error = link_L - norm2(temp_pos, prev_pos)
            if error > 0:
                a = c
            else:
                b = c
        a = c

        temp_pos = np.asarray([temp_pos]).T
        joint_pos = np.hstack((joint_pos, temp_pos))
        prev_pos = temp_pos
        sum_iter_count += iter_count
        # print(iter_count)
        # error_avg += np.abs(error)
    # print("Average error: {:.7f}".format(error_avg/(self.link_N-1)))

    # Add the end link position to the plot
    joint_pos = np.fliplr(joint_pos)
    # end_effctor_pos = np.dot(A_head, v_end)[0:3, :]
    # joint_pos = np.hstack((joint_pos, end_effctor_pos))
    # link_len = np.diff(joint_pos, axis=1)
    # link_len = np.linalg.norm(link_len, axis=0)
    # print("Debug: split_curve: link average length: {:.4f}\tAverage iterations: {:.1f}.".format(link_len.mean(), sum_iter_count/link_N))

    # print( (timer()-time1)*10e3)
    return joint_pos

def split_curve2(path):
    joint_pos = path[:, 0].reshape(3, 1)
    tck, u = sc.splprep(path, k=2, s=0)

    # b = 1
    a = 0
    # error_avg = 0
    prev_pos = joint_pos
    sum_iter_count = 0
    for i in range(link_N):
        iter_count = 0
        b = 1
        error = np.array([epsilon + 1, epsilon + 2])
        ind = np.array([0, 1])
        while error[ind.min()] > epsilon and iter_count < iter_max:
            iter_count += 1

            c = np.linspace(a, b, 8000)
            temp_pos = sc.splev(c, tck)

            error = np.abs(link_L - np.linalg.norm(temp_pos - prev_pos, axis=0))
            ind = np.argpartition(error, 2)[:2]
            a = c[ind.min()]
            b = c[ind.max()]
            # ind = np.argmin(error)
            # best_error = error[ind]
            # a = c[ind]
            # b = c[ind+1]

        temp_pos = np.array(temp_pos)[:, ind.min()].reshape((3, 1))
        joint_pos = np.hstack((joint_pos, temp_pos))
        prev_pos = temp_pos
        sum_iter_count += iter_count
        # print(iter_count)
        # error_avg += np.abs(error)
    # print("Average error: {:.7f}".format(error_avg/(self.link_N-1)))

    # Add the end link position to the plot
    joint_pos = np.fliplr(joint_pos)
    # end_effctor_pos = np.dot(A_head, v_end)[0:3, :]
    # joint_pos = np.hstack((joint_pos, end_effctor_pos))
    # link_len = np.diff(joint_pos, axis=1)
    # link_len = np.linalg.norm(link_len, axis=0)
    # print("Debug: split_curve: link average length: {:.4f}\tAverage iterations: {:.1f}.".format(link_len.mean(), sum_iter_count/link_N))

    # print( (timer()-time1)*10e3)
    return joint_pos

def norm2(x1, x2):
    return sqrt((x1[0]-x2[0]) ** 2 + (x1[1]-x2[1]) ** 2 + (x1[2]-x2[2]) ** 2)

time1 = timer()
for i in range(100):
    joint_pos1 = split_curve1(path)
d_time = (timer()-time1)*1000
print("time {:.3f} ms".format(d_time))

time1 = timer()
for i in range(100):
    joint_pos2 = split_curve2(path)
d_time = (timer()-time1)*1000
print("time {:.3f} ms".format(d_time))

fig = plt.figure()
ax = plt.axes(projection='3d')
ax.plot3D(x, y, z)

ax.plot3D(joint_pos1[0, :], joint_pos1[1, :], joint_pos1[2, :], 'xr')
ax.plot3D(joint_pos2[0, :], joint_pos2[1, :], joint_pos2[2, :], 'dg')

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")




plt.show(block=True)

