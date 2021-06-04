import numpy as np
from timeit import default_timer as timer
from math import sqrt

def vdot(x1, x2):
    return x1[0]*x2[0] + x1[1]*x2[1] + x1[2]*x2[2]


N = 10**5
x1 = np.array([[4], [5], [6]])
x2 = np.array([[1], [2], [3]])
v0 = np.array([[0], [0], [0], [1]])
R = np.array([[1, 2, 3,  4],
              [5,  6, 7,  8],
              [9,  10,  11, 12],
              [0,  0,  0,   1]])

vec_len = 1000
vx = np.array([[vec_len], [0], [0], [1]])
vy = np.array([[0], [vec_len], [0], [1]])
vz = np.array([[0], [0], [vec_len], [1]])
v = np.hstack([vx, vy, vz])
#################
time1 = timer()
for i in range(N):
    origin = R[0:3, -1].reshape(3, 1)
    x_hat = (np.dot(R, vx)[0:3] - origin) / vec_len
    y_hat = (np.dot(R, vy)[0:3] - origin) / vec_len
    z_hat = (np.dot(R, vz)[0:3] - origin) / vec_len
d_time = (timer()-time1)*10e3
res1 = np.hstack([x_hat, y_hat, z_hat])
print(res1)
print("time {:.3f} ms".format(d_time))


time1 = timer()
for i in range(N):
    origin = R[0:3, -1]
    a = np.dot(R, v)

    x_hat = (a[0:3, 0] - origin).reshape(3, 1) / vec_len
    y_hat = (a[0:3, 1] - origin).reshape(3, 1) / vec_len
    z_hat = (a[0:3, 2] - origin).reshape(3, 1) / vec_len
d_time = (timer()-time1)*10e3
res2 = np.hstack([x_hat, y_hat, z_hat])
print(res2)
print("time {:.3f} ms".format(d_time))
