import numpy as np
from timeit import default_timer as timer
from math import sqrt

def norm(x):
    return sqrt(x[0]**2 + x[1]**2 + x[2]**2)

def norm2(x1, x2):
    v = x1-x2
    return sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)
    # return np.sqrt((x1[0]-x2[0])**2 + (x1[1]-x2[1])**2 + (x1[2]-x2[2])**2)

N = 10**5
x1 = np.array([[4], [5], [6]])
x2 = np.array([[1], [2], [3]])


time1 = timer()
for i in range(N):
    a = np.linalg.norm(x1-x2)
d_time = (timer()-time1)*10e3
print("time {:.3f} ms".format(d_time))


time1 = timer()
for i in range(N):
    a = norm(x1-x2)
d_time = (timer()-time1)*10e3
print("time {:.3f} ms".format(d_time))


time1 = timer()
for i in range(N):
    a = norm2(x1, x2)
d_time = (timer()-time1)*10e3
print("time {:.3f} ms".format(d_time))
