import numpy as np
from timeit import default_timer as timer


def RzRyRd(z, y, d):
    R = np.array([[np.cos(y) * np.cos(z), -np.sin(z), np.sin(y) * np.cos(z), d * np.cos(y) * np.cos(z)],
                  [np.cos(y) * np.sin(z), np.cos(z), np.sin(y) * np.sin(z), d * np.cos(y) * np.sin(z)],
                  [-np.sin(y), 0, np.cos(y), -d * np.sin(y)],
                  [0, 0, 0, 1]])
    return R

def RzRyRd2(z, y, d):
    sz = np.sin(z)
    cz = np.cos(z)
    sy = np.sin(y)
    cy = np.cos(y)
    # a = cy * cz
    # b = cy * sz
    R = np.array([[cy * cz, -sz, sy * cz, d * cy * cz],
                  [cy * sz, cz, sy * sz, d * cy * sz],
                  [-sy, 0, cy, -d * sy],
                  [0, 0, 0, 1]])
    return R

N = 10**0
time_1 = timer()
for i in range(N):
    RzRyRd(i, i, i)
time_2 = timer()

for i in range(N):
    RzRyRd2(i, i, i)
time_3 = timer()

R1 = RzRyRd(0.2, 0.3, 0.4)
R2 = RzRyRd2(0.2, 0.3, 0.4)
print("R\n", R1)
print("R2\n", R2)

print(R2-R1)

print("RzRyRd: {:.5f}".format((time_2-time_1)))
print("RzRyRd2: {:.5f}".format((time_3-time_2)))


N = 10**8
time_1 = timer()
for i in range(N):
    time_0 = timer()
time_2 = timer()
print("timer func: ", time_2-time_1)