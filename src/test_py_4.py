import numpy as np
import scipy.interpolate as sc
import matplotlib.pyplot as plt


fig = plt.figure()
ax = plt.axes(projection='3d')
plt.xlabel('x')
plt.ylabel('y')
N = 10
x_points = np.linspace(0, 2*np.pi, N)
y_points = np.sin(x_points*2)
z_points = np.cos(x_points)

path = np.vstack([x_points, y_points, z_points])
tck, u = sc.splprep(path, k=3, s=0, t=2)
u_fine = np.linspace(0, 1, 100)
spline = sc.splev(u_fine, tck)


plt.plot(x_points, y_points, z_points, '--x')
plt.plot(spline[0], spline[1], spline[2])

t, c, k = tck
cc = np.asarray(c)    # cc.shape is (3, 10) now
spl0 = sc.PPoly.from_spline((t, cc.T[0], k))
spl1 = sc.PPoly.from_spline((t, cc.T[1], k))
# print(spl0.c)   # here are your coefficients for the component 0



x = spl0.x
coef = spl0.c
i = 0
xp = np.linspace(0, 0.2, 100)
S_y = sum(coef[m, i] * (xp - x[i])**(k-m) for m in range(k+1))
S_z = np.cos(xp)
plt.plot(xp, S_y, S_z)


coef = spl0.c
i = 1
xp = np.linspace(0.2, 0.4, 100)
S_y = sum(coef[m, i] * (xp - x[i])**(k-m) for m in range(k+1))
S_z = np.cos(xp)
plt.plot(xp, S_y, S_z)

# i = 0
# for i in range(spl0.c.shape[1]):
#     xi = i*np.pi/spl0.c.shape[1]
#     xii = (i+1)*np.pi/spl0.c.shape[1]
#     xp = np.linspace(xi, xii, 100)
#     S_y = sum(coef[m, i] * (xp - x[i])**(k-m) for m in range(k+1))
#     # S_z = sum(coef[m, i] * (xp - x[i])**(k-m) for m in range(k+1))
#     S_z = np.cos(xp)
#     # plt.plot(xp, S_y, S_z)

print(spl0.c.shape)
plt.show()
