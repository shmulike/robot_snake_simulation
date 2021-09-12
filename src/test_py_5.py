import numpy as np
import scipy.interpolate as sc
import matplotlib.pyplot as plt


fig = plt.figure()
ax = plt.axes(projection='3d')
plt.xlabel('x')
plt.ylabel('y')
N = 10
x_range = np.pi*2
x_points = np.linspace(0, x_range, N)
y_points = np.sin(2*x_points)
z_points = np.cos(x_points)
plt.plot(x_points, y_points, z_points, '--x')

#y
tcky = sc.splrep(x_points, y_points, k=3, s=0)
uy_fine = np.linspace(0, 1, 100)
u_fine = np.linspace(0, 2*np.pi, 100)
spliney = sc.splev(u_fine, tcky)
t, c, k = tcky
# cc = np.asarray(c)    # cc.shape is (3, 10) now
# sply0 = sc.PPoly.from_spline((t, cc.T[0], k))
ppy0 = sc.PPoly.from_spline((t, c, k))

#z
tckz = sc.splrep(x_points, z_points, k=3, s=0)
uz_fine = np.linspace(0, 1, 100)
splinez = sc.splev(u_fine, tckz)
t, c, k = tckz
# cc = np.asarray(c)    # cc.shape is (3, 10) now
# splz0 = sc.PPoly.from_spline((t, cc.T[0], k))
ppz0 = sc.PPoly.from_spline((t, c, k))

# plot 3D spline
plt.plot(u_fine, spliney, splinez)



xy = ppy0.x
xz = ppz0.x
for i in range(N):
    xi = x_range/N*i
    xii = x_range/N*(i+1)
    xp = np.linspace(xi, xii, 100)
    S_y = sum(ppy0.c[m, i] * (xp - xy[i])**(k-m) for m in range(k+1))
    S_z = sum(ppz0.c[m, i] * (xp - xz[i])**(k-m) for m in range(k+1))
    plt.plot(xp, S_y, S_z, '+')

plt.show()