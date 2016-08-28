import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt

time = []
real_pos_x = []
real_pos_y = []
real_pos_z = []
real_yaw = []
desired_pos_x = []
desired_pos_y = []
desired_pos_z = []
desired_yaw = []

with open("dronepose.log") as f:
    for line in f:
        p = [float(i) for i in line.split()]
        time.append(p[0])
        real_pos_x.append(p[1])
        real_pos_y.append(p[2])
        real_pos_z.append(p[3])
        real_yaw.append(p[4])
        desired_pos_x.append(p[5])
        desired_pos_y.append(p[6])
        desired_pos_z.append(p[7])
        desired_yaw.append(p[8])

min_time = min(time)
time = [x - min_time for x in time]


plt.figure(1)

plt.subplot(231)
plt.gca().set_color_cycle(['blue', 'red'])
plt.plot(time, real_pos_x)
plt.plot(time, desired_pos_x)
plt.ylabel('x')
plt.xlabel('time in seconds')

plt.subplot(232)
plt.gca().set_color_cycle(['blue', 'red'])
plt.plot(time, real_pos_y)
plt.plot(time, desired_pos_y)
plt.ylabel('y')
plt.xlabel('time in seconds')

plt.subplot(233)
plt.gca().set_color_cycle(['blue', 'red'])
plt.plot(time, real_pos_z)
plt.plot(time, desired_pos_z)
plt.ylabel('z')
plt.xlabel('time in seconds')

plt.subplot(234)
plt.gca().set_color_cycle(['blue', 'red'])
plt.plot(time, real_yaw)
plt.plot(time, desired_yaw)
plt.ylabel('yaw')
plt.xlabel('time in seconds')

plt.subplot(235)
plt.gca().set_color_cycle(['blue', 'red'])
plt.plot(real_pos_x, real_pos_y)
plt.plot(desired_pos_x, desired_pos_y)
plt.xlabel('x')
plt.ylabel('y')

# plt.show()

mpl.rcParams['legend.fontsize'] = 10

fig = plt.figure(2)
ax = fig.gca(projection='3d')
plt.gca().set_color_cycle(['blue', 'red'])
ax.plot(real_pos_x, real_pos_y, real_pos_z, label='real trajectory')
ax.plot(desired_pos_x, desired_pos_y, desired_pos_z, label='desired trajectory')
ax.legend()
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

plt.show()
