# -*- coding: utf-8 -*-
"""
author: John Bass
email: john.bobzwik@gmail.com
license: MIT
Please feel free to use and modify this, but keep the above information. Thanks!
"""

import numpy as np
from numpy import pi
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3

orient = "NED"

#### Prism ####


# Create a gridded coordinate system
xlim1 = -10
xlim2 =  10
xstep = 0.5

ylim1 = -3
ylim2 =  3
ystep = 0.5

zlim1 = -4
zlim2 =  4
zstep = 0.5

x_space = np.arange(xlim1, xlim2+xstep, xstep)
y_space = np.arange(ylim1, ylim2+ystep, ystep)
z_space = np.arange(zlim1, zlim2+zstep, zstep)

# Initialize pointcloud
pointcloud = np.zeros([1,3])

# Wall 1
for i in range(len(x_space)):
    for j in range(len(y_space)):
        x_point = x_space[i]
        y_point = y_space[j]
        z_point = zlim1

        # Create point and add it to point cloud
        point = np.array([x_point, y_point, z_point])
        pointcloud = np.vstack((pointcloud, point))

# Wall 2
for i in range(len(x_space)):
    for j in range(len(y_space)):
        x_point = x_space[i]
        y_point = y_space[j]
        z_point = zlim2

        # Create point and add it to point cloud
        point = np.array([x_point, y_point, z_point])
        pointcloud = np.vstack((pointcloud, point))

# Wall 3
for i in range(len(x_space)):
    for j in range(len(z_space)):
        x_point = x_space[i]
        y_point = ylim1
        z_point = z_space[j]

        # Create point and add it to point cloud
        point = np.array([x_point, y_point, z_point])
        pointcloud = np.vstack((pointcloud, point))

# Wall 4
for i in range(len(x_space)):
    for j in range(len(z_space)):
        x_point = x_space[i]
        y_point = ylim2
        z_point = z_space[j]

        # Create point and add it to point cloud
        point = np.array([x_point, y_point, z_point])
        pointcloud = np.vstack((pointcloud, point))        

# Remove first point
pointcloud = np.delete(pointcloud, 0, axis=0)

# Remove duplicates
pointcloud = np.unique(pointcloud, axis=0)

# Unpack pointcloud
x_obs = pointcloud[:,0]
y_obs = pointcloud[:,1]
z_obs = pointcloud[:,2]

# This is to have roughly proportional axis
maxRange = 0.5*np.array([x_obs.max()-x_obs.min(), y_obs.max()-y_obs.min(), z_obs.max()-z_obs.min()]).max()
mid_x = 0.5*(x_obs.max()+x_obs.min())
mid_y = 0.5*(y_obs.max()+y_obs.min())
mid_z = 0.5*(z_obs.max()+z_obs.min())

# Create figure
fig = plt.figure()
ax = fig.gca(projection='3d', adjustable='box')
ax.scatter(x_obs, y_obs, z_obs, color='red', alpha=1, marker = 'o', s = 2)
ax.set_xlim3d([mid_x-maxRange, mid_x+maxRange])
ax.set_xlabel('X')
if (orient == "NED"):
    ax.set_ylim3d([mid_y+maxRange, mid_y-maxRange])
elif (orient == "ENU"):
    ax.set_ylim3d([mid_y-maxRange, mid_y+maxRange])
ax.set_ylabel('Y')
ax.set_zlim3d([mid_z-maxRange, mid_z+maxRange])
ax.set_zlabel('Altitude')



#### Cylinder ####

# Create a gridded CYLINDRICAL coordinate system

radius = 6

theta_step = pi/16
theta_space = np.arange(0, 2*pi, theta_step)

pointcloud = np.zeros([1,3])

# Cylinder wall
for i in range(len(x_space)):
    for j in range(len(theta_space)):
        x_point = x_space[i]
        y_point = radius*np.cos(theta_space[j])
        z_point = radius*np.sin(theta_space[j])

        # Create point and add it to point cloud
        point = np.array([x_point, y_point, z_point])
        pointcloud = np.vstack((pointcloud, point))

# Remove first point
pointcloud = np.delete(pointcloud, 0, axis=0)

# Remove duplicates
pointcloud = np.unique(pointcloud, axis=0)

# Unpack pointcloud
x_obs = pointcloud[:,0]
y_obs = pointcloud[:,1]
z_obs = pointcloud[:,2]

# This is to have roughly proportional axis
maxRange = 0.5*np.array([x_obs.max()-x_obs.min(), y_obs.max()-y_obs.min(), z_obs.max()-z_obs.min()]).max()
mid_x = 0.5*(x_obs.max()+x_obs.min())
mid_y = 0.5*(y_obs.max()+y_obs.min())
mid_z = 0.5*(z_obs.max()+z_obs.min())

# Create figure
fig = plt.figure()
ax = fig.gca(projection='3d', adjustable='box')
ax.scatter(x_obs, y_obs, z_obs, color='red', alpha=1, marker = 'o', s = 2)
ax.set_xlim3d([mid_x-maxRange, mid_x+maxRange])
ax.set_xlabel('X')
if (orient == "NED"):
    ax.set_ylim3d([mid_y+maxRange, mid_y-maxRange])
elif (orient == "ENU"):
    ax.set_ylim3d([mid_y-maxRange, mid_y+maxRange])
ax.set_ylabel('Y')
ax.set_zlim3d([mid_z-maxRange, mid_z+maxRange])
ax.set_zlabel('Altitude')

plt.show()