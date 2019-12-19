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

orient = "ENU"

def find_nearest(array, value):
    array = np.asarray(array)
    idx = (np.abs(array - value)).argmin()
    return array[idx]



#### Grid params####

grid_step = 0.3



#### Prism ####

# Create a gridded coordinate system
xlim1 = -10
xlim2 =  10
xstep = 0.1

ylim1 = -3
ylim2 =  3
ystep = 0.1

zlim1 = -4
zlim2 =  4
zstep = 0.1

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

# This is to have roughly proportional axis when plotting figure
maxRange = 0.5*np.array([x_obs.max()-x_obs.min(), y_obs.max()-y_obs.min(), z_obs.max()-z_obs.min()]).max()
mid_x = 0.5*(x_obs.max()+x_obs.min())
mid_y = 0.5*(y_obs.max()+y_obs.min())
mid_z = 0.5*(z_obs.max()+z_obs.min())

# # Create figure UNFILTERED
# fig = plt.figure()
# ax = fig.gca(projection='3d', adjustable='box')
# ax.scatter(x_obs, y_obs, z_obs, color='red', alpha=1, marker = 'o', s = 2)
# ax.set_xlim3d([mid_x-maxRange, mid_x+maxRange])
# ax.set_xlabel('X')
# if (orient == "NED"):
#     ax.set_ylim3d([mid_y+maxRange, mid_y-maxRange])
# elif (orient == "ENU"):
#     ax.set_ylim3d([mid_y-maxRange, mid_y+maxRange])
# ax.set_ylabel('Y')
# ax.set_zlim3d([mid_z-maxRange, mid_z+maxRange])
# ax.set_zlabel('Altitude')

# Create filtered grid
x_grid = np.arange(0, x_obs.min()-grid_step, -grid_step)[::-1]
x_grid = np.append(x_grid, np.arange(0, x_obs.max()+grid_step, grid_step))
x_grid = np.unique(x_grid)

y_grid = np.arange(0, y_obs.min()-grid_step, -grid_step)[::-1]
y_grid = np.append(y_grid, np.arange(0, y_obs.max()+grid_step, grid_step))
y_grid = np.unique(y_grid)

z_grid = np.arange(0, z_obs.min()-grid_step, -grid_step)[::-1]
z_grid = np.append(z_grid, np.arange(0, z_obs.max()+grid_step, grid_step))
z_grid = np.unique(z_grid)

# Create filtered pointcloud
x_obs_filt = np.zeros(len(x_obs))
y_obs_filt = np.zeros(len(x_obs))
z_obs_filt = np.zeros(len(x_obs))
for i in range(len(x_obs)):
    x_obs_filt[i] = find_nearest(x_grid, x_obs[i])
    y_obs_filt[i] = find_nearest(y_grid, y_obs[i])
    z_obs_filt[i] = find_nearest(z_grid, z_obs[i])

pointcloud_filt = np.transpose(np.array([(x_obs_filt), (y_obs_filt), (z_obs_filt)]))

# Remove duplicates
pointcloud_filt = np.unique(pointcloud_filt, axis=0)

# Unpack pointcloud
x_obs = pointcloud_filt[:,0]
y_obs = pointcloud_filt[:,1]
z_obs = pointcloud_filt[:,2]

# Create figure FILTERED
fig = plt.figure()
ax = fig.gca(projection='3d', adjustable='box')
ax.scatter(x_obs, y_obs, z_obs, color='blue', alpha=1, marker = 'o', s = 2)
ax.set_xlim3d([mid_x-maxRange, mid_x+maxRange])
ax.set_xlabel('X')
if (orient == "NED"):
    ax.set_ylim3d([mid_y+maxRange, mid_y-maxRange])
elif (orient == "ENU"):
    ax.set_ylim3d([mid_y-maxRange, mid_y+maxRange])
ax.set_ylabel('Y')
ax.set_zlim3d([mid_z-maxRange, mid_z+maxRange])
ax.set_zlabel('Altitude')

##################################################################################################

#### Cylinder ####

# Create a gridded CYLINDRICAL coordinate system
tun_radius = 4

theta_step = pi/60
theta_space = np.arange(0, 2*pi, theta_step)

# Initialize pointcloud
pointcloud = np.zeros([1,3])

# Cylinder wall
for i in range(len(x_space)):
    for j in range(len(theta_space)):
        x_point = x_space[i]
        y_point = tun_radius*np.cos(theta_space[j])
        z_point = tun_radius*np.sin(theta_space[j])

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

# This is to have roughly proportional axis when plotting figure
maxRange = 0.5*np.array([x_obs.max()-x_obs.min(), y_obs.max()-y_obs.min(), z_obs.max()-z_obs.min()]).max()
mid_x = 0.5*(x_obs.max()+x_obs.min())
mid_y = 0.5*(y_obs.max()+y_obs.min())
mid_z = 0.5*(z_obs.max()+z_obs.min())

# # Create figure UNFILTERED
# fig = plt.figure()
# ax = fig.gca(projection='3d', adjustable='box')
# ax.scatter(x_obs, y_obs, z_obs, color='red', alpha=1, marker = 'o', s = 2)
# ax.set_xlim3d([mid_x-maxRange, mid_x+maxRange])
# ax.set_xlabel('X')
# if (orient == "NED"):
#     ax.set_ylim3d([mid_y+maxRange, mid_y-maxRange])
# elif (orient == "ENU"):
#     ax.set_ylim3d([mid_y-maxRange, mid_y+maxRange])
# ax.set_ylabel('Y')
# ax.set_zlim3d([mid_z-maxRange, mid_z+maxRange])
# ax.set_zlabel('Altitude')

# Create filtered grid
x_grid = np.arange(0, x_obs.min()-grid_step, -grid_step)[::-1]
x_grid = np.append(x_grid, np.arange(0, x_obs.max()+grid_step, grid_step))
x_grid = np.unique(x_grid)

y_grid = np.arange(0, y_obs.min()-grid_step, -grid_step)[::-1]
y_grid = np.append(y_grid, np.arange(0, y_obs.max()+grid_step, grid_step))
y_grid = np.unique(y_grid)

z_grid = np.arange(0, z_obs.min()-grid_step, -grid_step)[::-1]
z_grid = np.append(z_grid, np.arange(0, z_obs.max()+grid_step, grid_step))
z_grid = np.unique(z_grid)

# Create filtered pointcloud
x_obs_filt = np.zeros(len(x_obs))
y_obs_filt = np.zeros(len(x_obs))
z_obs_filt = np.zeros(len(x_obs))
for i in range(len(x_obs)):
    x_obs_filt[i] = find_nearest(x_grid, x_obs[i])
    y_obs_filt[i] = find_nearest(y_grid, y_obs[i])
    z_obs_filt[i] = find_nearest(z_grid, z_obs[i])

pointcloud_filt = np.transpose(np.array([(x_obs_filt), (y_obs_filt), (z_obs_filt)]))

# Remove duplicates
pointcloud_filt = np.unique(pointcloud_filt, axis=0)

# Unpack pointcloud
x_obs = pointcloud_filt[:,0]
y_obs = pointcloud_filt[:,1]
z_obs = pointcloud_filt[:,2]

# Create figure FILTERED
fig = plt.figure()
ax = fig.gca(projection='3d', adjustable='box')
ax.scatter(x_obs, y_obs, z_obs, color='blue', alpha=1, marker = 'o', s = 2)
ax.set_xlim3d([mid_x-maxRange, mid_x+maxRange])
ax.set_xlabel('X')
if (orient == "NED"):
    ax.set_ylim3d([mid_y+maxRange, mid_y-maxRange])
elif (orient == "ENU"):
    ax.set_ylim3d([mid_y-maxRange, mid_y+maxRange])
ax.set_ylabel('Y')
ax.set_zlim3d([mid_z-maxRange, mid_z+maxRange])
ax.set_zlabel('Altitude')


##################################################################################################

#### Arc ####

# Create a gridded CYLINDRICAL coordinate system
arc_radius = 15
arc_center_x = 0
arc_center_y = -15
tun_width = 6

phi_step = pi/240
phi_space = np.arange(0, pi/2+phi_step, phi_step)

radius_step = 0.08
radius_space = np.arange(arc_radius-tun_width/2, arc_radius+tun_width/2+radius_step, radius_step)

# Initialize pointcloud
pointcloud = np.zeros([1,3])

# Arc wall 1
for i in range(len(z_space)):
    for j in range(len(phi_space)):
        x_point = arc_center_x + (arc_radius+tun_width/2)*np.sin(phi_space[j])
        y_point = arc_center_y + (arc_radius+tun_width/2)*np.cos(phi_space[j])
        z_point = z_space[i]

        # Create point and add it to point cloud
        point = np.array([x_point, y_point, z_point])
        pointcloud = np.vstack((pointcloud, point))

# Arc wall 2
for i in range(len(z_space)):
    for j in range(len(phi_space)):
        x_point = arc_center_x + (arc_radius-tun_width/2)*np.sin(phi_space[j])
        y_point = arc_center_y + (arc_radius-tun_width/2)*np.cos(phi_space[j])
        z_point = z_space[i]

        # Create point and add it to point cloud
        point = np.array([x_point, y_point, z_point])
        pointcloud = np.vstack((pointcloud, point))

# Arc wall 3
for i in range(len(radius_space)):
    for j in range(len(phi_space)):
        x_point = arc_center_x + radius_space[i]*np.sin(phi_space[j])
        y_point = arc_center_y + radius_space[i]*np.cos(phi_space[j])
        z_point = zlim1

        # Create point and add it to point cloud
        point = np.array([x_point, y_point, z_point])
        pointcloud = np.vstack((pointcloud, point))

# Arc wall 4
for i in range(len(radius_space)):
    for j in range(len(phi_space)):
        x_point = arc_center_x + radius_space[i]*np.sin(phi_space[j])
        y_point = arc_center_y + radius_space[i]*np.cos(phi_space[j])
        z_point = zlim2

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

# This is to have roughly proportional axis when plotting figure
maxRange = 0.5*np.array([x_obs.max()-x_obs.min(), y_obs.max()-y_obs.min(), z_obs.max()-z_obs.min()]).max()
mid_x = 0.5*(x_obs.max()+x_obs.min())
mid_y = 0.5*(y_obs.max()+y_obs.min())
mid_z = 0.5*(z_obs.max()+z_obs.min())

# # Create figure UNFILTERED
# fig = plt.figure()
# ax = fig.gca(projection='3d', adjustable='box')
# ax.scatter(x_obs, y_obs, z_obs, color='red', alpha=1, marker = 'o', s = 2)
# ax.set_xlim3d([mid_x-maxRange, mid_x+maxRange])
# ax.set_xlabel('X')
# if (orient == "NED"):
#     ax.set_ylim3d([mid_y+maxRange, mid_y-maxRange])
# elif (orient == "ENU"):
#     ax.set_ylim3d([mid_y-maxRange, mid_y+maxRange])
# ax.set_ylabel('Y')
# ax.set_zlim3d([mid_z-maxRange, mid_z+maxRange])
# ax.set_zlabel('Altitude')

# Create filtered grid
x_grid = np.arange(0, x_obs.min()-grid_step, -grid_step)[::-1]
x_grid = np.append(x_grid, np.arange(0, x_obs.max()+grid_step, grid_step))
x_grid = np.unique(x_grid)

y_grid = np.arange(0, y_obs.min()-grid_step, -grid_step)[::-1]
y_grid = np.append(y_grid, np.arange(0, y_obs.max()+grid_step, grid_step))
y_grid = np.unique(y_grid)

z_grid = np.arange(0, z_obs.min()-grid_step, -grid_step)[::-1]
z_grid = np.append(z_grid, np.arange(0, z_obs.max()+grid_step, grid_step))
z_grid = np.unique(z_grid)

# Create filtered pointcloud
x_obs_filt = np.zeros(len(x_obs))
y_obs_filt = np.zeros(len(x_obs))
z_obs_filt = np.zeros(len(x_obs))
for i in range(len(x_obs)):
    x_obs_filt[i] = find_nearest(x_grid, x_obs[i])
    y_obs_filt[i] = find_nearest(y_grid, y_obs[i])
    z_obs_filt[i] = find_nearest(z_grid, z_obs[i])

pointcloud_filt = np.transpose(np.array([(x_obs_filt), (y_obs_filt), (z_obs_filt)]))

# Remove duplicates
pointcloud_filt = np.unique(pointcloud_filt, axis=0)

# Unpack pointcloud
x_obs = pointcloud_filt[:,0]
y_obs = pointcloud_filt[:,1]
z_obs = pointcloud_filt[:,2]

# Create figure FILTERED
fig = plt.figure()
ax = fig.gca(projection='3d', adjustable='box')
ax.scatter(x_obs, y_obs, z_obs, color='blue', alpha=1, marker = 'o', s = 2)
ax.set_xlim3d([mid_x-maxRange, mid_x+maxRange])
ax.set_xlabel('X')
if (orient == "NED"):
    ax.set_ylim3d([mid_y+maxRange, mid_y-maxRange])
elif (orient == "ENU"):
    ax.set_ylim3d([mid_y-maxRange, mid_y+maxRange])
ax.set_ylabel('Y')
ax.set_zlim3d([mid_z-maxRange, mid_z+maxRange])
ax.set_zlabel('Altitude')


##########
plt.show()

