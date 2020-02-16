# -*- coding: utf-8 -*-
"""
author: John Bass
email: john.bobzwik@gmail.com
license: MIT
Please feel free to use and modify this, but keep the above information. Thanks!
"""

import numpy as np
from numpy import pi

def find_nearest(array, value):
    array = np.asarray(array)
    idx = (np.abs(array - value)).argmin()
    return array[idx]

#### Multiple Arcs ####

# Grid params
grid_step = 0.25
tun_width = 4
tun_height = 4
yoffset = 0.05

# Initialize pointcloud
pointcloud = np.zeros([1,3])

# Fine gridded coordinate system for start prism and height
xlim1 = -10
xlim2 =  110
xstep = 0.1

ylim1 = yoffset-tun_width/2
ylim2 = yoffset+tun_width/2
ystep = 0.1

zlim1 = -tun_height/2
zlim2 =  tun_height/2
zstep = 0.1

x_space = np.arange(xlim1, xlim2+xstep, xstep)
y_space = np.arange(ylim1, ylim2+ystep, ystep)
z_space = np.arange(zlim1, zlim2+zstep, zstep)

## Start Prism
for i in range(len(x_space)):
    for j in range(len(y_space)):
        # Wall 1
        x_point = x_space[i]
        y_point = y_space[j]
        z_point = zlim1
        # Create point and add it to point cloud
        point = np.array([x_point, y_point, z_point])
        pointcloud = np.vstack((pointcloud, point))

        # Wall 2
        x_point = x_space[i]
        y_point = y_space[j]
        z_point = zlim2
        # Create point and add it to point cloud
        point = np.array([x_point, y_point, z_point])
        pointcloud = np.vstack((pointcloud, point))

    for j in range(len(z_space)):
        # Wall 3
        x_point = x_space[i]
        y_point = ylim1
        z_point = z_space[j]
        # Create point and add it to point cloud
        point = np.array([x_point, y_point, z_point])
        pointcloud = np.vstack((pointcloud, point))

        # Wall 4
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

# Save pointcloud
np.savetxt("./Simulation/environmentGeneration/pointcloud_fine.csv", pointcloud, fmt='%0.4f', delimiter=",")

# Unpack pointcloud
x_obs = pointcloud[:,0]
y_obs = pointcloud[:,1]
z_obs = pointcloud[:,2]

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

# Save pointcloud
np.savetxt("./Simulation/environmentGeneration/pointcloud_grid.csv", pointcloud_filt, fmt='%0.4f', delimiter=",")