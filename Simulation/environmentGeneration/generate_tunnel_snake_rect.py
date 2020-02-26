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
tun_width = 3
tun_height = 3

# Initialize pointcloud
pointcloud = np.zeros([1,3])

# Fine gridded coordinate system for start prism and height
xlim1 = -10
xlim2 =  0
xstep = 0.1

ylim1 = -tun_width/2
ylim2 =  tun_width/2
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


## Arc 1
# Create a fine CYLINDRICAL coordinate system
arc1_radius = 15
arc1_center_x = 0
arc1_center_y = -15

phi_lim1 = 0
phi_lim2 = pi*0.17444286 # Found using CAD software
phi_step = pi/240
phi_space = np.arange(phi_lim1, phi_lim2+phi_step, phi_step) ## POSITIVE increments for 1st arc

radius_step = 0.08
radius_space = np.arange(arc1_radius-tun_width/2, arc1_radius+tun_width/2, radius_step)

# Arc 1
for i in range(len(phi_space)):
    for j in range(len(z_space)):
        # wall 1
        x_point = arc1_center_x + (arc1_radius+tun_width/2)*np.sin(phi_space[i])
        y_point = arc1_center_y + (arc1_radius+tun_width/2)*np.cos(phi_space[i])
        z_point = z_space[j]
        # Create point and add it to point cloud
        point = np.array([x_point, y_point, z_point])
        pointcloud = np.vstack((pointcloud, point))

        # wall 2
        x_point = arc1_center_x + (arc1_radius-tun_width/2)*np.sin(phi_space[i])
        y_point = arc1_center_y + (arc1_radius-tun_width/2)*np.cos(phi_space[i])
        z_point = z_space[j]
        # Create point and add it to point cloud
        point = np.array([x_point, y_point, z_point])
        pointcloud = np.vstack((pointcloud, point))
    
    for j in range(len(radius_space)):
        # floor
        x_point = arc1_center_x + radius_space[j]*np.sin(phi_space[i])
        y_point = arc1_center_y + radius_space[j]*np.cos(phi_space[i])
        z_point = zlim1
        # Create point and add it to point cloud
        point = np.array([x_point, y_point, z_point])
        pointcloud = np.vstack((pointcloud, point))

        # Ceiling
        x_point = arc1_center_x + radius_space[j]*np.sin(phi_space[i])
        y_point = arc1_center_y + radius_space[j]*np.cos(phi_space[i])
        z_point = zlim2
        # Create point and add it to point cloud
        point = np.array([x_point, y_point, z_point])
        pointcloud = np.vstack((pointcloud, point))

## Arc 2
arc2_radius = 15            ## You can change radius of following arcs
radius_space = np.arange(arc2_radius-tun_width/2, arc2_radius+tun_width/2, radius_step)

arc1_end_x = arc1_center_x + arc1_radius*np.sin(phi_lim2)
arc1_end_y = arc1_center_y + arc1_radius*np.cos(phi_lim2)

arc2_center_x = arc1_end_x + arc2_radius*(arc1_end_x-arc1_center_x)/arc1_radius
arc2_center_y = arc1_end_y + arc2_radius*(arc1_end_y-arc1_center_y)/arc1_radius

phi_lim1 = phi_space[-1]+pi          ## last angle of previous arc PLUS pi
phi_lim2 = 3*pi/4
phi_space = np.arange(phi_lim1, phi_lim2-phi_step, -phi_step)  ## NEGATIVE increments for 2nd arc

# Arc 2
for i in range(len(phi_space)):
    for j in range(len(z_space)):
        # wall 1
        x_point = arc2_center_x + (arc2_radius+tun_width/2)*np.sin(phi_space[i])
        y_point = arc2_center_y + (arc2_radius+tun_width/2)*np.cos(phi_space[i])
        z_point = z_space[j]
        # Create point and add it to point cloud
        point = np.array([x_point, y_point, z_point])
        pointcloud = np.vstack((pointcloud, point))

        # wall 2
        x_point = arc2_center_x + (arc2_radius-tun_width/2)*np.sin(phi_space[i])
        y_point = arc2_center_y + (arc2_radius-tun_width/2)*np.cos(phi_space[i])
        z_point = z_space[j]
        # Create point and add it to point cloud
        point = np.array([x_point, y_point, z_point])
        pointcloud = np.vstack((pointcloud, point))
    
    for j in range(len(radius_space)):
        # floor
        x_point = arc2_center_x + radius_space[j]*np.sin(phi_space[i])
        y_point = arc2_center_y + radius_space[j]*np.cos(phi_space[i])
        z_point = zlim1
        # Create point and add it to point cloud
        point = np.array([x_point, y_point, z_point])
        pointcloud = np.vstack((pointcloud, point))

        # Ceiling
        x_point = arc2_center_x + radius_space[j]*np.sin(phi_space[i])
        y_point = arc2_center_y + radius_space[j]*np.cos(phi_space[i])
        z_point = zlim2
        # Create point and add it to point cloud
        point = np.array([x_point, y_point, z_point])
        pointcloud = np.vstack((pointcloud, point))

## Arc 3
arc3_radius = 15            ## You can change radius of following arcs
radius_space = np.arange(arc3_radius-tun_width/2, arc3_radius+tun_width/2, radius_step)

arc2_end_x = arc2_center_x + arc2_radius*np.sin(phi_lim2)
arc2_end_y = arc2_center_y + arc2_radius*np.cos(phi_lim2)

arc3_center_x = arc2_end_x + arc3_radius*(arc2_end_x-arc2_center_x)/arc2_radius
arc3_center_y = arc2_end_y + arc3_radius*(arc2_end_y-arc2_center_y)/arc2_radius

phi_lim1 = phi_space[-1]-pi   ## last angle of previous arc MINUS pi
phi_lim2 = pi/4
phi_space = np.arange(phi_lim1, phi_lim2+phi_step, phi_step)  ## POSITIVE increments for 3rd arc

# Arc 3
for i in range(len(phi_space)):
    for j in range(len(z_space)):
        # wall 1
        x_point = arc3_center_x + (arc3_radius+tun_width/2)*np.sin(phi_space[i])
        y_point = arc3_center_y + (arc3_radius+tun_width/2)*np.cos(phi_space[i])
        z_point = z_space[j]
        # Create point and add it to point cloud
        point = np.array([x_point, y_point, z_point])
        pointcloud = np.vstack((pointcloud, point))

        # wall 2
        x_point = arc3_center_x + (arc3_radius-tun_width/2)*np.sin(phi_space[i])
        y_point = arc3_center_y + (arc3_radius-tun_width/2)*np.cos(phi_space[i])
        z_point = z_space[j]
        # Create point and add it to point cloud
        point = np.array([x_point, y_point, z_point])
        pointcloud = np.vstack((pointcloud, point))
    
    for j in range(len(radius_space)):
        # floor
        x_point = arc3_center_x + radius_space[j]*np.sin(phi_space[i])
        y_point = arc3_center_y + radius_space[j]*np.cos(phi_space[i])
        z_point = zlim1
        # Create point and add it to point cloud
        point = np.array([x_point, y_point, z_point])
        pointcloud = np.vstack((pointcloud, point))

        # Ceiling
        x_point = arc3_center_x + radius_space[j]*np.sin(phi_space[i])
        y_point = arc3_center_y + radius_space[j]*np.cos(phi_space[i])
        z_point = zlim2
        # Create point and add it to point cloud
        point = np.array([x_point, y_point, z_point])
        pointcloud = np.vstack((pointcloud, point))

## Arc 4
arc4_radius = 15            ## You can change radius of following arcs
radius_space = np.arange(arc4_radius-tun_width/2, arc4_radius+tun_width/2, radius_step)

arc3_end_x = arc3_center_x + arc3_radius*np.sin(phi_lim2)
arc3_end_y = arc3_center_y + arc3_radius*np.cos(phi_lim2)

arc4_center_x = arc3_end_x + arc4_radius*(arc3_end_x-arc3_center_x)/arc3_radius
arc4_center_y = arc3_end_y + arc4_radius*(arc3_end_y-arc3_center_y)/arc3_radius

phi_lim1 = phi_space[-1]+pi          ## last angle of previous arc PLUS pi
phi_lim2 = 3*pi/4
phi_space = np.arange(phi_lim1, phi_lim2-phi_step, -phi_step)  ## NEGATIVE increments for 4th arc

# Arc 4
for i in range(len(phi_space)):
    for j in range(len(z_space)):
        # wall 1
        x_point = arc4_center_x + (arc4_radius+tun_width/2)*np.sin(phi_space[i])
        y_point = arc4_center_y + (arc4_radius+tun_width/2)*np.cos(phi_space[i])
        z_point = z_space[j]
        # Create point and add it to point cloud
        point = np.array([x_point, y_point, z_point])
        pointcloud = np.vstack((pointcloud, point))

        # wall 2
        x_point = arc4_center_x + (arc4_radius-tun_width/2)*np.sin(phi_space[i])
        y_point = arc4_center_y + (arc4_radius-tun_width/2)*np.cos(phi_space[i])
        z_point = z_space[j]
        # Create point and add it to point cloud
        point = np.array([x_point, y_point, z_point])
        pointcloud = np.vstack((pointcloud, point))
    
    for j in range(len(radius_space)):
        # floor
        x_point = arc4_center_x + radius_space[j]*np.sin(phi_space[i])
        y_point = arc4_center_y + radius_space[j]*np.cos(phi_space[i])
        z_point = zlim1
        # Create point and add it to point cloud
        point = np.array([x_point, y_point, z_point])
        pointcloud = np.vstack((pointcloud, point))

        # Ceiling
        x_point = arc4_center_x + radius_space[j]*np.sin(phi_space[i])
        y_point = arc4_center_y + radius_space[j]*np.cos(phi_space[i])
        z_point = zlim2
        # Create point and add it to point cloud
        point = np.array([x_point, y_point, z_point])
        pointcloud = np.vstack((pointcloud, point))

## Arc 5
arc5_radius = 15            ## You can change radius of following arcs
radius_space = np.arange(arc5_radius-tun_width/2, arc5_radius+tun_width/2, radius_step)

arc4_end_x = arc4_center_x + arc4_radius*np.sin(phi_lim2)
arc4_end_y = arc4_center_y + arc4_radius*np.cos(phi_lim2)

arc5_center_x = arc4_end_x + arc5_radius*(arc4_end_x-arc4_center_x)/arc4_radius
arc5_center_y = arc4_end_y + arc5_radius*(arc4_end_y-arc4_center_y)/arc4_radius

phi_lim1 = phi_space[-1]-pi   ## last angle of previous arc MINUS pi
phi_lim2 = pi*0.17444286
phi_space = np.arange(phi_lim1, phi_lim2+phi_step, phi_step)  ## POSITIVE increments for 5th arc

# Arc 5
for i in range(len(phi_space)):
    for j in range(len(z_space)):
        # wall 1
        x_point = arc5_center_x + (arc5_radius+tun_width/2)*np.sin(phi_space[i])
        y_point = arc5_center_y + (arc5_radius+tun_width/2)*np.cos(phi_space[i])
        z_point = z_space[j]
        # Create point and add it to point cloud
        point = np.array([x_point, y_point, z_point])
        pointcloud = np.vstack((pointcloud, point))

        # wall 2
        x_point = arc5_center_x + (arc5_radius-tun_width/2)*np.sin(phi_space[i])
        y_point = arc5_center_y + (arc5_radius-tun_width/2)*np.cos(phi_space[i])
        z_point = z_space[j]
        # Create point and add it to point cloud
        point = np.array([x_point, y_point, z_point])
        pointcloud = np.vstack((pointcloud, point))
    
    for j in range(len(radius_space)):
        # floor
        x_point = arc5_center_x + radius_space[j]*np.sin(phi_space[i])
        y_point = arc5_center_y + radius_space[j]*np.cos(phi_space[i])
        z_point = zlim1
        # Create point and add it to point cloud
        point = np.array([x_point, y_point, z_point])
        pointcloud = np.vstack((pointcloud, point))

        # Ceiling
        x_point = arc5_center_x + radius_space[j]*np.sin(phi_space[i])
        y_point = arc5_center_y + radius_space[j]*np.cos(phi_space[i])
        z_point = zlim2
        # Create point and add it to point cloud
        point = np.array([x_point, y_point, z_point])
        pointcloud = np.vstack((pointcloud, point))

## Arc 6
arc6_radius = 15            ## You can change radius of following arcs
radius_space = np.arange(arc6_radius-tun_width/2, arc6_radius+tun_width/2, radius_step)

arc5_end_x = arc5_center_x + arc5_radius*np.sin(phi_lim2)
arc5_end_y = arc5_center_y + arc5_radius*np.cos(phi_lim2)

arc6_center_x = arc5_end_x + arc6_radius*(arc5_end_x-arc5_center_x)/arc5_radius
arc6_center_y = arc5_end_y + arc6_radius*(arc5_end_y-arc5_center_y)/arc5_radius

phi_lim1 = phi_space[-1]+pi          ## last angle of previous arc PLUS pi
phi_lim2 = pi
phi_space = np.arange(phi_lim1, phi_lim2-phi_step, -phi_step)  ## NEGATIVE increments for 6th arc

# Arc 6
for i in range(len(phi_space)):
    for j in range(len(z_space)):
        # wall 1
        x_point = arc6_center_x + (arc6_radius+tun_width/2)*np.sin(phi_space[i])
        y_point = arc6_center_y + (arc6_radius+tun_width/2)*np.cos(phi_space[i])
        z_point = z_space[j]
        # Create point and add it to point cloud
        point = np.array([x_point, y_point, z_point])
        pointcloud = np.vstack((pointcloud, point))

        # wall 2
        x_point = arc6_center_x + (arc6_radius-tun_width/2)*np.sin(phi_space[i])
        y_point = arc6_center_y + (arc6_radius-tun_width/2)*np.cos(phi_space[i])
        z_point = z_space[j]
        # Create point and add it to point cloud
        point = np.array([x_point, y_point, z_point])
        pointcloud = np.vstack((pointcloud, point))
    
    for j in range(len(radius_space)):
        # floor
        x_point = arc6_center_x + radius_space[j]*np.sin(phi_space[i])
        y_point = arc6_center_y + radius_space[j]*np.cos(phi_space[i])
        z_point = zlim1
        # Create point and add it to point cloud
        point = np.array([x_point, y_point, z_point])
        pointcloud = np.vstack((pointcloud, point))

        # Ceiling
        x_point = arc6_center_x + radius_space[j]*np.sin(phi_space[i])
        y_point = arc6_center_y + radius_space[j]*np.cos(phi_space[i])
        z_point = zlim2
        # Create point and add it to point cloud
        point = np.array([x_point, y_point, z_point])
        pointcloud = np.vstack((pointcloud, point))

arc6_end_x = arc6_center_x + arc6_radius*np.sin(phi_lim2)
arc6_end_y = arc6_center_y + arc6_radius*np.cos(phi_lim2)
print((arc6_end_x, arc6_end_y))

# Fine gridded coordinate system for end prism and height
xlim1 = arc6_end_x
xlim2 =  arc6_end_x+15
xstep = 0.1

ylim1 = arc6_end_y-tun_width/2
ylim2 = arc6_end_y+tun_width/2
ystep = 0.1

zlim1 = zlim1
zlim2 = zlim2
zstep = 0.1

x_space = np.arange(xlim1, xlim2+xstep, xstep)
y_space = np.arange(ylim1, ylim2+ystep, ystep)
z_space = np.arange(zlim1, zlim2+zstep, zstep)

## End Prism
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

# Put grid step size as first value of the pointcloud
pointcloud_filt = np.insert(pointcloud_filt, 0, [grid_step, grid_step, grid_step], axis=0)

# Save pointcloud
np.savetxt("./Simulation/environmentGeneration/pointcloud_grid.csv", pointcloud_filt, fmt='%0.4f', delimiter=",")