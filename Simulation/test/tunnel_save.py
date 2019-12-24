# -*- coding: utf-8 -*-
"""
author: John Bass
email: john.bobzwik@gmail.com
license: MIT
Please feel free to use and modify this, but keep the above information. Thanks!
"""

import numpy as np
from numpy import pi

orient = "ENU"

def find_nearest(array, value):
    array = np.asarray(array)
    idx = (np.abs(array - value)).argmin()
    return array[idx]



#### Grid params####

grid_step = 0.3

# Create a fine gridded coordinate system

xlim1 = -10
xlim2 =  10
xstep = 0.1

ylim1 = -3
ylim2 =  3
ystep = 0.1

zlim1 = -2
zlim2 =  2
zstep = 0.1

x_space = np.arange(xlim1, xlim2+xstep, xstep)
y_space = np.arange(ylim1, ylim2+ystep, ystep)
z_space = np.arange(zlim1, zlim2+zstep, zstep)

##################################################################################################

#### Multiple Arcs ####

# Initialize pointcloud
pointcloud = np.zeros([1,3])

## Arc 1
# Create a fine CYLINDRICAL coordinate system
arc1_radius = 15
arc1_center_x = 0
arc1_center_y = -15
tun_width = 5

phi_lim1 = 0
phi_lim2 = pi*0.17444286
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
phi_space = np.arange(phi_lim1, phi_lim2-phi_step, -phi_step)  ## NEGATIVE increments for 2nd arc

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
phi_space = np.arange(phi_lim1, phi_lim2+phi_step, phi_step)  ## POSITIVE increments for 3rd arc

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
phi_space = np.arange(phi_lim1, phi_lim2-phi_step, -phi_step)  ## NEGATIVE increments for 2nd arc

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

# Remove first point
pointcloud = np.delete(pointcloud, 0, axis=0)

# Remove duplicates
pointcloud = np.unique(pointcloud, axis=0)

# Save pointcloud
np.savetxt("./Simulation/test/pointcloud_fine.csv", pointcloud, fmt='%0.4f', delimiter=",")

# Unpack pointcloud
x_obs = pointcloud[:,0]
y_obs = pointcloud[:,1]
z_obs = pointcloud[:,2]

# This is to have roughly proportional axis when plotting figure
maxRange = 0.5*np.array([x_obs.max()-x_obs.min(), y_obs.max()-y_obs.min(), z_obs.max()-z_obs.min()]).max()
mid_x = 0.5*(x_obs.max()+x_obs.min())
mid_y = 0.5*(y_obs.max()+y_obs.min())
mid_z = 0.5*(z_obs.max()+z_obs.min())

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
np.savetxt("./Simulation/test/pointcloud_grid.csv", pointcloud_filt, fmt='%0.4f', delimiter=",")

# Unpack pointcloud
x_obs = pointcloud_filt[:,0]
y_obs = pointcloud_filt[:,1]
z_obs = pointcloud_filt[:,2]

# ##################################################################################################

# #### Multiple Cylinder Arcs ####

# # Initialize pointcloud
# pointcloud = np.zeros([1,3])

# tun_radius = 4

# theta_step = pi/60
# theta_space = np.arange(0, 2*pi, theta_step)

# ## Arc 1
# # Create a gridded CYLINDRICAL coordinate system
# arc1_radius = 15
# arc1_center_x = 0
# arc1_center_y = -15

# phi_lim1 = 0
# phi_lim2 = pi/4
# phi_step = pi/240
# phi_space = np.arange(phi_lim1, phi_lim2+phi_step, phi_step) ## POSITIVE increments for 1st arc

# radius_step = 0.15

# # Arc 1 cylinder wall
# for i in range(len(theta_space)):
#     for j in range(len(phi_space)):
#         x_point = arc1_center_x + (arc1_radius + tun_radius*np.cos(theta_space[i]))*np.sin(phi_space[j])
#         y_point = arc1_center_y + (arc1_radius + tun_radius*np.cos(theta_space[i]))*np.cos(phi_space[j])
#         z_point = tun_radius*np.sin(theta_space[i])

#         # Create point and add it to point cloud
#         point = np.array([x_point, y_point, z_point])
#         pointcloud = np.vstack((pointcloud, point))

# ## Arc 2 
# arc2_radius = 15            ## You can change radius of following arcs

# arc1_end_x = arc1_center_x + arc1_radius*np.sin(phi_lim2)
# arc1_end_y = arc1_center_y + arc1_radius*np.cos(phi_lim2)

# arc2_center_x = arc1_end_x + arc2_radius*(arc1_end_x-arc1_center_x)/arc1_radius
# arc2_center_y = arc1_end_y + arc2_radius*(arc1_end_y-arc1_center_y)/arc1_radius

# phi_lim1 = phi_lim2+pi          ## last angle of previous arc PLUS pi
# phi_lim2 = 3*pi/4
# phi_space = np.arange(phi_lim1, phi_lim2-phi_step, -phi_step)  ## NEGATIVE increments for 2nd arc

# # Arc 2 cylinder wall
# for i in range(len(theta_space)):
#     for j in range(len(phi_space)):
#         x_point = arc2_center_x + (arc2_radius + tun_radius*np.cos(theta_space[i]))*np.sin(phi_space[j])
#         y_point = arc2_center_y + (arc2_radius + tun_radius*np.cos(theta_space[i]))*np.cos(phi_space[j])
#         z_point = tun_radius*np.sin(theta_space[i])

#         # Create point and add it to point cloud
#         point = np.array([x_point, y_point, z_point])
#         pointcloud = np.vstack((pointcloud, point))

# ## Arc 3
# arc3_radius = 15            ## You can change radius of following arcs

# arc2_end_x = arc2_center_x + arc2_radius*np.sin(phi_lim2)
# arc2_end_y = arc2_center_y + arc2_radius*np.cos(phi_lim2)

# arc3_center_x = arc2_end_x + arc3_radius*(arc2_end_x-arc2_center_x)/arc2_radius
# arc3_center_y = arc2_end_y + arc3_radius*(arc2_end_y-arc2_center_y)/arc2_radius

# phi_lim1 = phi_lim2-pi   ## last angle of previous arc MINUS pi
# phi_lim2 = pi/4
# phi_space = np.arange(phi_lim1, phi_lim2+phi_step, phi_step)  ## POSITIVE increments for 3rd arc

# # Arc 3 cylinder wall
# for i in range(len(theta_space)):
#     for j in range(len(phi_space)):
#         x_point = arc3_center_x + (arc3_radius + tun_radius*np.cos(theta_space[i]))*np.sin(phi_space[j])
#         y_point = arc3_center_y + (arc3_radius + tun_radius*np.cos(theta_space[i]))*np.cos(phi_space[j])
#         z_point = tun_radius*np.sin(theta_space[i])

#         # Create point and add it to point cloud
#         point = np.array([x_point, y_point, z_point])
#         pointcloud = np.vstack((pointcloud, point))

# # Remove first point
# pointcloud = np.delete(pointcloud, 0, axis=0)

# # Remove duplicates
# pointcloud = np.unique(pointcloud, axis=0)

# # Unpack pointcloud
# x_obs = pointcloud[:,0]
# y_obs = pointcloud[:,1]
# z_obs = pointcloud[:,2]

# # This is to have roughly proportional axis when plotting figure
# maxRange = 0.5*np.array([x_obs.max()-x_obs.min(), y_obs.max()-y_obs.min(), z_obs.max()-z_obs.min()]).max()
# mid_x = 0.5*(x_obs.max()+x_obs.min())
# mid_y = 0.5*(y_obs.max()+y_obs.min())
# mid_z = 0.5*(z_obs.max()+z_obs.min())

# # Create figure UNFILTERED
# # fig = plt.figure()
# # ax = fig.gca(projection='3d', adjustable='box')
# # ax.scatter(x_obs, y_obs, z_obs, color='red', alpha=1, marker = 'o', s = 2)
# # ax.set_xlim3d([mid_x-maxRange, mid_x+maxRange])
# # ax.set_xlabel('X')
# # if (orient == "NED"):
# #     ax.set_ylim3d([mid_y+maxRange, mid_y-maxRange])
# # elif (orient == "ENU"):
# #     ax.set_ylim3d([mid_y-maxRange, mid_y+maxRange])
# # ax.set_ylabel('Y')
# # ax.set_zlim3d([mid_z-maxRange, mid_z+maxRange])
# # ax.set_zlabel('Altitude')

# # Create filtered grid
# x_grid = np.arange(0, x_obs.min()-grid_step, -grid_step)[::-1]
# x_grid = np.append(x_grid, np.arange(0, x_obs.max()+grid_step, grid_step))
# x_grid = np.unique(x_grid)

# y_grid = np.arange(0, y_obs.min()-grid_step, -grid_step)[::-1]
# y_grid = np.append(y_grid, np.arange(0, y_obs.max()+grid_step, grid_step))
# y_grid = np.unique(y_grid)

# z_grid = np.arange(0, z_obs.min()-grid_step, -grid_step)[::-1]
# z_grid = np.append(z_grid, np.arange(0, z_obs.max()+grid_step, grid_step))
# z_grid = np.unique(z_grid)

# # Create filtered pointcloud
# x_obs_filt = np.zeros(len(x_obs))
# y_obs_filt = np.zeros(len(x_obs))
# z_obs_filt = np.zeros(len(x_obs))
# for i in range(len(x_obs)):
#     x_obs_filt[i] = find_nearest(x_grid, x_obs[i])
#     y_obs_filt[i] = find_nearest(y_grid, y_obs[i])
#     z_obs_filt[i] = find_nearest(z_grid, z_obs[i])

# pointcloud_filt = np.transpose(np.array([(x_obs_filt), (y_obs_filt), (z_obs_filt)]))

# # Remove duplicates
# pointcloud_filt = np.unique(pointcloud_filt, axis=0)

# # Unpack pointcloud
# x_obs = pointcloud_filt[:,0]
# y_obs = pointcloud_filt[:,1]
# z_obs = pointcloud_filt[:,2]

# # Create figure FILTERED
# fig = plt.figure()
# ax = fig.gca(projection='3d', adjustable='box')
# ax.scatter(x_obs, y_obs, z_obs, color='blue', alpha=1, marker = 'o', s = 2)
# ax.set_xlim3d([mid_x-maxRange, mid_x+maxRange])
# ax.set_xlabel('X')
# if (orient == "NED"):
#     ax.set_ylim3d([mid_y+maxRange, mid_y-maxRange])
# elif (orient == "ENU"):
#     ax.set_ylim3d([mid_y-maxRange, mid_y+maxRange])
# ax.set_ylabel('Y')
# ax.set_zlim3d([mid_z-maxRange, mid_z+maxRange])
# ax.set_zlabel('Altitude')
