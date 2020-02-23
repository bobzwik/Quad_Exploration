# -*- coding: utf-8 -*-
"""
author: John Bass
email: john.bobzwik@gmail.com
license: MIT
Please feel free to use and modify this, but keep the above information. Thanks!
"""

import numpy as np
from numpy import pi
import vispy
from vispy import app, scene
import time
import sys

import utils
from utils.vispyMods import ColorMarkers, NonUpdatingTurntable
import config

rad2deg = 180.0/pi
deg2rad = pi/180.0

class MyScene(vispy.scene.SceneCanvas):
    def __init__(self, pointcloud, **kwargs):
        super(MyScene, self).__init__(**kwargs)
        self.unfreeze()
        self.pointcloud = pointcloud
        self.startTime  = None
        self.idx_prev = 0
        self.yellowPoints = []
        self.redPoints = []

        if (config.orient == "NED"):
            self.pointcloud[:,2] = -self.pointcloud[:,2]

        self.timer = app.Timer()
        self.freeze()


def third_PV_animation(t_all, waypoints, pos_all, quat_all, euler_all, sDes_tr_all, Ts, params, xyzType, yawType, potfld, notInRange_all, inRange_all, inField_all, ifsave):
    
    x = pos_all[:,0]
    y = pos_all[:,1]
    z = pos_all[:,2]

    xDes = sDes_tr_all[:,0]
    yDes = sDes_tr_all[:,1]
    zDes = sDes_tr_all[:,2]

    x_wp = waypoints[:,0]
    y_wp = waypoints[:,1]
    z_wp = waypoints[:,2]

    psi_ini = euler_all[0,2]*rad2deg

    # Add Canvas
    canvas = MyScene(pointcloud=potfld.pointcloud, keys='interactive', show=True)
    canvas.measure_fps()
    
    # Add Camera
    view = canvas.central_widget.add_view()
    view.camera = NonUpdatingTurntable(distance=3.5, elevation=8, azimuth=(-90-psi_ini), fov=90)
    
    # Flip Z coordinates if NED
    if (config.orient == "NED"):
        z    = -z
        zDes = -zDes
        z_wp = -z_wp
        flip = view.camera.flip
        view.camera.flip = flip[0], not flip[1], flip[2] # Flip camera in Y axis

    # Get initial points
    canvas.yellowPoints = np.where(np.logical_or(notInRange_all[0,:], inRange_all[0,:]))[0]
    canvas.redPoints = np.where(inField_all[0,:])[0]

    # Create color array
    color_points = (1, 1, 0.5, 1)
    color_field  = (1, 0, 0, 1)
    color_edges  = (0, 0, 0, 0.3)
    colors = np.ones((potfld.num_points, 4), dtype=np.float32)

    colors[canvas.yellowPoints] = color_points
    colors[canvas.redPoints] = color_field

    # create scatter object and fill in the data
    scatter = ColorMarkers()
    scatter.set_data(canvas.pointcloud, edge_color=color_edges, face_color=colors, size=6)
    view.add(scatter)

    # Add a colored 3D axis for orientation
    axis = scene.visuals.XYZAxis(parent=view.scene)

    # Lines to draw quadrotor and past trajectory
    line1 = scene.visuals.LinePlot([[],[],[]], width=6, color='red',  marker_size=0, parent=view.scene)
    line2 = scene.visuals.LinePlot([[],[],[]], width=6, color='blue', marker_size=0, parent=view.scene)
    line3 = scene.visuals.LinePlot([[],[],[]], width=2, color='red',  marker_size=0, parent=view.scene)
    
    # Drone params
    dxm = params["dxm"]
    dym = params["dym"]
    dzm = params["dzm"]

    def update(ev):
        if canvas.idx_prev == 0:
            # Start time
            canvas.startTime = time.perf_counter()

        # Get time and find the index of the simulation
        currentTime = time.perf_counter()-canvas.startTime
        idx_now = np.argmax(t_all > currentTime)

        if (idx_now < canvas.idx_prev):
            # Stop animation
            canvas.timer.stop()
            app.quit()
        else:
            # Get drone state for current time
            Simtime = t_all[idx_now]
            pos = pos_all[idx_now]
            x = pos[0]
            y = pos[1]
            z = pos[2]
            x_from0 = pos_all[0:idx_now+1, 0]
            y_from0 = pos_all[0:idx_now+1, 1]
            z_from0 = pos_all[0:idx_now+1, 2]
            quat = quat_all[idx_now]

            # Determine by how much yaw (psi) has changed since last frame
            if (idx_now==0):
                psi_diff = 0
            else:
                psi_diff = (euler_all[idx_now,2] - euler_all[canvas.idx_prev,2])*rad2deg
        
            # Normal NED frame changes
            if (config.orient == "NED"):
                z = -z
                z_from0 = -z_from0
                quat = np.array([quat[0], -quat[1], -quat[2], quat[3]])
                psi_diff = -psi_diff
        
            # Find motor positions
            R = utils.quat2Dcm(quat)    
            motorPoints = np.array([[dxm, -dym, dzm], [0, 0, 0], [dxm, dym, dzm], [-dxm, dym, dzm], [0, 0, 0], [-dxm, -dym, dzm]])
            motorPoints = np.dot(R, np.transpose(motorPoints))
            motorPoints[0,:] = motorPoints[0,:] + x 
            motorPoints[1,:] = motorPoints[1,:] + y 
            motorPoints[2,:] = motorPoints[2,:] + z 
            
            # Draw quadrotor and past trajectory
            line1.set_data(np.array([motorPoints[0, 0:3], motorPoints[1, 0:3], motorPoints[2, 0:3]]).T, marker_size=0,)
            line2.set_data(np.array([motorPoints[0, 3:6], motorPoints[1, 3:6], motorPoints[2, 3:6]]).T, marker_size=0,)
            line3.set_data(np.array([x_from0, y_from0, z_from0]).T, marker_size=0)

            # Change pointcloud colors
            yellowPoints = np.where(np.logical_or(notInRange_all[idx_now,:], inRange_all[idx_now,:]))[0]
            redPoints = np.where(inField_all[idx_now,:])[0]
            if np.setdiff1d(redPoints, canvas.redPoints).size != 0:
                canvas.yellowPoints = yellowPoints
                canvas.redPoints = redPoints
                colors[canvas.yellowPoints] = color_points
                colors[canvas.redPoints] = color_field
                scatter.set_color(face_color=colors)

            # Change camera angle and position
            view.camera.azimuth = view.camera.azimuth + psi_diff
            view.camera.center = [x, y, z]
            
            # Update view
            view.camera.view_changed()
            
            # Set previous index
            canvas.idx_prev = idx_now
    
    canvas.timer.connect(update)
    canvas.timer.start()

    if sys.flags.interactive != 1:
        vispy.app.run()
