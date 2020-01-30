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
from vispy.scene import visuals

import utils
import config

numFrames = 10
rad2deg = 180.0/pi
deg2rad = pi/180.0

def sameAxisAnimation(t_all, waypoints, pos_all, quat_all, euler_all, sDes_tr_all, Ts, params, xyzType, yawType, potfld, notInRange_all, inRange_all, inField_all, ifsave):
    
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

    Plot3D1 = scene.visuals.create_visual_node(vispy.visuals.LinePlotVisual)
    Plot3D2 = scene.visuals.create_visual_node(vispy.visuals.LinePlotVisual)
    Plot3D3 = scene.visuals.create_visual_node(vispy.visuals.LinePlotVisual)

    pointcloud = potfld.pointcloud

    # Add Canvas
    canvas = vispy.scene.SceneCanvas(keys='interactive', show=True)
    canvas.measure_fps()
    
    # Add Camera
    view = canvas.central_widget.add_view()
    view.camera = vispy.scene.cameras.TurntableCamera(distance=3, elevation=8, azimuth=(-90-psi_ini), fov=80)
    
    # Add Grid
    # grid = scene.visuals.GridLines(color=(1,1,1), parent=view.scene)
    
    # Flip Z coordinates if NED
    if (config.orient == "NED"):
        z    = -z
        zDes = -zDes
        z_wp = -z_wp
        pointcloud[:,2] = -pointcloud[:,2]
        flip = view.camera.flip
        view.camera.flip = flip[0], not flip[1], flip[2] # Flip camera in Y axis

    # create scatter object and fill in the data
    scatter = visuals.Markers()
    scatter.set_data(pointcloud[np.where(notInRange_all[0,:])[0]], edge_color=None, face_color=(1, 1, 0, 0.7), size=6)
    # scatter.set_gl_state('translucent', cull_face=False)
    view.add(scatter)

    scatterInRange = visuals.Markers()
    scatterInRange.set_data(pointcloud[np.where(inRange_all[0,:])[0]], edge_color=None, face_color=(0, 1, 0, 0.7), size=6)
    # scatter.set_gl_state('translucent', cull_face=False)
    view.add(scatterInRange)

    scatterInField = visuals.Markers()
    scatterInField.set_data(pointcloud[np.where(inField_all[0,:])[0]], edge_color=None, face_color=(1, 0, 0, 0.7), size=6)
    # scatter.set_gl_state('translucent', cull_face=False)
    view.add(scatterInField)

    # add a colored 3D axis for orientation
    axis = visuals.XYZAxis(parent=view.scene)

    line1 = Plot3D1([[],[],[]], width=3, color='red', marker_size=0, parent=view.scene)
    line2 = Plot3D2([[],[],[]], width=3, color='blue', marker_size=0, parent=view.scene)
    line3 = Plot3D3([[],[],[]], width=2, color='red', marker_size=0, parent=view.scene)

    i = 1


    def update(ev):
        nonlocal i
        time = t_all[i*numFrames]
        pos = pos_all[i*numFrames]
        x = pos[0]
        y = pos[1]
        z = pos[2]
        x_from0 = pos_all[0:i*numFrames,0]
        y_from0 = pos_all[0:i*numFrames,1]
        z_from0 = pos_all[0:i*numFrames,2]

        psi = euler_all[i*numFrames,2]*rad2deg

        dxm = params["dxm"]
        dym = params["dym"]
        dzm = params["dzm"]
        
        quat = quat_all[i*numFrames]
    
        if (config.orient == "NED"):
            z = -z
            z_from0 = -z_from0
            quat = np.array([quat[0], -quat[1], -quat[2], quat[3]])
    
        R = utils.quat2Dcm(quat)    
        motorPoints = np.array([[dxm, -dym, dzm], [0, 0, 0], [dxm, dym, dzm], [-dxm, dym, dzm], [0, 0, 0], [-dxm, -dym, dzm]])
        motorPoints = np.dot(R, np.transpose(motorPoints))
        motorPoints[0,:] = motorPoints[0,:] + x 
        motorPoints[1,:] = motorPoints[1,:] + y 
        motorPoints[2,:] = motorPoints[2,:] + z 
        line1.set_data(np.array([motorPoints[0,0:3], motorPoints[1,0:3], motorPoints[2,0:3]]).T, marker_size=0,)
        line2.set_data(np.array([motorPoints[0,3:6], motorPoints[1,3:6], motorPoints[2,3:6]]).T, marker_size=0,)
        line3.set_data(np.array([x_from0, y_from0, z_from0]).T, marker_size=0)

        view.camera.azimuth = -90-psi
        view.camera.center = [x,y,z]
        scatter.set_data(pointcloud[np.where(notInRange_all[i*numFrames,:])[0]], edge_color=None, face_color=(1, 1, 0, 0.7), size=6)
        scatterInRange.set_data(pointcloud[np.where(inRange_all[i*numFrames,:])[0]], edge_color=None, face_color=(0, 1, 0, 0.7), size=6)
        scatterInField.set_data(pointcloud[np.where(inField_all[i*numFrames,:])[0]], edge_color=None, face_color=(1, 0, 0, 0.7), size=6)

        i += 1

    timer = app.Timer(Ts*numFrames)
    timer.connect(update)
    timer.start(iterations=len(x)/numFrames-1)

    import sys
    if sys.flags.interactive != 1:
        vispy.app.run()