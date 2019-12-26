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

def sameAxisAnimation(t_all, waypoints, pos_all, quat_all, sDes_tr_all, Ts, params, xyzType, yawType, potfld, notInRange_all, inRange_all, inField_all, ifsave):
    # print(vispy.sys_info())
    
    x = pos_all[:,0]
    y = pos_all[:,1]
    z = pos_all[:,2]

    xDes = sDes_tr_all[:,0]
    yDes = sDes_tr_all[:,1]
    zDes = sDes_tr_all[:,2]

    x_wp = waypoints[:,0]
    y_wp = waypoints[:,1]
    z_wp = waypoints[:,2]

    Plot3D1 = scene.visuals.create_visual_node(vispy.visuals.LinePlotVisual)
    Plot3D2 = scene.visuals.create_visual_node(vispy.visuals.LinePlotVisual)
    Plot3D3 = scene.visuals.create_visual_node(vispy.visuals.LinePlotVisual)

    # Add Canvas
    canvas = vispy.scene.SceneCanvas(keys='interactive', show=True)
    canvas.measure_fps()
    
    # Add Camera
    view = canvas.central_widget.add_view()
    view.camera = vispy.scene.cameras.TurntableCamera(distance=30)
    
    # Add Grid
    grid = scene.visuals.GridLines(color=(1,1,1), parent=view.scene)
    
    # create scatter object and fill in the data
    scatter = visuals.Markers()
    scatter.set_data(potfld.pointcloud[np.where(notInRange_all[0,:])[0]], edge_color=None, face_color=(1, 1, 0, .2), size=6)
    # scatter.set_gl_state('translucent', cull_face=False)
    view.add(scatter)

    scatterInRange = visuals.Markers()
    scatterInRange.set_data(potfld.pointcloud[np.where(inRange_all[0,:])[0]], edge_color=None, face_color=(0, 1, 0, .2), size=6)
    # scatter.set_gl_state('translucent', cull_face=False)
    view.add(scatterInRange)

    scatterInField = visuals.Markers()
    scatterInField.set_data(potfld.pointcloud[np.where(inField_all[0,:])[0]], edge_color=None, face_color=(1, 0, 0, .2), size=6)
    # scatter.set_gl_state('translucent', cull_face=False)
    view.add(scatterInField)

    # add a colored 3D axis for orientation
    axis = visuals.XYZAxis(parent=view.scene)
    
    if (config.orient == "NED"):
        z = -z
        zDes = -zDes
        z_wp = -z_wp
        flip = view.camera.flip
        view.camera.flip = flip[0], not flip[1], flip[2]

    line1 = Plot3D1([[],[],[]], width=3, color='red', marker_size=0, parent=view.scene)
    line2 = Plot3D2([[],[],[]], width=3, color='blue', marker_size=0, parent=view.scene)
    line3 = Plot3D3([[],[],[]], width=2, color='red', marker_size=0, parent=view.scene)

    # # Setting the axes properties
    # extraEachSide = 0.5
    # maxRange = 0.5*np.array([x.max()-x.min(), y.max()-y.min(), z.max()-z.min()]).max() + extraEachSide
    # mid_x = 0.5*(x.max()+x.min())
    # mid_y = 0.5*(y.max()+y.min())
    # mid_z = 0.5*(z.max()+z.min())
    
    # ax.set_xlim3d([mid_x-maxRange, mid_x+maxRange])
    # ax.set_xlabel('X')
    # if (config.orient == "NED"):
    #     ax.set_ylim3d([mid_y+maxRange, mid_y-maxRange])
    # elif (config.orient == "ENU"):
    #     ax.set_ylim3d([mid_y-maxRange, mid_y+maxRange])
    # ax.set_ylabel('Y')
    # ax.set_zlim3d([mid_z-maxRange, mid_z+maxRange])
    # ax.set_zlabel('Altitude')

    # titleTime = ax.text2D(0.05, 0.95, "", transform=ax.transAxes)

    # trajType = ''
    # yawTrajType = ''

    # if (xyzType == 0):
    #     trajType = 'Hover'
    # else:
    #     ax.scatter(x_wp, y_wp, z_wp, color='green', alpha=1, marker = 'o', s = 25)
    #     if (xyzType == 1 or xyzType == 12):
    #         trajType = 'Simple Waypoints'
    #     else:
    #         ax.plot(xDes, yDes, zDes, ':', lw=1.3, color='green')
    #         if (xyzType == 2):
    #             trajType = 'Simple Waypoint Interpolation'
    #         elif (xyzType == 3):
    #             trajType = 'Minimum Velocity Trajectory'
    #         elif (xyzType == 4):
    #             trajType = 'Minimum Acceleration Trajectory'
    #         elif (xyzType == 5):
    #             trajType = 'Minimum Jerk Trajectory'
    #         elif (xyzType == 6):
    #             trajType = 'Minimum Snap Trajectory'
    #         elif (xyzType == 7):
    #             trajType = 'Minimum Acceleration Trajectory - Stop'
    #         elif (xyzType == 8):
    #             trajType = 'Minimum Jerk Trajectory - Stop'
    #         elif (xyzType == 9):
    #             trajType = 'Minimum Snap Trajectory - Stop'
    #         elif (xyzType == 10):
    #             trajType = 'Minimum Jerk Trajectory - Fast Stop'
    #         elif (xyzType == 1):
    #             trajType = 'Minimum Snap Trajectory - Fast Stop'

    # if (yawType == 0):
    #     yawTrajType = 'None'
    # elif (yawType == 1):
    #     yawTrajType = 'Waypoints'
    # elif (yawType == 2):
    #     yawTrajType = 'Interpolation'
    # elif (yawType == 3):
    #     yawTrajType = 'Follow'
    # elif (yawType == 4):
    #     yawTrajType = 'Zero'


    # titleType1 = ax.text2D(0.95, 0.95, trajType, transform=ax.transAxes, horizontalalignment='right')
    # titleType2 = ax.text2D(0.95, 0.91, 'Yaw: '+ yawTrajType, transform=ax.transAxes, horizontalalignment='right')   
    
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
        # titleTime.set_text(u"Time = {:.2f} s".format(time))

        view.camera.center = [x,y,z]
        scatter.set_data(potfld.pointcloud[np.where(notInRange_all[i*numFrames,:])[0]], edge_color=None, face_color=(1, 1, 0, .2), size=6)
        scatterInRange.set_data(potfld.pointcloud[np.where(inRange_all[i*numFrames,:])[0]], edge_color=None, face_color=(0, 1, 0, .2), size=6)
        scatterInField.set_data(potfld.pointcloud[np.where(inField_all[i*numFrames,:])[0]], edge_color=None, face_color=(1, 0, 0, .2), size=6)

        i += 1

    timer = app.Timer(Ts*numFrames)
    timer.connect(update)
    timer.start(iterations=len(x)/numFrames-1)

    import sys
    if sys.flags.interactive != 1:
        vispy.app.run()