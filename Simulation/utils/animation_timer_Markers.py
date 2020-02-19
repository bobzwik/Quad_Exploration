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
from vispy.color import ColorArray
import time
import sys

import utils
from utils.boxmarkers import BoxMarkers
import config

rad2deg = 180.0/pi
deg2rad = pi/180.0

class ColorMarkers(visuals.Markers):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
    
    def set_color(self, face_color='white'):
        face_color = ColorArray(face_color).rgba
        if len(face_color) == 1:
            face_color = face_color[0]

        self._data['a_bg_color'] = face_color
        self._vbo.set_data(self._data)
        self.update()

class NonUpdatingTurntable(vispy.scene.cameras.TurntableCamera):
    def __init__(self, **kwargs):
        super(NonUpdatingTurntable, self).__init__(**kwargs)
    
    @property
    def azimuth(self):
        """ The angle of the camera in degrees around the y axis. An angle of
        0 places the camera within the (y, z) plane.
        """
        return self._azimuth

    @azimuth.setter
    def azimuth(self, azim):
        azim = float(azim)
        while azim < -180:
            azim += 360
        while azim > 180:
            azim -= 360
        self._azimuth = azim
        # self.view_changed()       # Don't update view right now
    
    @property
    def center(self):
        """ The center location for this camera

        The exact meaning of this value differs per type of camera, but
        generally means the point of interest or the rotation point.
        """
        return self._center or (0, 0, 0)
    
    @center.setter
    def center(self, val):
        if len(val) == 2:
            self._center = float(val[0]), float(val[1]), 0.0
        elif len(val) == 3:
            self._center = float(val[0]), float(val[1]), float(val[2])
        else:
            raise ValueError('Center must be a 2 or 3 element tuple')
        # self.view_changed()       # Don't update view right now

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

    # Create color array
    color_points = (1, 1, 0.5, 1)
    color_field  = (1, 0, 0, 1)
    color_edges  = (0, 0, 0, 0.3)
    colors = np.ones((potfld.num_points, 4), dtype=np.float32)
    canvas.yellowPoints = np.where(np.logical_or(notInRange_all[0,:], inRange_all[0,:]))[0]
    canvas.redPoints = np.where(inField_all[0,:])[0]

    colors[canvas.yellowPoints] = color_points
    colors[canvas.redPoints] = color_field

    # create scatter object and fill in the data
    scatter = ColorMarkers()
    scatter.set_data(canvas.pointcloud, edge_color=color_edges, face_color=colors, size=6)
    # scatter.set_gl_state('translucent', cull_face=False)
    view.add(scatter)

    # Add a colored 3D axis for orientation
    axis = visuals.XYZAxis(parent=view.scene)

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
            canvas.yellowPoints = np.where(np.logical_or(notInRange_all[idx_now,:], inRange_all[idx_now,:]))[0]
            canvas.redPoints = np.where(inField_all[idx_now,:])[0]
            colors[canvas.yellowPoints] = color_points
            colors[canvas.redPoints] = color_field

            scatter.set_color(face_color=colors)

            # Change camera angle and position
            view.camera.azimuth = view.camera.azimuth-psi_diff
            view.camera.center = [x, y, z]
            
            # Update view
            view.camera.view_changed()
            
            # Set previous index
            canvas.idx_prev = idx_now
    
    canvas.timer.connect(update)
    canvas.timer.start()

    if sys.flags.interactive != 1:
        vispy.app.run()