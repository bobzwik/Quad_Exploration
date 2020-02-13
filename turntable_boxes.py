# -*-coding: utf-8 -*-
# vispy: gallery 30
# -----------------------------------------------------------------------------
# Copyright (c) Vispy Development Team. All Rights Reserved.
# Distributed under the (new) BSD License. See LICENSE.txt for more info.
# -----------------------------------------------------------------------------
"""
Simple use of SceneCanvas to display a cube with an arcball camera.
"""
import sys
import numpy as np
import vispy
import boxmarkers
from vispy import scene
from vispy.color import Color
from vispy import app
from vispy.visuals.transforms import STTransform

# print(vispy.sys_info())

# Pointcloud
pc_size = 100000

point_x = np.arange(pc_size)*0.1
point_y = np.zeros(pc_size)
point_z = np.zeros(pc_size)

pointcloud = np.array([point_x, point_y, point_z]).T

# Canvas
canvas = scene.SceneCanvas(keys='interactive', size=(800, 600), show=True)
canvas.measure_fps()

# Set up a viewbox to display the cube with interactive arcball
view = canvas.central_widget.add_view()
view.bgcolor = '#efefef'
view.camera = scene.cameras.TurntableCamera()
view.padding = 100
# print(pointcloud.shape[0])
color = Color((1,0,0))
cubes = boxmarkers.BoxMarkers(pointcloud, 1, 1, 1, color=color, edge_color="black", parent=view.scene)
# cubes = [boxmarkers.BoxMarkers(pointcloud[i], 1, 1, 1, color=color, edge_color="black", parent=view.scene) for i in range(len(pointcloud))]
# for cube, i in zip(cubes, range(len(cubes))):
#     transform = STTransform(translate=pointcloud[i])
#     cube.transform = transform




if __name__ == '__main__' and sys.flags.interactive == 0:
    canvas.app.run()
