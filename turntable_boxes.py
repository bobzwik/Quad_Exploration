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
from boxmarkers import BoxMarkers
from vispy import scene
from vispy.color import Color
from vispy import app
from vispy.visuals.transforms import STTransform

# print(vispy.sys_info())

# Pointcloud
pc_size = 2

point_x = np.arange(pc_size)*2
point_y = np.zeros(pc_size)
point_z = np.zeros(pc_size)

pointcloud = np.array([point_x, point_y, point_z]).T

# Canvas
canvas = scene.SceneCanvas(keys='interactive', size=(800, 600), show=True)
canvas.measure_fps()

# Set up a viewbox to display the cube with interactive arcball
view = canvas.central_widget.add_view()
view.bgcolor = [0,0,0]
view.camera = scene.cameras.TurntableCamera()

# Insert cubes
color = Color((1,0,0,0.5))
cubes = BoxMarkers(pointcloud, 1, 1, 1, color=color, edge_color="black", parent=view.scene)

print(cubes.mesh.mesh_data.get_vertices())



if __name__ == '__main__' and sys.flags.interactive == 0:
    canvas.app.run()
