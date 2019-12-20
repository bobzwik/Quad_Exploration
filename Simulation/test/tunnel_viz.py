# -*- coding: utf-8 -*-
"""
author: John Bass
email: john.bobzwik@gmail.com
license: MIT
Please feel free to use and modify this, but keep the above information. Thanks!
"""

import numpy as np
from numpy import pi
import vispy.scene
from vispy import app
from vispy.scene import visuals

orient = "ENU"


pointcloud      = np.genfromtxt("./Simulation/test/pointcloud_fine.csv", delimiter=",")
pointcloud_filt = np.genfromtxt("./Simulation/test/pointcloud_grid.csv", delimiter=",")

####################################

# Make a canvas and add simple view
canvas = vispy.scene.SceneCanvas(keys='interactive', show=True)
view = canvas.central_widget.add_view()
# view.camera = vispy.scene.cameras.TurntableCamera(distance=15)
view.camera = vispy.scene.cameras.FlyCamera(fov=60)

# create scatter object and fill in the data
scatter = visuals.Markers()
scatter.set_data(pointcloud, edge_color=None, face_color=(1, 1, 1, .5), size=5)
view.add(scatter)

# add a colored 3D axis for orientation
axis = visuals.XYZAxis(parent=view.scene)

####################################

# Make a canvas and add simple view
canvas = vispy.scene.SceneCanvas(keys='interactive', show=True)
view = canvas.central_widget.add_view()
# view.camera = vispy.scene.cameras.TurntableCamera(distance=15)
view.camera = vispy.scene.cameras.FlyCamera(fov=60)

# create scatter object and fill in the data
scatter = visuals.Markers()
scatter.set_data(pointcloud_filt, edge_color=None, face_color=(1, 1, 1, .5), size=5)
view.add(scatter)

# add a colored 3D axis for orientation
axis = visuals.XYZAxis(parent=view.scene)

####################################

if __name__ == '__main__':
    import sys
    if sys.flags.interactive != 1:
        vispy.app.run()