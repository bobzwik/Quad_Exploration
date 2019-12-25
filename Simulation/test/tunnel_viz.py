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
import vispy.app
from vispy.scene import visuals

orient = "ENU"


pointcloud      = np.genfromtxt("./Simulation/test/pointcloud_fine.csv", delimiter=",")
pointcloud_filt = np.genfromtxt("./Simulation/test/pointcloud_grid.csv", delimiter=",")

center_pc_x = (pointcloud[:,0].max() + pointcloud[:,0].min())/2
center_pc_y = (pointcloud[:,1].max() + pointcloud[:,1].min())/2
center_pc_z = (pointcloud[:,2].max() + pointcloud[:,2].min())/2

print(len(pointcloud))
print(len(pointcloud_filt))

####################################

# # Make a canvas and add simple view
# canvas = vispy.scene.SceneCanvas(keys='interactive', show=True)
# view = canvas.central_widget.add_view()
# view.camera = vispy.scene.cameras.TurntableCamera(distance=60, center=[center_pc_x, center_pc_y, center_pc_z])

# # create scatter object and fill in the data
# scatter = visuals.Markers()
# scatter.set_data(pointcloud, edge_color=None, face_color=(1, 0, 0, .5), size=5)
# view.add(scatter)

# # add a colored 3D axis for orientation
# axis = visuals.XYZAxis(parent=view.scene)

####################################

# Make a canvas and add simple view
canvas = vispy.scene.SceneCanvas(keys='interactive', show=True, bgcolor=(0.,0.,0.))
view = canvas.central_widget.add_view()
view.camera = vispy.scene.cameras.TurntableCamera(distance=60, center=[center_pc_x, center_pc_y, center_pc_z])
# view.camera.set_range(x=[-3, 3])

# create scatter object and fill in the data
scatter = visuals.Markers()
scatter.set_data(pointcloud_filt, edge_color=None, face_color=(1, 1, 0, .5), size=2)
view.add(scatter)

# add a colored 3D axis for orientation
axis = visuals.XYZAxis(parent=view.scene)

# add grid
grid = visuals.GridLines(color=(1,1,1))
# grid.set_gl_state('translucent', cull_face=False)
view.add(grid)

####################################

if __name__ == '__main__':
    import sys
    if sys.flags.interactive != 1:
        vispy.app.run()