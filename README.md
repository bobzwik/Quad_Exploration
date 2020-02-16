# Potential Field Branch (of Quad_SimCon Project)

This branch is for developing Potential Field navigation within pointclouds. Essentially, every point of the pointcloud is an obstacle that exerts a force on the drone in order to stop it from colliding with the obstacle. In this project, the Potential Field is used to navigate within a voxelized pointcloud tunnel.

[![Watch the video](http://img.youtube.com/vi/WuDDGpTPt2g/0.jpg)](https://youtu.be/WuDDGpTPt2g)


## Pointcloud Generation
There are a few available scripts in this branch to generate simple tunnels. They take a while to execute, since a first create a very fine pointcloud, in order to make sure that once voxelized, there are no gaps between points. By voxelized, I mean that I create a corse Cartesian grid (0.25 m between "voxel" centers) and every point of the fine pointcloud is approximated to its closest "voxel".

## Visualization Using Vispy
Since `Matplotlib` only uses CPU to compute the drawing, it will not be fast enough to to draw pointclouds of thousands of points. I therefore opted to use the [`Vispy`](http://vispy.org/) package for visualization. It uses OpenGL in order to use the power of GPUs.

There are currently 3 different methods to visualize the pointcloud and the points that are withing the Potential Field.

1. **Using 2D `Markers`.** This method runs fast but lacks depth perception (markers that are close or far are drawn the same size).

2. **Using 3D `BoxMarkers` (points in the Potential Field have their face color updated).** This method is currently slow (work in progress), but depth is easier to understand. The markers are 3D cubes, but updating the face color of certain faces is currently CPU intensive.

3. **Using two separate 3D `BoxMarkers` instance (a fixed one for the pointcloud tunnel, and a dynamic one for the points in the Potential Field)** The pointcloud tunnel is again presented as 3D cubes, and the points in the Potential Field use a second instance of `BoxMarkers`. As updating that smaller instance is less CPU intensive, is runs a bit faster (but work in progress).

For now, you can select the visualizing method by commenting and uncommenting lines in the `utils\__init__.py` file.

### Vispy Installation
To be able to visualize the animation, you need to first install `Vispy` and its dependancies. Vispy requires a OpenGL backend [`PyQt5`](https://pypi.org/project/PyQt5/), [`Pyglet`](https://pypi.org/project/pyglet/) or others) (I use `PyQt5`). You also need [`PyOpenGL`](https://pypi.org/project/PyOpenGL/).

If you have the pip package manager installed you can simply type:

`$ pip install vispy PyQt5 pyopengl pyopengl_accelerate` 


## Potential Field Implementation


### Useful links


## To-Do
* Implement "Extended Potential Field" to reduce oscillations
* Improve Vispy performance
* Implement "Frontier Based Exploration" (so learn about ray-casting and possibly octomaps)
* Use the `multiprocessing` Python package to parallelize different tasks (a process for the quadrotor dynamics, one for the control, one for the "Frontier Based Exploration" and one for the visualization) ([dev branch](https://github.com/bobzwik/Quadcopter_SimCon/tree/potentialField_multiprocess))
