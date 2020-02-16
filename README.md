# Artificial Potential Field Branch (of Quad_SimCon Project)

This branch is for developing Artificial Potential Field navigation within pointclouds. Essentially, every point of the pointcloud is an obstacle that exerts a force on the drone in order to stop it from colliding with the obstacle. In this project, the Potential Field is used to navigate within a voxelized pointcloud tunnel.

[![Watch the video](http://img.youtube.com/vi/WuDDGpTPt2g/0.jpg)](https://youtu.be/WuDDGpTPt2g)


## Pointcloud Generation
There are a few available scripts in this branch to generate simple tunnels. They take a while to execute, since a first create a very fine pointcloud, in order to make sure that once voxelized, there are no gaps between points. By voxelized, I mean that I create a corse Cartesian grid (0.25 m between "voxel" centers) and every point of the fine pointcloud is approximated to its closest "voxel".

## Artificial Potential Field Implementation
In this Artificial Potential Field, there is no **explicit** attractive force, as the flight control already commands a velocity, given a desired position. This velocity command is saturated. The velocity control then computes a desired thrust/force, which in turn indicates the desired quadrotor attitude (orientation) ([See 'master' readme and PX4 controller](https://github.com/bobzwik/Quadcopter_SimCon/tree/master))

The repulsive force is calculated using a widely used algorithm (see **Siegwart's Introduction to Autonomous Mobile Robots**). Every point of the voxelized pointcloud is considered an object. This force is then implemented into the controller through 3 different possible ways.

1. Added to the saturated "attractive" velocity. The resulting desired velocity is then saturated again (to prevent large velocity commands).

2. Added to the desired thrust/force, which is then saturated (to prevent large force commands). The saturation is defined using a specified maximum tilt of the quadrotor.

3.  Added to the desired thrust/force after its saturation.

Currently, method # 1 is used, and the repulsive gain is set accordingly.


### Useful links
* [Youtube video on Artificial Potential Field](https://youtu.be/8Vva0bnMIEI)
* [Slides on Artificial Potential Field](https://www.cs.cmu.edu/~motionplanning/lecture/Chap4-Potential-Field_howie.pdf)
* [More slides](https://www.dis.uniroma1.it/~oriolo/amr/slides/MotionPlanning3_Slides.pdf)
* A pdf of Siegwart's book can easily be found online ;)


## Visualization Using Vispy
Since `Matplotlib` only uses CPU to compute the drawing, it will not be fast enough to to draw pointclouds of thousands of points. I therefore opted to use the [`Vispy`](http://vispy.org/) package for visualization. It uses OpenGL in order to use the power of GPUs.

There are currently 3 different methods to visualize the pointcloud and the points that are withing the Potential Field.

1. **Using 2D `Markers`.** This method runs fast but lacks depth perception (markers that are close or far are drawn the same size).

2. **Using 3D `BoxMarkers` (points in the Potential Field have their face color updated).** This method is currently slow (work in progress), but depth is easier to understand. The markers are 3D cubes, but updating the face color of certain faces is currently CPU intensive.

3. **Using two separate 3D `BoxMarkers` instances (a fixed one for the pointcloud tunnel, and a dynamic one for the points in the Potential Field).** The pointcloud tunnel is again presented as 3D cubes, and the points in the Potential Field use a second instance of `BoxMarkers`. As updating that smaller instance is less CPU intensive, is runs a bit faster (but work in progress).

For now, you can select the visualizing method by commenting and uncommenting lines in the `utils\__init__.py` file.

### Vispy Installation
To be able to visualize the animation, you need to first install `Vispy` and its dependancies. Vispy requires a OpenGL backend [`PyQt5`](https://pypi.org/project/PyQt5/), [`Pyglet`](https://pypi.org/project/pyglet/) or others) (I use `PyQt5`). You also need [`PyOpenGL`](https://pypi.org/project/PyOpenGL/).

If you have the pip package manager installed you can simply type:

`$ pip install vispy PyQt5 pyopengl pyopengl_accelerate` 

### OS/GPU settings tweaks

## To-Do
* Implement "Extended Potential Field" to reduce oscillations
* Improve Vispy performance
* Implement "Frontier Based Exploration" (so learn about ray-casting and possibly octomaps)
* Use the `multiprocessing` Python package to parallelize different tasks (a process for the quadrotor dynamics, one for the control, one for the "Frontier Based Exploration" and one for the visualization) ([dev branch](https://github.com/bobzwik/Quadcopter_SimCon/tree/potentialField_multiprocess))
