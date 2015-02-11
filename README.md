# offscreen_render
A utility for rendering OpenRAVE kinbodies offscreen to get properties like depth, occlusion, color, etc.

##Installation Requirements##

The project depends on the following:

* OpenRAVE (specifically, the github package [openrave_catkin](https://github.com/personalrobotics/openrave_catkin))
* ROS Hydro
* PCL 1.6 or higher (included in ROS-hydro-desktop-full)
* OpenGL 2.1 or higher (included in most linux distros)
* GLEW (included in GLFW)
* [GLFW framework 3.0](http://www.glfw.org/) or higher

##Object Tracker Usage##

This project builds an OpenRAVE object tracker plugin called "object_tracker". The plugin requires the following inputs:

* A depth image
* A point cloud

Using these, it generates synthetic point clouds of the desired objects, and optimizes the object pose using a particle filter from `pcl_tracking`. The tracker uses depth only, and can only render the collision models of objects.

To run in python:

    #Load the tracker from a plugin
    tracker = tracker = RaveCreateModule(env, "object_tracker")
    # Only need to run this once to initialize the tracker
    tracker.SendCommand("Initialize <depth_camera_info_topic> <point_cloud_topic>")
    # Blocks until the number of iterations is achieved. Moves the openrave object into alignment.
    tracker.SendCommand("Track <openrave_object_name> <number_of_tracking_iters>")
    
The object needs to be localized fairly well (within 10 cm) before tracking; but the result is pretty decent alignment.

![Image](http://i.imgur.com/hhyGqER.png)

The tracker creates an offscreen window to get an OpenGL context. Do not close it. It also publishes a synthetic point cloud to `offscreen_render/synth_point_cloud` for debugging purposes.
