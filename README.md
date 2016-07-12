# offscreen_render
A utility for rendering OpenRAVE kinbodies offscreen to get properties like depth, occlusion, color, etc. Also simulates ROS cameras and tracks OpenRAVE objects. This library was developed by [Matthew Klingensmith](https://github.com/mklingen) and contributed to by [Shushman Choudhury](https://github.com/Shushman)

##Installation Requirements##

The project depends on the following:

* OpenRAVE (specifically, the github package [openrave_catkin](https://github.com/personalrobotics/openrave_catkin))
* ROS Hydro
* PCL 1.6 or higher (included in ROS-hydro-desktop-full)
* OpenGL 2.1 or higher (included in most linux distros)
* GLEW (included in GLFW)
* [GLFW framework 3.0](http://www.glfw.org/) or higher
```python
sudo add-apt-repository ppa:keithw/glfw3
sudo apt-get update
sudo apt-get install libglfw3-dev
```

##Python Wrappers##
Probably the easiest way to use this package is with one of the python wrappers. (`ros_camera_sim.py` and `interface_wrapper.py`). This  wraps the OpenRAVE plugin and handles the update thread, etc. Here's how to use it:

###Ros Camera Sim###
```python
# Create a simulated camera in the given environment
camera = RosCameraSim(env)
# Start looping by simulating a ROS camera with the given topic.
camera.start('/head/kinect2/qhd')
# Add or remove bodies from the environment
camera.add_body(my_body);
```
The images/pointclouds get published to ROS.


###Non-ROS Wrapper###
```python
# Initialize the camera
camera = interface_wrapper.SimCamera(env, transform=tf,
                 fx=529, fy=525, cx=328, cy=267, near=0.01, far=10.0, 
                 width=640, height=480)
                 
# Add the bodies to the camera renderer
camera.add_body(my_body)
# This is now a W x H x 3 numpy array filled with bytes
img = camera.render()
````
You can use the camera to query information about the scene like occlusion, for instance.

##Simple Occlusion Example##

Here is a script for determining how many pixels of a specific object are visible to the camera.

```python
# Load some objects and set them up in the environment
env.Load('/home/mklingen/prdev/src/pr-ordata/data/objects/bowl.kinbody.xml')
env.Load('/home/mklingen/prdev/src/pr-ordata/data/objects/fuze_bottle.kinbody.xml')
bowl = env.GetKinBody('bowl')
fuze = env.GetKinBody('fuze_bottle')
tf = fuze.GetTransform()
tf[0:3, 3] = numpy.array([0, 0, 1])
fuze.SetTransform(tf)

# Create the sensor
sensor = openravepy.RaveCreateSensor(env, 'offscreen_render_camera')
# Set its intrinsics (fx, fy, cx, cy, near, far)
sensor.SendCommand('setintrinsic 529 525 328 267 0.01 10')
# And its resolution in pixels
sensor.SendCommand('setdims 640 480')
#Initialize the sensor. Right now the size of the sensor can't be changed after you do this.
# It will also open up an annoying opengl window just to get context.
sensor.Configure(openravepy.Sensor.ConfigureCommand.PowerOn)
# Add bodies to render with the given (r, g, b) colors
sensor.SendCommand('addbody bowl 255 0 0')
sensor.SendCommand('addbody fuze_bottle 0 255 0')
# You can also set the sensor's extrinsic transform
sensor.SetTransform([...])
# This is how you make it render a frame (the argument is meaningless)
sensor.SimulationStep(0.01)

# Copy data from OpenGL using the sensor interface
data = sensor.GetSensorData();

# This is now a W x H x 3 numpy array filled with bytes
img = data.imagedata

#You can plot the data if you want
matplotlib.pyplot.imshow(img)
```
![image](https://camo.githubusercontent.com/1ffdf9d3c652d0d8e5ff2ffdcb99b28824b27eb0/687474703a2f2f692e696d6775722e636f6d2f7a61566576354a2e706e67)


##Advanced Usage##
It's also possible to directly modify the data that's being drawn. The renderer supports vertex coloring, and allows you to modify the position of vertices.

```python
"""Loads up an environment, attaches a viewer, loads a scene, and renders 
images using the advanced interface.
"""
import openravepy
import matplotlib
import random
from matplotlib import pyplot
from openravepy import *
from offscreen_render import interface_wrapper
from mpl_toolkits.mplot3d import Axes3D

env = Environment() # create openrave environment

# Load some objects and set them up in the environment
env.Load('/home/mklingen/prdev/src/pr-ordata/data/objects/bowl.kinbody.xml')
env.Load('/home/mklingen/prdev/src/pr-ordata/data/objects/fuze_bottle.kinbody.xml')
bowl = env.GetKinBody('bowl')
fuze = env.GetKinBody('fuze_bottle')

# Set the object positions
tf = fuze.GetTransform()
tf[0:3, 3] = numpy.array([0.3, 0, 0.5])
bowl.SetTransform(tf)

# Position the camera
tf[0:3, 3] = numpy.array([0.1, 0, -0.2])

# Initialize the camera
camera = interface_wrapper.SimCamera(env, transform=tf,
                 fx=529, fy=525, cx=328, cy=267, near=0.01, far=10.0, 
                 width=640, height=480)
                 
# Add the bodies to the camera renderer
camera.add_body(fuze)
camera.add_body(bowl)

# Now we're going to query some values that the renderer is using
# internally to render the objects. Each kinbody has some number of
# links and some number of geometries for each link. This fuze bottle
# has one link and two geometries per link. Apparently geometry 0 is empty
# for the fuze bottle, and geometry 1 has the actual data. We just need
# to pass in the link and geometry index to get the data associated with
# this mesh. Note that the OpenRAVE collision meshes are used here,
# not the visual meshes! Also note that these functions are pretty slow,
# so use them sparingly.
# Mesh positions are stored as a flat array of x, y, z values
positions = camera.get_link_mesh_positions(fuze, 0, 1)
# Mesh colors are stored as a flat array of r, g, b values
colors = camera.get_link_mesh_colors(fuze, 0, 1)
# The triangle index is a flat array of unsigned short values. Each group
# of three values corresponds to a triangle in the mesh.
indices = camera.get_link_mesh_indices(fuze, 0, 1)

# Print the values to see what they are!
print 'vertex position buffer: '
print positions
print 'vertex color buffer: '
print colors
print 'triangle index buffer: '
print indices


# Now we're going to randomize the positions and colors
num_colors = colors.shape
num_positions = positions.shape

# Add 1 cm of random noise to the positions
for i in xrange(0, num_positions[0]):
    positions[i] += random.random() * 0.01

# Randomly set the colors (valid values are between 0 and 1)
for i in xrange(0, num_colors[0]):
    colors[i] = random.random()

# Now set the positions and colors for geometry 1 of link 0 of the fuze
# bottle. Note that these functions are pretty slow, so use them sparingly.    
camera.set_link_mesh_positions(fuze, positions, 0, 1)
camera.set_link_mesh_colors(fuze, colors, 0, 1)

# This is now a W x H x 3 numpy array filled with bytes
img = camera.render()

#You can plot the data if you want
matplotlib.pyplot.imshow(img)

pyplot.show();
```
![img](http://i.imgur.com/nCNkQWZ.png)

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
