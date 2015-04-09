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

##Occlusion Renderer Usage##

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
