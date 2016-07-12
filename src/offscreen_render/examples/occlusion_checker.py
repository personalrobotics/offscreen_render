"""Loads up an environment, attaches a viewer, loads a scene, and renders 
an occlusion image.
"""
import openravepy
import matplotlib
from matplotlib import pyplot
from openravepy import *

env = Environment() # create openrave environment

# Load some objects and set them up in the environment
env.Load('/home/mklingen/prdev/src/pr-ordata/data/objects/bowl.kinbody.xml')
env.Load('/home/mklingen/prdev/src/pr-ordata/data/objects/fuze_bottle.kinbody.xml')
bowl = env.GetKinBody('bowl')
fuze = env.GetKinBody('fuze_bottle')
tf = fuze.GetTransform()
tf[0:3, 3] = numpy.array([0.3, 0, 0.5])
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
tf[0:3, 3] = numpy.array([0.0, 0, -0.5])
sensor.SetTransform(tf)

# This is how you make it render a frame (the argument is meaningless)
sensor.SimulationStep(0.01)

# Copy data from OpenGL using the sensor interface
data = sensor.GetSensorData();

# This is now a W x H x 3 numpy array filled with bytes
img = data.imagedata

#You can plot the data if you want
matplotlib.pyplot.imshow(img)

pyplot.show();
