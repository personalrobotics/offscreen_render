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
from catkin.find_in_workspaces import find_in_workspaces
import parse_intrinsics

env = Environment() # create openrave environment

# Load some objects and set them up in the environment
bowl_path = find_in_workspaces(search_dirs=['share'],
                      project="pr_ordata",
                      path="data/objects/bowl.kinbody.xml",
                      first_match_only=True)[0]

fuze_bottle_path = find_in_workspaces(search_dirs=['share'],
                      project="pr_ordata",
                      path="data/objects/fuze_bottle.kinbody.xml",
                      first_match_only=True)[0]

env.Load(bowl_path)
env.Load(fuze_bottle_path)


bowl = env.GetKinBody('bowl')
fuze = env.GetKinBody('fuze_bottle')

# Set the object positions
tf = fuze.GetTransform()
tf[0:3, 3] = numpy.array([0.3, 0, 0.5])
bowl.SetTransform(tf)

# Position the camera
tf[0:3, 3] = numpy.array([0.1, 0, -0.2])

# Read intrinsics
intrinsics_path = find_in_workspaces(search_dirs=['share'],
                      project="herb_description",
                      path="camera/kinect2/502845441942/calib_color.yaml",
                      first_match_only=True)[0]
fx, fy, cx, cy = parse_intrinsics.getIntrinsics(intrinsics_path)

# Initialize the camera
camera = interface_wrapper.SimCamera(env, transform=tf,
                 #fx=529, fy=525, cx=328, cy=267, near=0.01, far=10.0, 
                 fx = fx, fy = fy, cx = cx, cy = cy, near = 0.01, far = 10.0,
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
