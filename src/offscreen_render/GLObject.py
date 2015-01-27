import sys
import numpy as np
import OpenGL.GL as gl
import OpenGL.GLUT as glut
import vispy.gloo as gloo
import transforms as tf
import openravepy
from openravepy import *

class GLObject:
    def __init__(self, body, world, geometry, color):
        self.body = body;
        trimesh = geometry.GetCollisionMesh()

        V = np.zeros(len(trimesh.vertices),
                        dtype = [("position", np.float32, 3),
                        ("color", np.float32, 3)])

        V["position"] = trimesh.vertices
        V["color"] = np.tile(color, (len(trimesh.vertices), 1))


        self.vertex_buffer = gloo.VertexBuffer(V)
        self.index_buffer = gloo.IndexBuffer(trimesh.indices)
        self.world = np.dot(world, geometry.GetTransform())