#! /usr/bin/env python
# -*- coding: utf-8 -*-
# -----------------------------------------------------------------------------
# Copyright (c) 2014, Nicolas P. Rougier. All rights reserved.
# Distributed under the terms of the new BSD License.
# -----------------------------------------------------------------------------
import sys
import numpy as np
import OpenGL.GL as gl
import OpenGL.GLUT as glut
import vispy.gloo
import transforms

vertex = """
uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
attribute vec3 position;
varying float distToCamera;
void main()
{
    vec4 cs_position =  view * model * vec4(position, 1.0);
    distToCamera = -cs_position.z;
    gl_Position = projection * cs_position;
}
"""

fragment = """
varying float distToCamera;

void main()
{
    gl_FragColor = vec4(distToCamera, distToCamera, distToCamera, 1.0);
}
"""


def display():
    gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)
    program.draw(gl.GL_TRIANGLES, indices)
    glut.glutSwapBuffers()

def reshape(width,height):
    gl.glViewport(0, 0, width, height)
    projection = transforms.perspective( 45.0, width/float(height), 0.01, 2.0 )
    program['projection'] = projection

def keyboard(key, x, y):
    if key == '\033': sys.exit( )

def timer(fps):
    global theta, phi
    theta += .5
    phi += .5
    model = np.eye(4, dtype=np.float32)
    transforms.translate(model, 0, 0, 0)
    transforms.rotate(model, theta, 0,0,1)
    transforms.rotate(model, phi, 0,1,0)
    program['model'] = model
    glut.glutTimerFunc(1000/fps, timer, fps)
    glut.glutPostRedisplay()


# Glut init
# --------------------------------------
glut.glutInit(sys.argv)
glut.glutInitDisplayMode(glut.GLUT_DOUBLE | glut.GLUT_RGBA | glut.GLUT_DEPTH)
glut.glutCreateWindow('Rotating Cube')
glut.glutReshapeWindow(512,512)
glut.glutReshapeFunc(reshape)
glut.glutKeyboardFunc(keyboard )
glut.glutDisplayFunc(display)
glut.glutTimerFunc(1000/60, timer, 60)

# Build cube data
# --------------------------------------
V = np.zeros(8, [("position", np.float32, 3)])
V["position"] = [[ 0.1, 0.1, 0.1], [-0.1, 0.1, 0.1], [-0.1,-0.1, 0.1], [ 0.1,-0.1, 0.1],
                 [ 0.1,-0.1,-0.1], [ 0.1, 0.1,-0.1], [-0.1, 0.1,-0.1], [-0.1,-0.1,-0.1]]
vertices = vispy.gloo.VertexBuffer(V)

I = [0,1,2, 0,2,3,  0,3,4, 0,4,5,  0,5,6, 0,6,1,
     1,6,7, 1,7,2,  7,4,3, 7,3,2,  4,7,6, 4,6,5]
indices = vispy.gloo.IndexBuffer(I)

# Build program
# --------------------------------------
program = vispy.gloo.Program(vertex, fragment)
program.bind(vertices)

# Build view, model, projection & normal
# --------------------------------------
view = np.eye(4,dtype=np.float32)
model = np.eye(4,dtype=np.float32)
projection = np.eye(4,dtype=np.float32)
transforms.translate(view, 0,0,-0.5)
transforms.translate(model, 0, 0, 0.1)
program['model'] = model
program['view'] = view
phi, theta = 0,0

# OpenGL initalization
# --------------------------------------
gl.glClearColor(1,1,1,1)
gl.glEnable(gl.GL_DEPTH_TEST)

# Start
# --------------------------------------
glut.glutMainLoop()