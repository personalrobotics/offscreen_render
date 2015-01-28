#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Very simple transformation library that is needed for some examples.
"""

import math
import numpy
import numpy as np

def invert(K):
    M = K
    R = M[0:2, 0:2]
    M[0:2, 0:2] = np.transpose(R)
    M[0:2, 3] = -np.dot(np.transpose(R), M[0:2, 3])
    return M

def translate(M, x, y=None, z=None):
    """
    translate produces a translation by (x, y, z) . 
    
    Parameters
    ----------
    x, y, z
        Specify the x, y, and z coordinates of a translation vector.
    """
    if y is None: y = x
    if z is None: z = x
    T = [[ 1, 0, 0, x],
         [ 0, 1, 0, y],
         [ 0, 0, 1, z],
         [ 0, 0, 0, 1]]
    T = np.array(T, dtype=np.float32).T
    M[...] = np.dot(M,T)


def scale(M, x, y=None, z=None):
    """
    scale produces a non uniform scaling along the x, y, and z axes. The three
    parameters indicate the desired scale factor along each of the three axes.

    Parameters
    ----------
    x, y, z
        Specify scale factors along the x, y, and z axes, respectively.
    """
    if y is None: y = x
    if z is None: z = x
    S = [[ x, 0, 0, 0],
         [ 0, y, 0, 0],
         [ 0, 0, z, 0],
         [ 0, 0, 0, 1]]
    S = np.array(S,dtype=np.float32).T
    M[...] = np.dot(M,S)


def xrotate(M,theta):
    t = math.pi*theta/180
    cosT = math.cos( t )
    sinT = math.sin( t )
    R = numpy.array(
        [[ 1.0,  0.0,  0.0, 0.0 ],
         [ 0.0, cosT,-sinT, 0.0 ],
         [ 0.0, sinT, cosT, 0.0 ],
         [ 0.0,  0.0,  0.0, 1.0 ]], dtype=np.float32)
    M[...] = np.dot(M,R)

def yrotate(M,theta):
    t = math.pi*theta/180
    cosT = math.cos( t )
    sinT = math.sin( t )
    R = numpy.array(
        [[ cosT,  0.0, sinT, 0.0 ],
         [ 0.0,   1.0,  0.0, 0.0 ],
         [-sinT,  0.0, cosT, 0.0 ],
         [ 0.0,  0.0,  0.0, 1.0 ]], dtype=np.float32)
    M[...] = np.dot(M,R)

def zrotate(M,theta):
    t = math.pi*theta/180
    cosT = math.cos( t )
    sinT = math.sin( t )
    R = numpy.array(
        [[ cosT,-sinT, 0.0, 0.0 ],
         [ sinT, cosT, 0.0, 0.0 ],
         [ 0.0,  0.0,  1.0, 0.0 ],
         [ 0.0,  0.0,  0.0, 1.0 ]], dtype=np.float32)
    M[...] = np.dot(M,R)


def rotate(M, angle, x, y, z, point=None):
    """
    rotate produces a rotation of angle degrees around the vector (x, y, z).
    
    Parameters
    ----------
    M
       Current transformation as a numpy array

    angle
       Specifies the angle of rotation, in degrees.

    x, y, z
        Specify the x, y, and z coordinates of a vector, respectively.
    """
    angle = math.pi*angle/180
    c,s = math.cos(angle), math.sin(angle)
    n = math.sqrt(x*x+y*y+z*z)
    x /= n
    y /= n
    z /= n
    cx,cy,cz = (1-c)*x, (1-c)*y, (1-c)*z
    R = numpy.array([[ cx*x + c  , cy*x - z*s, cz*x + y*s, 0],
                     [ cx*y + z*s, cy*y + c  , cz*y - x*s, 0],
                     [ cx*z - y*s, cy*z + x*s, cz*z + c,   0],
                     [          0,          0,        0,   1]]).T
    M[...] = np.dot(M,R)


def ortho( left, right, bottom, top, znear, zfar ):
    assert( right  != left )
    assert( bottom != top  )
    assert( znear  != zfar )
    
    M = np.zeros((4,4), dtype=np.float32)
    M[0,0] = +2.0/(right-left)
    M[3,0] = -(right+left)/float(right-left)
    M[1,1] = +2.0/(top-bottom)
    M[3,1] = -(top+bottom)/float(top-bottom)
    M[2,2] = -2.0/(zfar-znear)
    M[3,2] = -(zfar+znear)/float(zfar-znear)
    M[3,3] = 1.0
    return M
        
def frustum( left, right, bottom, top, znear, zfar ):
    assert( right  != left )
    assert( bottom != top  )
    assert( znear  != zfar )

    M = np.zeros((4,4), dtype=np.float32)
    M[0,0] = +2.0*znear/(right-left)
    M[2,0] = (right+left)/(right-left)
    M[1,1] = +2.0*znear/(top-bottom)
    M[3,1] = (top+bottom)/(top-bottom)
    M[2,2] = -(zfar+znear)/(zfar-znear)
    M[3,2] = -2.0*znear*zfar/(zfar-znear)
    M[2,3] = -1.0
    return M



def look_at(eyePosition3D, center3D, upVector3D):
    forward = center3D - eyePosition3D;
    #------------------

    forward /= (np.sqrt(np.dot(forward, forward)))
    #------------------
    #Side = forward x up
    side = np.cross(forward, upVector3D)
    side /= np.sqrt(np.dot(side, side))
    #------------------
    #Recompute up as: up = side x forward
    up = np.cross(side, forward)

    matrix2 = np.zeros((4, 4), dtype=np.float32)
    #------------------
    matrix2[0, 0] = side[0]
    matrix2[1, 0] = side[1]
    matrix2[2, 0] = side[2]
    matrix2[3, 0] = 0.0
    #-----------------
    matrix2[0, 1] = up[0]
    matrix2[1, 1] = up[1]
    matrix2[2, 1] = up[2]
    matrix2[3, 1] = 0.0
    #-----------------
    matrix2[0, 2] = -forward[0]
    matrix2[1, 2] = -forward[1]
    matrix2[2, 2] = -forward[2]
    matrix2[3, 2] = 0.0
    matrix2[3, 3] = 1.0
    matrix2[0:3, 3] = np.array([-eyePosition3D[0], -eyePosition3D[1], -eyePosition3D[2]]);
    return matrix2;

def perspective_camera_calib(fx, fy, cx, cy, near, far, width, height):
    return np.array([[2 * fx / width, 0, 2 * (0.5 - cx / width), 0], 
                     [0, 2 * fy / height, 2 * (cy / height - 0.5), 0], 
                     [0, 0, -(far + near) / (far - near), -2 * far * near / (far - near)], 
                     [0, 0, -1, 0]]);

def perspective(fovy, aspect, znear, zfar):
    assert( znear != zfar )
    h = np.tan(fovy / 360.0 * np.pi) * znear
    w = h * aspect
    return frustum( -w, w, -h, h, znear, zfar )