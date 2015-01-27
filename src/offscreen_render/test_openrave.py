#! /usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import numpy as np
import OpenGL.GL as gl
import OpenGL.GLUT as glut
import vispy.gloo as gloo
import transforms as tf
import openravepy
import time
from openravepy import *
import OffscreenRenderer;

def main():
    try:
        openravepy.RaveInitialize(load_all_plugins=True)
        env = openravepy.Environment()
        env.Load('robots/barrettwam.robot.xml')
        env.SetViewer('qtcoin');
        robot = env.GetRobots()[0];
        viewer = env.GetViewer();
        time.sleep(3)
        camera_transform = viewer.GetCameraTransform();
        renderer = OffscreenRenderer.OffscreenRenderer(640, 480, env);
        renderer.set_view_projection_matrix(640., 480., 640., 480.,320., 240., 0.01, 4.,camera_transform);
        renderer.add_kinbody(robot);
        renderer.initialize_context()
        renderer.loop()

    except openravepy.openrave_exception, e:
        print e

if __name__ == "__main__":
    main()