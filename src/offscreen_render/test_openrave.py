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
        env.Load('/homes/mklingen/prdev_catkin/src/ada/ada_description/ordata/robots/mico-modified.robot.xml')
        env.SetViewer('qtcoin');
        robot = env.GetRobots()[0];
        viewer = env.GetViewer();
        #sensors = env.GetSensors();
        #sensor = sensors[0];
        #geometry = sensor.GetSensorGeometry(Sensor.Type.Camera)
        #fx = geometry.KK[0, 0];
        #fy = geometry.KK[1, 1];
        #cx = geometry.KK[0, 2];
        #cy = geometry.KK[1, 2];
        fx = 640;
        fy = 480;
        cx = fx / 2;
        cy = fy / 2;
        width = 640;
        height = 480;
        print (fx, fy, cx, cy)
        time.sleep(3)
        camera_transform = viewer.GetCameraTransform();
        renderer = OffscreenRenderer.OffscreenRenderer(width, height, env);
        renderer.set_view_projection_matrix(width, height, fx, fy, cx, cy, 0.01, 4.,camera_transform);
        renderer.add_kinbody(robot);
        renderer.initialize_context()
        renderer.loop()

    except openravepy.openrave_exception, e:
        print e

if __name__ == "__main__":
    main()