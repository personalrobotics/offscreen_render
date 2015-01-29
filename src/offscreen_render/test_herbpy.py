#!/usr/bin/env python
"""
Provides a simple console that sets up basic functionality for 
using herbpy and openravepy.
"""
import ros_camera_bridge
from ros_camera_bridge import RosCamera
import OffscreenRenderer;
import os
import rospy
import time
if os.environ.get('ROS_DISTRO', 'hydro')[0] <= 'f':
    import roslib
    roslib.load_manifest('herbpy')

import argparse, herbpy, logging, numpy, openravepy, sys

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='utility script for loading HerbPy')
    parser.add_argument('-s', '--sim', action='store_true',
                        help='simulation mode')
    parser.add_argument('-v', '--viewer', nargs='?', const=True,
                        help='attach a viewer of the specified type')
    parser.add_argument('--robot-xml', type=str,
                        help='robot XML file; defaults to herb_description')
    parser.add_argument('--env-xml', type=str,
                        help='environment XML file; defaults to an empty environment')
    parser.add_argument('-b', '--segway-sim', action='store_true',
                        help='simulate base')
    parser.add_argument('--debug', action='store_true',
                        help='enable debug logging')
    args = parser.parse_args()

    openravepy.misc.InitOpenRAVELogging()

    if args.debug:
        openravepy.RaveSetDebugLevel(openravepy.DebugLevel.Debug)


    herbpy_args = {'sim':args.sim,
                   'attach_viewer':args.viewer,
                   'robot_xml':args.robot_xml,
                   'env_path':args.env_xml,
                   'segway_sim':args.segway_sim}
    if args.sim and not args.segway_sim:
        herbpy_args['segway_sim'] = args.sim
    
    env, robot = herbpy.initialize(**herbpy_args)
    rospy.init_node('herbpy')
    time.sleep(1)
    camera = ros_camera_bridge.RosCamera('/camera/depth/camera_info', '/map');


    while not camera.has_transform:
        time.sleep(1)

    renderer = OffscreenRenderer.OffscreenRenderer(camera.width, camera.height, env);
    print "USING TRANSFORM\n" + str(camera.transform)
    renderer.set_view_projection_matrix(camera.width, camera.height, camera.fx, camera.fy, camera.cx, camera.cy, 0.01, 4., camera.transform);
    renderer.add_kinbody(robot);
    renderer.initialize_context()
    renderer.loop()