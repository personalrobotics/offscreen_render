#!/usr/bin/env python
import rospy
import tf
import numpy as np
from sensor_msgs.msg import CameraInfo
import sensor_msgs
from sensor_msgs import *
import ctypes
import math
import struct
import time;
import sensor_msgs.point_cloud2 as pc2
from multiprocessing import Pool
from itertools import product
from sensor_msgs.msg import PointCloud2, PointField, ChannelFloat32
from geometry_msgs.msg import Point32

class RosCamera():
    def __init__(self, camera_name, base_frame, near, far):
        self.base_frame = base_frame
        self.listener = tf.TransformListener()
        self.camera_name = camera_name;
        self.info_subscriber = rospy.Subscriber(camera_name, CameraInfo, self.info_callback)
        self.frame = ""
        self.transform = np.identity(4)
        self.last_time = rospy.Time(0)
        self.has_info = False;
        self.has_transform = False;
        self.fx = 0;
        self.fy = 0;
        self.cx = 0;
        self.cy = 0;
        self.width = 0;
        self.height = 0;
        self.near = near;
        self.far = far;

    def info_callback(self, camera_info):
        self.last_info = camera_info
        self.frame = camera_info.header.frame_id
        self.last_time = camera_info.header.stamp
        self.has_info = True
        self.fx = camera_info.K[0]
        self.fy = camera_info.K[4]
        self.cx = camera_info.K[2]
        self.cy = camera_info.K[5]
        self.width = camera_info.width
        self.height = camera_info.height

        self.lookup_transform()



    def create_cloud_xyz32(self, header, pts):
        p_list = [None] * pts.shape[1] 
        chlist = [0.] * pts.shape[1]
        i = 0
        for p in pts.T: 
            p_list[i] = Point32(p[0],p[1],p[2])
            i = i + 1
        ch = ChannelFloat32('t',chlist) 
        pc = PointCloud(None,p_list,[ch])
        pc.header.stamp = rospy.Time.now();
        pc.header.frame_id = self.frame;
        print "Created cloud"
        print pc.header
        return pc; 

    def lookup_transform(self):
        if(not self.has_info):
            return
        else:
            try:
                (trans, rot) = self.listener.lookupTransform(self.base_frame, self.frame, self.last_time)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                return

            self.transform =  tf.transformations.quaternion_matrix(rot);
            self.transform[0:3, 3] = trans
            self.has_transform = True

    def create_synth_pointcloud(self, depth_image):
        shape = depth_image.shape;
        rows = shape[0];
        cols = shape[1];
        start = time.clock();
        pc = pc2.create_cloud_xyz32(None, np.reshape(depth_image, (rows * cols, 3)))
        pc.header.stamp = rospy.Time.now();
        pc.header.frame_id = self.frame;
        print "Convert", time.clock() - start;
        return pc;
