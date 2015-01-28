#!/usr/bin/env python
import rospy
import tf
import numpy as np
from sensor_msgs.msg import CameraInfo

class RosCamera():
    def __init__(self, camera_name, base_frame):
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
        print "Constructor"

    def info_callback(self, camera_info):
        print "Callback"
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

            print self.transform