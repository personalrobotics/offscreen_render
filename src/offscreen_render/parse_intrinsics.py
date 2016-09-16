from cv2 import cv
import numpy as np
import yaml

def getIntrinsics(filepath):
	camera = np.array( cv.Load(filepath, cv.CreateMemStorage(), "cameraMatrix") )
	fx = camera[0][0]
	fy = camera[1][1]
	cx = camera[0][2]
	cy = camera[1][2]
	print "fx", fx
	return fx, fy, cx, cy
filepath = '/home/pengjuj/ros/local/catkin_ws/src/herb_launch/calibration/kinect2/502845441942/calib_color.yaml'
getIntrinsics(filepath)	
