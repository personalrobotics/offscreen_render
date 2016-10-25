from cv2 import cv
import numpy as np
import yaml

def getIntrinsics(filepath):
    """
    Given a file path to camera intrinsics, parse
    out the parameters.

    @param filepath Path to read
    @return fx, fy, cx, cy
    """
    camera = np.array( cv.Load(filepath, cv.CreateMemStorage(), "cameraMatrix") )
    fx = camera[0][0]
    fy = camera[1][1]
    cx = camera[0][2]
    cy = camera[1][2]
    return fx, fy, cx, cy
