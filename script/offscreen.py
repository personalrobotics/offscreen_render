import offscreen_render
import herbpy
import os 
import numpy
import math
import openravepy
import sys
import prpy
from prpy.perception.apriltags import ApriltagsModule
from prpy.util import FindCatkinResource


env, robot = herbpy.initialize(sim=True, attach_viewer='rviz')

ap_detect = ApriltagsModule(marker_topic='/apriltags_kinect2/marker_array',
                              marker_data_path=FindCatkinResource('pr_ordata',
                                                                  'data/objects/tag_data.json'),
                              kinbody_path=FindCatkinResource('pr_ordata',
                                                              'data/objects'),
                              destination_frame='/herb_base',
                              detection_frame='/head/kinect2_rgb_optical_frame',
                              reference_link=robot.GetLink('/herb_base'))
ap_detect.DetectObjects(robot)
added_kinbodies, updated_kinbodies = ap_detect.Update()
# the viewer stays open after the example is done
import IPython; IPython.embed()
