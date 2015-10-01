import openravepy;
import rospy;
import sensor_msgs;
from threading import Timer, Lock
import random;
import time;
import tf;
import numpy as np;


def render_frame_thread(camera_sim):
    info = camera_sim.camera_info;
    camera_sim.camera = openravepy.RaveCreateSensor(camera_sim.env, 'rave_to_ros_camera');
    camera_sim.send_command('setintrinsic ' + str(info.K[0]) + ' ' + str(info.K[4]) + ' ' + str(info.K[2]) + ' ' + str(info.K[5]) + ' ' + str(camera_sim.near) + ' ' + str(camera_sim.far));
    camera_sim.send_command('setdims ' + str(info.width) + ' ' + str(info.height));
    camera_sim.send_command('initialize ~ ' + camera_sim.topic + '/sim_depth' + ' ' + camera_sim.topic + '/sim_color ' + camera_sim.topic + '/sim_points ' + info.header.frame_id + '_sim ' + camera_sim.fixed_frame);
    camera_sim.camera.Configure(openravepy.Sensor.ConfigureCommand.PowerOn);

    while (True):
        time.sleep(0.016);
        with camera_sim.lock:
            camera_sim.camera.SimulationStep(0.01);

            for add in camera_sim.additions:
                camera_sim.send_command('addbody ' + add[0] + ' ' + str(add[1]) + ' ' + str(add[2]) + ' ' + str(add[3]));

            camera_sim.additions = [];

            for rem in camera_sim.removals:
                camera_sim.send_command('removebody ' + rem[0]);

            camera_sim.removals = [];

    pass;

class RosCameraSim:
    """"Simulates a ROS camera in OpenRAVE"""

    def __init__(self, env, fixed_frame='/map', near=0.01, far=10.0):
        self.env = env;
        self.near = near;
        self.far = far;
        self.fixed_frame = fixed_frame;
        self.camera = None;
        self.thread = None;
        self.camera_info = None;
        self.topic = None;
        self.info_subscriber = None;
        self.listener = None;
        self.additions = []
        self.removals = []

    def add_body(self, body, r=-1, g=-1, b=-1):
        with self.lock:
            if (self.camera is None):
                print "Error:no camera created yet. Can't add body";
                return;

            if (r is -1):
                r = random.randint(0, 255);
                g = random.randint(0, 255);
                b = random.randint(0, 255);
            self.additions += [(body.GetName(), r, g, b)];

    def remove_body(self, body):
        with self.lock:
            if (self.camera is None):
                print "Error:no camera created yet. Can't remove body";
                return;
            self.removals += [body.GetName()];

    def send_command(self, command):
        print command;
        self.camera.SendCommand(command);

    def info_callback(self, info):
        with self.lock:
            self.camera_info = info;

            if (self.running and not (self.camera is None)):
                try:
                    (trans, rot) = self.listener.lookupTransform(self.fixed_frame, self.camera_info.header.frame_id,
                                                                 self.camera_info.header.stamp);
                    m = openravepy.matrixFromQuat(np.array([rot[3], rot[0], rot[1], rot[2]]));
                    m[0, 3] = trans[0];
                    m[1, 3] = trans[1];
                    m[2, 3] = trans[2];
                    self.camera.SetTransform(m);
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                     #print "Failed to lookup " + self.camera_info.header.frame_id + " in " + self.fixed_frame;
                    pass;

    def start(self, camera_info_topic):
        self.topic = camera_info_topic;
        self.info_subscriber = rospy.Subscriber(camera_info_topic, sensor_msgs.msg.CameraInfo, self.info_callback);
        self.listener = tf.TransformListener();
        self.lock = Lock();
        print "Waiting for callback..."
        self.running = True;

        iters = 0;
        while (self.camera_info is None and iters < 5):
            iters += 1;
            time.sleep(1);

        if (self.camera_info is None):
            print "Failed to connect to camera " + camera_info_topic;
            return;
        info = self.camera_info;

        self.thread = Timer(0.1, render_frame_thread, [self]);
        self.thread.start();


    def stop(self):
        self.running = False;
        self.thread.cancel();
