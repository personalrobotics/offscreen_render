import openravepy;
import random;
import time;
from io import BytesIO
import numpy as np;
from threading import Timer, Lock

class SimCamera:
    """"Simulates a camera in OpenRAVE to do offscreen computations"""

    def __init__(self, env, transform=None,
                 fx=529, fy=525, cx=328, cy=267, near=0.01, far=10.0, 
                 width=640, height=480):
        """Initializes the camera with intrinsics fx, fy, cx, cy, with a near
           and far plane (in meters) with width and height in pixels"""
           
        self.env = env;
        self.lock = Lock()
        self.camera = openravepy.RaveCreateSensor(env, 'offscreen_render_camera')
        self.set_intrinsic(fx, fy, cx, cy, near, far, width, height)
        self.camera.Configure(openravepy.Sensor.ConfigureCommand.PowerOn);
        if (transform is None):
            self.set_transform(numpy.eye(4))
        else:
            self.set_transform(transform)
            
    def render(self):
        """Renders an image from the camera and returns as a numpy array"""
        self.camera.SimulationStep(0.01)
        return self.camera.GetSensorData().imagedata;

    def set_transform(self, tf):
        """Sets the position/orientation of the camera"""
        self.camera.SetTransform(tf)
    
    def add_body(self, body, r=-1, g=-1, b=-1):
        """Adds a kinbody with the given color in red, green, blue 0-255"""
        with self.lock:
            if (self.camera is None):
                print "Error:no camera created yet. Can't add body";
                return;

            if (r is -1):
                r = random.randint(0, 255);
                g = random.randint(0, 255);
                b = random.randint(0, 255);
            self.send_command('addbody ' + body.GetName() + ' ' + str(r) +
             ' ' + str(g) + ' ' + str(b))

    def get_mesh_ids(self, body):
        """Gets the mesh link ids associated with the given body"""
        with self.lock:
            return self.send_command('get_kinbody_link_mesh_ids ' + body.GetName())

    def get_num_link_geometries(self, body, link_idx=0):
        """Gets the number of link geometries associated with a given kinbody 
           and link"""
        with self.lock:
            return self.send_command('get_num_link_geometries ' + body.GetName()
             + ' ' + str(link_idx))
             
    def get_link_mesh_positions(self, body, link_idx=0, geometry_idx=0):
        with self.lock:
            """Gets the (x, y, z flat array) positions associated with the given kinbody,
               link, and geometry"""
            response = self.send_command('get_link_mesh_positions ' + body.GetName() + ' ' + str(link_idx) + ' ' + str(geometry_idx));
            filehandle = BytesIO(response)
            return np.loadtxt(filehandle)

    def get_link_mesh_colors(self, body, link_idx=0, geometry_idx=0):
        with self.lock:
            """Gets the mesh colors (r, g, b) flat array associated with the given kinbody,
                link, and geometry"""
            response = self.send_command('get_link_mesh_colors ' + body.GetName() + ' ' + str(link_idx) + ' ' + str(geometry_idx));
            filehandle = BytesIO(response)
            return np.loadtxt(filehandle)
            
    def get_link_mesh_indices(self, body, link_idx=0, geometry_idx=0):
        with self.lock:
            """Gets the triangle indices (short array) associated with the given kinbody,
            link and geometry"""
            response = self.send_command('get_link_mesh_indices ' + body.GetName() + ' ' + str(link_idx) + ' ' + str(geometry_idx));
            filehandle = BytesIO(response)
            return np.loadtxt(filehandle)    
                 
    def set_link_mesh_colors(self, body, colors, link_idx=0, geometry_idx=0):
        with self.lock:
            """Given a kinbody, its link index, and geometry index, sets the colors
               of that mesh (a flat array of (r, g, b) values)"""
            colors_string = ' '.join(['%.5f' % num for num in colors])
            return self.send_command('set_link_mesh_colors '+ body.GetName() + ' ' + str(link_idx) + ' ' + str(geometry_idx) + ' ' + colors_string);        
    
    def set_link_mesh_positions(self, body, colors, link_idx=0, geometry_idx=0):
        with self.lock:
             """Given a kinbody, its link index, and geometry index, sets the positions
                of its vertices (a flat array of (x, y, z) values)"""
            pos_string = ' '.join(['%.5f' % num for num in colors])
            return self.send_command('set_link_mesh_positions '+ body.GetName() + ' ' + str(link_idx) + ' ' + str(geometry_idx) + ' ' + pos_string);
    
    def set_link_mesh_indices(self, body, colors, link_idx=0, geometry_idx=0):
        with self.lock:
            """Given a kinbody, its link index and geometry index, sets the triangle
               index values (a flat array of 16-bit unsigned ints)"""
            idx_string = ' '.join(['%.5f' % num for num in colors])
            return self.send_command('set_link_mesh_indices '+ body.GetName() + ' ' + str(link_idx) + ' ' + str(geometry_idx) + ' ' + idx_string);
    
    def remove_body(self, body):
        """Removes the given kinbody from the renderer"""
        with self.lock:
            if (self.camera is None):
                print "Error:no camera created yet. Can't remove body";
                return;
            self.send_command('removebody ' + body.GetName())
            
    def clear_bodies(self):
        """Clears all the bodies from the renderer"""
        self.send_command('clearbodies')
   
    def set_intrinsic(self, fx, fy, cx, cy, near, far, width, height):
        """Sets the camera intrinsic values"""
        self.send_command('setintrinsic ' + 
            str(fx) + ' ' + str(fy) + ' ' + str(cx) + 
            ' ' + str(cy) + ' ' + str(near) + ' ' + str(far));
        self.send_command('setdims ' + 
        str(width) + ' ' + str(height));

    def send_command(self, command):
        #print command;
        return self.camera.SendCommand(command);

