import sys
import numpy as np
import OpenGL.GL as gl
import OpenGL.GLUT as glut
import vispy.gloo as gloo
import GLObject
import transforms as tf
import rospy
from openravepy import *
import sensor_msgs
from sensor_msgs import *
import cv_bridge

colormap_r = np.linspace(0.1, 1, 100)
colormap_g = np.linspace(0.1, 1, 100)
colormap_b = np.linspace(0.1, 1, 100)

class OffscreenRenderer:
    def __init__(self, width, height, env, camera):
        self.camera = camera;
        self.env = env
        self.view = np.identity(4, dtype=np.float32)
        self.projection = np.identity(4, dtype=np.float32)
        self.vertex_shaders = dict()
        self.fragment_shaders = dict()
        self.framebuffers = dict()
        self.programs = dict()
        self.objects = []
        self.width = width
        self.height = height
        self.fx = 0;
        self.fy = 0;
        self.cx = 0;
        self.cy = 0;
        self.near = 0;
        self.far = 0;
        self.bridge = cv_bridge.CvBridge()
        self.cloud_publisher = rospy.Publisher('synth_point_cloud', sensor_msgs.msg.PointCloud2)
        self.depth_publisher = rospy.Publisher('synth_depth', sensor_msgs.msg.Image)
        self.vertex_shaders["depth"] = """
                        uniform mat4 world;
                        uniform mat4 view;
                        uniform mat4 projection;
                        attribute vec3 position;
                        attribute vec3 color;
                        varying vec4 cs_position;
                        void main()
                        {
                            cs_position =  view * world * vec4(position, 1.0);
                            gl_Position = projection * cs_position;
                        }
                        """
        self.fragment_shaders["depth"] = """
                        varying vec4 cs_position;
                        void main()
                        {
                            gl_FragColor = vec4(cs_position.x, -cs_position.y, -cs_position.z, 1.0);
                        }
                        """
        self.vertex_shaders["color"] = """
                        uniform mat4 world;
                        uniform mat4 view;
                        uniform mat4 projection;
                        varying vec3 color_out;
                        attribute vec3 position;
                        attribute vec3 color;
                        void main()
                        {
                            gl_Position =  projection  * view * world  * vec4(position, 1.0);
                            color_out = color;
                        }
                        """

        self.fragment_shaders["color"] = """
                        varying vec3 color_out;

                        void main()
                        {
                            gl_FragColor = vec4(color_out.x, color_out.y, color_out.z, 1);
                        }
                       """
        self.initialize_buffers()

    def get_depth_image(self):
        return self.get_image('depth');

    def get_color_image(self):
        return self.get_image('color');

    def read_pixels(self, viewport=None, alpha=True, out_type='unsigned_byte'):
        """Read pixels from the currently selected buffer. 
        
        Under most circumstances, this function reads from the front buffer.
        Unlike all other functions in vispy.gloo, this function directly executes
        an OpenGL command.
        Parameters
        ----------
        viewport : array-like | None
            4-element list of x, y, w, h parameters. If None (default),
            the current GL viewport will be queried and used.
        alpha : bool
            If True (default), the returned array has 4 elements (RGBA).
            If False, it has 3 (RGB).
        out_type : str | dtype
            Can be 'unsigned_byte' or 'float'. Note that this does not
            use casting, but instead determines how values are read from
            the current buffer. Can also be numpy dtypes ``np.uint8``,
            ``np.ubyte``, or ``np.float32``.
        Returns
        -------
        pixels : array
            3D array of pixels in np.uint8 or np.float32 format. 
            The array shape is (h, w, 3) or (h, w, 4), with the top-left corner 
            of the framebuffer at index [0, 0] in the returned array.
        """

        
        type_dict = {'unsigned_byte': gl.GL_UNSIGNED_BYTE,
                     np.uint8: gl.GL_UNSIGNED_BYTE,
                     'float': gl.GL_FLOAT,
                     np.float32: gl.GL_FLOAT}
        type_ = gloo.wrappers._check_conversion(out_type, type_dict)
        if viewport is None:
            viewport = gl.glGetParameter(gl.GL_VIEWPORT)
        viewport = np.array(viewport, int)
        if viewport.ndim != 1 or viewport.size != 4:
            raise ValueError('viewport should be 1D 4-element array-like, not %s'
                             % (viewport,))
        x, y, w, h = viewport
        gl.glPixelStorei(gl.GL_PACK_ALIGNMENT, 1)  # PACK, not UNPACK
        fmt = gl.GL_RGBA if alpha else gl.GL_RGB
        im = gl.glReadPixels(x, y, w, h, fmt, type_)
        gl.glPixelStorei(gl.GL_PACK_ALIGNMENT, 4)
        # reshape, flip, and return
        if not isinstance(im, np.ndarray):
            np_dtype = np.uint8 if type_ == gl.GL_UNSIGNED_BYTE else np.float32
            im = np.frombuffer(im, np_dtype)

        im.shape = h, w, (4 if alpha else 3)  # RGBA vs RGB
        im = im[::-1, :, :]  # flip the image
        return im

    def get_image(self, buffer_name):
        buf = self.framebuffers[buffer_name];
        with buf:
            data = self.read_pixels(viewport=[0, 0, self.width, self.height], alpha=False, out_type=np.float32)
            return data;


    def set_view_projection_matrix(self, width, height, fx, fy, cx, cy, near, far, camera_transform):
        self.width = width;
        self.height = height;
        self.fx = fx;
        self.fy = fy;
        self.cx = cx;
        self.cy = cy;
        self.near = near;
        self.far = far;
        v = camera_transform;
        v[0:3, 0:3] = np.transpose(v[0:3, 0:3])
        v[0:3, 0:3] = np.dot(np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]]), v[0:3, 0:3])
        self.view = v;
        self.projection = self.get_projection_matrix(width, height, fx, fy, cx, cy, near, far)

    def get_projection_matrix(self, width, height, fx, fy, cx, cy, near, far):
        return tf.perspective_camera_calib(fx, fy, cx, cy, near, far, width, height);

    def add_kinbody(self, body):
        for link in body.GetLinks():
            for geometry in link.GetGeometries():
                c = np.array([colormap_r[len(self.objects) % 100], colormap_g[len(self.objects) * 3 % 100], colormap_b[len(self.objects) * 7 % 100]])
                self.objects.append(GLObject.GLObject(body, link.GetTransform(), geometry, c));

    def clear_bodies(self):
        self.objects = []

    def reshape(self, x, y):
        pass;

    def keyboard(self, key, x, y):
        pass;

    def display(self):
        for name in self.programs:
            with self.framebuffers[name]:
                gl.glViewport(0, 0, self.width, self.height)
                gl.glEnable(gl.GL_DEPTH_TEST)
                gloo.set_clear_color((0.0, 0.0, 0.0, 1))
                gloo.clear(color=True, depth=True)
                self.programs[name]['view'] = np.transpose(self.view)
                self.programs[name]['projection'] = np.transpose(self.projection)
                for body in self.objects:
                    self.programs[name].bind(body.vertex_buffer)
                    self.programs[name]['world'] = np.transpose(np.dot(body.body.GetTransform(), body.world))
                    self.programs[name].draw(gl.GL_TRIANGLES, body.index_buffer)
        glut.glutSwapBuffers()
        #glut.glutLeaveMainLoop()
        self.cloud_publisher.publish(self.camera.create_synth_pointcloud(self.get_image("depth")))
        #img = self.bridge.cv2_to_imgmsg(self.get_image("depth"), "bgr8");
        #img.header.frame_id = self.camera.frame;
        #img.header.stamp = rospy.Time.now();
        #self.depth_publisher.publish(img)

    def loop(self):
        glut.glutMainLoop()

    def onTimer(self, fps):
        glut.glutPostRedisplay()
        glut.glutTimerFunc(1000/60, lambda fp2 : self.onTimer(fp2), 60)
        #self.set_view_projection_matrix(self.width, self.height, self.fx, self.fy, self.cx, self.cy, self.near, self.far, self.view)

    def initialize_buffers(self):
        self.framebuffers["depth"] = gloo.FrameBuffer(color=gloo.ColorBuffer((self.width, self.height), format=gl.GL_RGB32F), resizeable=False, depth=gloo.DepthBuffer((self.width, self.height)))
        self.framebuffers["color"] = gloo.FrameBuffer(color=gloo.ColorBuffer((self.width, self.height)), resizeable=False, depth=gloo.DepthBuffer((self.width, self.height)))
        self.programs["depth"] = gloo.Program(self.vertex_shaders["depth"], self.fragment_shaders["depth"])
        self.programs["color"] = gloo.Program(self.vertex_shaders["color"], self.fragment_shaders["color"])

    def initialize_context(self):
        glut.glutInit(sys.argv)
        glut.glutInitDisplayMode(glut.GLUT_DOUBLE | glut.GLUT_RGBA | glut.GLUT_DEPTH)
        glut.glutCreateWindow('Offscreen Window')
        glut.glutReshapeWindow(self.width,self.height)
        glut.glutPositionWindow(100, 100)
        glut.glutReshapeFunc(lambda  x, y : self.reshape(x, y))
        glut.glutKeyboardFunc(lambda key, x, y: self.keyboard(x, y))
        glut.glutDisplayFunc(lambda : self.display())
        glut.glutTimerFunc(1000/60, lambda fps : self.onTimer(fps),60)
        gl.glEnable(gl.GL_DEPTH_TEST)


    def render_frame(self):
        self.initialize_context()
        glut.glutMainLoop()