import sys
import numpy as np
import OpenGL.GL as gl
import OpenGL.GLUT as glut
import vispy.gloo as gloo
import GLObject
import transforms as tf
from openravepy import *
import cv2
from matplotlib import pyplot as plt

colormap_r = np.linspace(0.1, 1, 100)
colormap_g = np.linspace(0.1, 1, 100)
colormap_b = np.linspace(0.1, 1, 100)

class OffscreenRenderer:
    def __init__(self, width, height, env):
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

        self.vertex_shaders["depth"] = """
                        uniform mat4 world;
                        uniform mat4 view;
                        uniform mat4 projection;
                        attribute vec3 position;
                        attribute vec3 color;
                        void main()
                        {
                            vec4 cs_position =  view * world * vec4(position, 1.0);
                            gl_Position = projection * cs_position;
                        }
                        """
        self.fragment_shaders["depth"] = """
 
                        void main()
                        {
                            float originalZ = gl_FragCoord.z / gl_FragCoord.w;
                            gl_FragColor = vec4(originalZ, originalZ, originalZ, 1.0);
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

    def get_image_cv(self, buffer_name):
        buf = self.framebuffers[buffer_name];
        with buf:
            data = gloo.wrappers.read_pixels(viewport=(0, 0, self.width, self.height),alpha=False)
            print data.shape
            plt.imshow(data[:, :, 0], plt.get_cmap("jet"));
            plt.show()


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
        self.get_image_cv("depth")

    def loop(self):
        glut.glutMainLoop()

    def onTimer(self, fps):
        glut.glutPostRedisplay()
        glut.glutTimerFunc(1000/60, lambda fp2 : self.onTimer(fp2),60)
        #self.set_view_projection_matrix(self.width, self.height, self.fx, self.fy, self.cx, self.cy, self.near, self.far, self.view)

    def initialize_buffers(self):
        self.framebuffers["depth"] = gloo.FrameBuffer(color=gloo.ColorBuffer((self.width, self.height)), resizeable=False, depth=gloo.DepthBuffer((self.width, self.height)))
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