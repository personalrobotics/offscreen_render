import sys
import numpy as np
import OpenGL.GL as gl
import OpenGL.GLUT as glut
import vispy.gloo as gloo
import GLObject
import transforms as tf
from openravepy import *

class OffscreenRenderer:
    def __init__(self, width, height, env):
        self.env = env
        self.view = np.identity(4, dtype=np.float32)
        self.projection = np.identity(4, dtype=np.float32)
        self.vertex_shaders = dict()
        self.fragment_shaders = dict()
        self.framebuffers = dict()
        self.renderbuffers = dict()
        self.programs = dict()
        self.objects = []
        self.width = width
        self.height = height

        self.vertex_shaders["depth"] = """
                        uniform mat4 world;
                        uniform mat4 view;
                        uniform mat4 projection;
                        attribute vec3 position;
                        attribute vec3 color;
                        varying float distToCamera;
                        void main()
                        {
                            vec4 cs_position =  view * world * vec4(position, 1.0);
                            distToCamera = -cs_position.z;
                            gl_Position = projection * cs_position;
                        }
                        """
        self.fragment_shaders["depth"] = """
                        varying float distToCamera;

                        void main()
                        {
                            gl_FragColor = vec4(distToCamera, distToCamera, distToCamera, 1.0);
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

    def set_view_projection_matrix(self, width, height, fx, fy, cx, cy, near, far, camera_transform):
        self.view = camera_transform
        self.projection = self.get_projection_matrix(width, height, fx, fy, cx, cy, near, far)

    def get_projection_matrix(self, width, height, fx, fy, cx, cy, near, far):
        return tf.perspective_camera_calib(fx, fy, cx, cy, near, far, width, height);

    def add_kinbody(self, body):
        for link in body.GetLinks():
            for geometry in link.GetGeometries():
                self.objects.append(GLObject.GLObject(body, link.GetTransform(), geometry, geometry.GetDiffuseColor()));

    def clear_bodies(self):
        self.objects = []

    def reshape(self, x, y):
        pass;

    def keyboard(self, key, x, y):
        pass;

    def display(self):
        gl.glViewport(0, 0, self.width, self.height)
        for name in self.programs:
            #with self.framebuffers[name]:
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

    def loop(self):
        glut.glutMainLoop()

    def onTimer(self, fps):
        glut.glutPostRedisplay()
        glut.glutTimerFunc(1000/60, lambda fp2 : self.onTimer(fp2),60)
        camera_transform = self.env.GetViewer().GetCameraTransform()

        self.set_view_projection_matrix(640., 480., 320., 240., 320., 240., 0.01, 4.,camera_transform)

    def initialize_buffers(self):
        self.renderbuffers["depth"] = gloo.ColorBuffer(shape=(self.width, self.height), format='color', resizeable=False)
        self.renderbuffers["color"] = gloo.ColorBuffer(shape=(self.width, self.height), format='color', resizeable=False)
        self.framebuffers["depth"] = gloo.FrameBuffer(color=self.renderbuffers["depth"], resizeable=False)
        self.framebuffers["color"] = gloo.FrameBuffer(color=self.renderbuffers["color"], resizeable=False)
        self.programs["depth"] = gloo.Program(self.vertex_shaders["depth"], self.fragment_shaders["depth"])
        #self.programs["color"] = gloo.Program(self.vertex_shaders["color"], self.fragment_shaders["color"])

    def initialize_context(self):
        glut.glutInit(sys.argv)
        glut.glutInitDisplayMode(glut.GLUT_DOUBLE | glut.GLUT_RGBA | glut.GLUT_DEPTH)
        glut.glutCreateWindow('Offscreen Window')
        glut.glutReshapeWindow(512,512)
        glut.glutPositionWindow(100, 100)
        glut.glutReshapeFunc(lambda  x, y : self.reshape(x, y))
        glut.glutKeyboardFunc(lambda key, x, y: self.keyboard(x, y))
        glut.glutDisplayFunc(lambda : self.display())
        glut.glutTimerFunc(1000/60, lambda fps : self.onTimer(fps),60)
        gl.glEnable(gl.GL_DEPTH_TEST)


    def render_frame(self):
        self.initialize_context()
        glut.glutMainLoop()