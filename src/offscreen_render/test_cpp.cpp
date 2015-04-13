/*
#include <offscreen_render/RaveObjectTracker.h>
#include <openrave-core.h>
#include <thread>

using namespace offscreen_render;
OpenRAVE::EnvironmentBasePtr environment;

void ViewerThread()
{
    OpenRAVE::ViewerBasePtr viewer = OpenRAVE::RaveCreateViewer(environment, "rviz");
    environment->Add(viewer);
    bool showgui = true;
    viewer->main(showgui);
}


int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "offscreen_renderer");
    ros::NodeHandle node("~");

    // Initialize OpenRAVE
    OpenRAVE::RaveInitialize(true);
    environment = OpenRAVE::RaveCreateEnvironment();
    std::thread viewThread(ViewerThread);
    environment->Load("/homes/mklingen/prdev_catkin/src/pr-ordata/data/objects/pop_tarts.kinbody.xml");

    OpenRAVE::KinBodyPtr fuze = environment->GetKinBody("pop_tarts");
    OpenRAVE::Transform fuzeTF = fuze->GetTransform();
    fuzeTF.trans = OpenRAVE::Vector(0.75, -0.15, -0.1);
    fuze->SetTransform(fuzeTF);

    for(int i = 0; i < 1000; i++)
    {
        usleep(1000);
    }

    RaveObjectTracker tracker(node);
    tracker.Initialize("/camera/depth/camera_info", "/camera/depth_registered/points");
    tracker.TrackObject(fuze, 1000);
    RAVELOG_INFO("Done tracking object.");
    viewThread.join();
    return 0;
}
*/

#include <offscreen_render/Geometry.h>
#include <offscreen_render/Conversions.h>
#include <offscreen_render/OffscreenRenderer.h>
#include <GLFW/glfw3.h>
#include <stdlib.h>
#include <ros/package.h>
#include <stdio.h>

static void error_callback(int error, const char* description)
{
    fputs(description, stderr);
}
static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window, GL_TRUE);
}


void DrawSceneFixed(float aspect)
{
    glClearColor(0, 1.0, 0, 1.0);
    glClear(GL_COLOR_BUFFER_BIT);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(-aspect, aspect, -1.f, 1.f, 1.f, -1.f);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glRotatef((float) glfwGetTime() * 50.f, 0.f, 0.f, 1.f);
    glBegin(GL_TRIANGLES);
    glColor3f(1.f, 0.f, 0.f);
    glVertex3f(-0.6f, -0.4f, 0.f);
    glColor3f(0.f, 1.f, 0.f);
    glVertex3f(0.6f, -0.4f, 0.f);
    glColor3f(0.f, 0.f, 1.f);
    glVertex3f(0.f, 0.6f, 0.f);
    glEnd();
}

using namespace offscreen_render;
int main(void)
{
    GLFWwindow* window;
    glfwSetErrorCallback(error_callback);
    if (!glfwInit())
        exit(EXIT_FAILURE);
    window = glfwCreateWindow(640, 480, "Simple example", NULL, NULL);
    if (!window)
    {
        glfwTerminate();
        exit(EXIT_FAILURE);
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    glfwSetKeyCallback(window, key_callback);
    float ratio;
    int width, height;
    glfwGetFramebufferSize(window, &width, &height);
    ratio = width / (float) height;
    GLenum err = glewInit();
    if (err != GLEW_OK)
    {
        RAVELOG_ERROR("Failed to initialize GLEW.");
        return false;
    }
    if (!GLEW_VERSION_2_1)
    {
        RAVELOG_ERROR("Need GLEW 2.1 or higher.");
        return false;
    }

    if (!GLEW_ARB_framebuffer_object)
    {
        RAVELOG_ERROR("Glew does not support framebuffer\n");
        return false;
    }

    OffscreenRenderer renderer;
    Shader colorShader;
    Shader depthShader;

    std::string shaders = ros::package::getPath("offscreen_render") + "/shaders/";

    if(!depthShader.LoadFromFile(shaders + "FragmentShader.glsl", shaders + "VertexShader.glsl"))
    {
      printf("Failed to load depth shaders.\n");
      return -1;
    }

    if(!colorShader.LoadFromFile(shaders + "ColorFragmentShader.glsl", shaders + "VertexShader.glsl"))
    {
        printf("Failed to load color shaders.\n");
        return -1;
    }

    renderer.depthShader = &depthShader;
    renderer.colorShader = &colorShader;
    renderer.Initialize(width, height);

    Model cube;
    cube.buffer = new VertexBuffer();
    static const GLfloat g_vertex_buffer_data[] = {
        -1.0f,-1.0f,-1.0f, // triangle 1 : begin
        -1.0f,-1.0f, 1.0f,
        -1.0f, 1.0f, 1.0f, // triangle 1 : end
        1.0f, 1.0f,-1.0f, // triangle 2 : begin
        -1.0f,-1.0f,-1.0f,
        -1.0f, 1.0f,-1.0f, // triangle 2 : end
        1.0f,-1.0f, 1.0f,
        -1.0f,-1.0f,-1.0f,
        1.0f,-1.0f,-1.0f,
        1.0f, 1.0f,-1.0f,
        1.0f,-1.0f,-1.0f,
        -1.0f,-1.0f,-1.0f,
        -1.0f,-1.0f,-1.0f,
        -1.0f, 1.0f, 1.0f,
        -1.0f, 1.0f,-1.0f,
        1.0f,-1.0f, 1.0f,
        -1.0f,-1.0f, 1.0f,
        -1.0f,-1.0f,-1.0f,
        -1.0f, 1.0f, 1.0f,
        -1.0f,-1.0f, 1.0f,
        1.0f,-1.0f, 1.0f,
        1.0f, 1.0f, 1.0f,
        1.0f,-1.0f,-1.0f,
        1.0f, 1.0f,-1.0f,
        1.0f,-1.0f,-1.0f,
        1.0f, 1.0f, 1.0f,
        1.0f,-1.0f, 1.0f,
        1.0f, 1.0f, 1.0f,
        1.0f, 1.0f,-1.0f,
        -1.0f, 1.0f,-1.0f,
        1.0f, 1.0f, 1.0f,
        -1.0f, 1.0f,-1.0f,
        -1.0f, 1.0f, 1.0f,
        1.0f, 1.0f, 1.0f,
        -1.0f, 1.0f, 1.0f,
        1.0f,-1.0f, 1.0f
    };

    // One color for each vertex. They were generated randomly.
    static const GLfloat g_color_buffer_data[] = {
        0.583f,  0.771f,  0.014f,
        0.609f,  0.115f,  0.436f,
        0.327f,  0.483f,  0.844f,
        0.822f,  0.569f,  0.201f,
        0.435f,  0.602f,  0.223f,
        0.310f,  0.747f,  0.185f,
        0.597f,  0.770f,  0.761f,
        0.559f,  0.436f,  0.730f,
        0.359f,  0.583f,  0.152f,
        0.483f,  0.596f,  0.789f,
        0.559f,  0.861f,  0.639f,
        0.195f,  0.548f,  0.859f,
        0.014f,  0.184f,  0.576f,
        0.771f,  0.328f,  0.970f,
        0.406f,  0.615f,  0.116f,
        0.676f,  0.977f,  0.133f,
        0.971f,  0.572f,  0.833f,
        0.140f,  0.616f,  0.489f,
        0.997f,  0.513f,  0.064f,
        0.945f,  0.719f,  0.592f,
        0.543f,  0.021f,  0.978f,
        0.279f,  0.317f,  0.505f,
        0.167f,  0.620f,  0.077f,
        0.347f,  0.857f,  0.137f,
        0.055f,  0.953f,  0.042f,
        0.714f,  0.505f,  0.345f,
        0.783f,  0.290f,  0.734f,
        0.722f,  0.645f,  0.174f,
        0.302f,  0.455f,  0.848f,
        0.225f,  0.587f,  0.040f,
        0.517f,  0.713f,  0.338f,
        0.053f,  0.959f,  0.120f,
        0.393f,  0.621f,  0.362f,
        0.673f,  0.211f,  0.457f,
        0.820f,  0.883f,  0.371f,
        0.982f,  0.099f,  0.879f
    };

    for (int i = 0 ; i < 12 * 3 *3; i++)
    {
        cube.buffer->position_data.push_back(g_vertex_buffer_data[i]);
        cube.buffer->color_data.push_back(g_color_buffer_data[i]);
        cube.buffer->index_data.push_back(i);
    }

    cube.buffer->Initialize(*renderer.colorShader);
    cube.transform = Mat4x4::Identity();
    renderer.models.push_back(cube);
    renderer.projectionMatrix = GetPerspectiveMatrix(525, 525, 325, 267, 0.01, 10.0f, width, height);
    //renderer.projectionMatrix = GetPerspectiveFOV(90.0f, 4.0f / 3.0f, 10.0f, 0.01f);
    Transform cameraTF;
    cameraTF = Transform::Identity();
    cameraTF.translation() = Vec3(0, 0, -5);
    renderer.viewMatrix = GetViewMatrix(cameraTF);
    float t = 0.0f;
    while (!glfwWindowShouldClose(window))
    {
        t += 0.01f;
        glViewport(0, 0, width, height);
        glClearColor(0, 0, 0, 1);
        glEnable(GL_DEPTH_TEST);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        renderer.Draw();
        Transform obj_tf = Transform::Identity();
        obj_tf.translation() = Vec3(0, 0, 5);
        obj_tf.linear() = Eigen::AngleAxisf(t, Vec3(0, 1, 0)).matrix();
        cube.transform = obj_tf.matrix();
        renderer.models.clear();
        renderer.models.push_back(cube);
        glfwSwapBuffers(window);
        glfwPollEvents();
    }
    glfwDestroyWindow(window);
    glfwTerminate();
    exit(EXIT_SUCCESS);
}
