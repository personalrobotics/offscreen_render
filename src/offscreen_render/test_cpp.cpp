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


#include <GLFW/glfw3.h>
#include <stdlib.h>
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
    while (!glfwWindowShouldClose(window))
    {
        float ratio;
        int width, height;
        glfwGetFramebufferSize(window, &width, &height);
        ratio = width / (float) height;
        glViewport(0, 0, width, height);
        glClearColor(0, 1.0, 0, 1.0);
        glClear(GL_COLOR_BUFFER_BIT);
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        glOrtho(-ratio, ratio, -1.f, 1.f, 1.f, -1.f);
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
        glfwSwapBuffers(window);
        glfwPollEvents();
    }
    glfwDestroyWindow(window);
    glfwTerminate();
    exit(EXIT_SUCCESS);
}
