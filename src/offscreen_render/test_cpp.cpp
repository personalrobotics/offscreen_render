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
