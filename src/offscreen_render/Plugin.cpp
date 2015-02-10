#include <openrave/openrave.h>
#include <openrave/plugin.h>
#include <offscreen_render/RaveObjectTracker.h>

using namespace offscreen_render;
using namespace OpenRAVE;
using namespace std;

class ObjectTrackerPlugin : public ModuleBase
{
    public:
        ObjectTrackerPlugin(EnvironmentBasePtr env, ros::NodeHandle handle) : ModuleBase(env)
        {
            nodeHandle = handle;
            tracker.reset(new RaveObjectTracker(nodeHandle));
            __description = "A plugin for tracking objects with depth cameras.";
            RegisterCommand("Initialize", boost::bind(&ObjectTrackerPlugin::Initialize, this, _1, _2), "Initialize <depth_camera_info> <point_cloud_topic>. Initialize tracking system.");
            RegisterCommand("Track", boost::bind(&ObjectTrackerPlugin::Track, this, _1, _2), "Track <object_name> <iterations>. Update the pose of an object given the point cloud.");
        }

        virtual ~ObjectTrackerPlugin()
        {

        }
        bool Track(ostream& sout, istream& sinput)
        {
            std::string objectName;
            int numIters;
            sinput >> objectName;
            sinput >> numIters;
            tracker->TrackObject(GetEnv()->GetKinBody(objectName), numIters);
            return true;
        }


        bool Initialize(ostream& sout, istream& sinput)
        {
            std::string depthCameraInfo;
            std::string pointCloudTopic;
            sinput >> depthCameraInfo;
            sinput >> pointCloudTopic;
            return tracker->Initialize(depthCameraInfo, pointCloudTopic);
        }


        void Destroy()
        {
            RAVELOG_INFO("Unloaded object tracker.\n");
        }

        int main(const std::string& cmd)
        {
            return 0;
        }

        ros::NodeHandle nodeHandle;
        std::shared_ptr<RaveObjectTracker> tracker;
};


static char* argv[1] = {const_cast<char *>("object_tracker")};
static int argc = 1;
InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if( type == PT_Module && interfacename == "object_tracker" )
    {
        if (!ros::isInitialized())
        {
          ros::init(argc, argv, "object_tracker");
        }
        else
        {
          RAVELOG_DEBUG("Using existing ROS node '%s'\n", ros::this_node::getName().c_str());
        }
        ros::NodeHandle nh("~");
        return InterfaceBasePtr(new ObjectTrackerPlugin(penv, nh));
    }
    return InterfaceBasePtr();
}

void GetPluginAttributesValidated(PLUGININFO& info)
{
    info.interfacenames[PT_Module].push_back("object_tracker");
}

OPENRAVE_PLUGIN_API void DestroyPlugin()
{
    RAVELOG_INFO("destroying plugin\n");
}
