#include <openrave/config.h>
#include <openrave/openrave.h>
#include <openrave/plugin.h>
#include <boost/make_shared.hpp>

#include <offscreen_render/RaveCamera.h>
#include <offscreen_render/RaveCamera2ROS.h>

void GetPluginAttributesValidated(OpenRAVE::PLUGININFO &info)
{
    info.interfacenames[OpenRAVE::PT_Sensor].push_back("offscreen_render_camera");
    info.interfacenames[OpenRAVE::PT_Sensor].push_back("rave_to_ros_camera");
    return;
}

OpenRAVE::InterfaceBasePtr CreateInterfaceValidated(OpenRAVE::InterfaceType type, std::string const &interface_name, std::istream &sinput, OpenRAVE::EnvironmentBasePtr env)
{

    if (type == OpenRAVE::PT_Sensor && interface_name == "offscreen_render_camera")
    {
        return boost::make_shared<offscreen_render::RaveCamera>(env);
    }
    else if (type == OpenRAVE::PT_Sensor && interface_name == "rave_to_ros_camera")
    {
        return boost::make_shared<offscreen_render::RaveCamera2ROS>(env);
    }
    else
    {
        return OpenRAVE::InterfaceBasePtr();
    }
}

OPENRAVE_PLUGIN_API void DestroyPlugin()
{
    return;
}
