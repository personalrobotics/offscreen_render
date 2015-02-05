#include <offscreen_render/ROSCamera.h>
#include <offscreen_render/Conversions.h>
namespace offscreen_render
{

    ROSCamera::ROSCamera(const ros::NodeHandle& node, const std::string& camera,
            const std::string& base, float n, float f)
    {
        nodeHandle = node;
        cameraName = camera;
        baseFrame = base;
        near = n;
        far = f;
        cameraSubscriber = nodeHandle.subscribe(camera, 1, &ROSCamera::CameraInfoCallback, this);
        frame = "";
        hasInfo = false;
        hasTransform = false;
        fx = 0;
        fy = 0;
        cx = 0;
        cy = 0;
        width = 0;
        height = 0;
    }

    ROSCamera::~ROSCamera()
    {
    }


    void ROSCamera::LookupTransform()
    {
        if(!hasInfo) return;
        tf::StampedTransform stampedTransform;

        try
        {
            listener.lookupTransform(baseFrame, frame, lastTime, stampedTransform);
        }
        catch(tf::LookupException& ex)
        {
            return;
        }
        catch(tf::ExtrapolationException& ex)
        {
            return;
        }

        transform = TFToTransform(stampedTransform);
        hasTransform = true;
    }

    void ROSCamera::CameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& info)
    {
        lastInfo = info;
        frame = info->header.frame_id;
        lastTime = info->header.stamp;
        hasInfo = true;
        fx = info->K[0];
        fy = info->K[4];
        cx = info->K[2];
        cy = info->K[5];
        width = info->width;
        height = info->height;
        LookupTransform();
    }
}
