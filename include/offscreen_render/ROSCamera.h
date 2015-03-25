#ifndef ROSCAMERA_H_
#define ROSCAMERA_H_

#include "Geometry.h"
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

namespace offscreen_render
{

    class ROSCamera
    {
        public:
            ROSCamera() :
                hasInfo(false),
                hasTransform(false),
                fx(0),
                fy(0),
                cx(0),
                cy(0),
                width(0),
                height(0),
                near(0),
                far(0)
            {

            }
            ROSCamera(const ros::NodeHandle& node, const std::string& camera,
                    const std::string& base, float near, float far);
            virtual ~ROSCamera();

            void CameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& info);

            void LookupTransform();
            bool LookupTransform(const std::string& frame, Transform& transformOut);
            ros::NodeHandle nodeHandle;
            std::string frame;
            std::string baseFrame;
            tf::TransformListener listener;
            std::string cameraName;
            ros::Subscriber cameraSubscriber;
            Transform transform;
            ros::Time lastTime;
            sensor_msgs::CameraInfoConstPtr lastInfo;
            bool hasInfo;
            bool hasTransform;
            float fx;
            float fy;
            float cx;
            float cy;
            float width;
            float height;
            float near;
            float far;
    };

}

#endif // ROSCAMERA_H_ 
