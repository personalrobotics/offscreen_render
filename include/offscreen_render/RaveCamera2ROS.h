/*
 * RaveCamera2ROS.h
 *
 *  Created on: Apr 22, 2015
 *      Author: mklingen
 */

#ifndef RAVECAMERA2ROS_H_
#define RAVECAMERA2ROS_H_

#include <ros/ros.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/camera_publisher.h>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <offscreen_render/CloudGenerator.h>

#include <openrave/openrave.h>
#include <offscreen_render/RaveCamera.h>

namespace offscreen_render
{
    class RaveCamera2ROS : public RaveCamera
    {
        public:
            RaveCamera2ROS(OpenRAVE::EnvironmentBasePtr env);
            virtual ~RaveCamera2ROS();

            bool Initialize(const ros::NodeHandle& node,
                    const std::string& depthTopic,
                    const std::string& colorTopic,
                    const std::string& cloudTopic,
                    const std::string& tfFrame,
                    const std::string& baseFrame);

            virtual bool SimulationStep(OpenRAVE::dReal fTimeElapsed);

            // Interface Commands
            bool _Initialize(std::ostream& out, std::istream& in);

            void SendTransforms();
            void PublishImages();
            void PublishCloud();

            void DepthBuffer2ROSImg(const FrameBuffer<float, 3>& xyz, sensor_msgs::Image* img);
            void ColorBuffer2ROSImg(const FrameBuffer<float, 3>& rgb, sensor_msgs::Image* img);
            sensor_msgs::CameraInfo GetDepthCameraInfo();
            sensor_msgs::CameraInfo GetColorCameraInfo();

        protected:
            ros::NodeHandle nodeHandle;
            CloudGenerator cloudGenerator;
            std::string tfFrame;
            std::string baseFrame;
            image_transport::CameraPublisher depthCameraPub;
            image_transport::CameraPublisher colorCameraPub;
            sensor_msgs::ImagePtr lastDepthImage;
            sensor_msgs::ImagePtr lastColorImage;
            tf::TransformBroadcaster tfBroadcaster;
            PointCloud::Ptr synthCloud;

    };
}

#endif /* RAVECAMERA2ROS_H_ */
