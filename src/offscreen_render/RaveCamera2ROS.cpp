/*
 * RaveCamera2ROS.cpp
 *
 *  Created on: Apr 22, 2015
 *      Author: mklingen
 */

#include <offscreen_render/RaveCamera2ROS.h>
#include <image_transport/image_transport.h>
#include <offscreen_render/Conversions.h>

namespace offscreen_render
{
    RaveCamera2ROS::RaveCamera2ROS(OpenRAVE::EnvironmentBasePtr env) :
            RaveCamera(env)
    {
        RegisterCommand("initialize", boost::bind(&RaveCamera2ROS::_Initialize, this, _1, _2), "Initialize the ROS camera (nodeName, depthTopic, colorTopic, cloudTopic, tfFrame, baseFrame).");
    }

    RaveCamera2ROS::~RaveCamera2ROS()
    {

    }

    bool RaveCamera2ROS::Initialize(const ros::NodeHandle& node,
            const std::string& depthTopic,
            const std::string& colorTopic,
            const std::string& cloudTopic,
            const std::string& tfFrame,
            const std::string& baseFrame)
    {
        nodeHandle = node;
        image_transport::ImageTransport it(nodeHandle);
        depthCameraPub = it.advertiseCamera(depthTopic, 1);
        colorCameraPub = it.advertiseCamera(colorTopic, 1);
        this->tfFrame = tfFrame;
        this->baseFrame = baseFrame;
        lastDepthImage.reset(new sensor_msgs::Image());
        lastColorImage.reset(new sensor_msgs::Image());
        synthCloud.reset(new PointCloud());
        cloudGenerator.Initialize(nodeHandle, cloudTopic);
        return RaveCamera::Initialize();
    }

    // Interface Commands
    bool RaveCamera2ROS::_Initialize(std::ostream& out, std::istream& in)
    {
        std::string nodeName, depthTopic, colorTopic, cloudTopic, tfFrame, baseFrame;
        in >> nodeName >> depthTopic >> colorTopic >> cloudTopic >> tfFrame >> baseFrame;
        return Initialize(ros::NodeHandle(nodeName), depthTopic, colorTopic, cloudTopic, tfFrame, baseFrame);
    }

    void RaveCamera2ROS::DepthBuffer2ROSImg(const FrameBuffer<float, 3>& xyz, sensor_msgs::Image* img)
    {
        img->data.resize(xyz.width * xyz.height * sizeof(float));
        std::vector<float> singleChannel;
        singleChannel.resize(xyz.width * xyz.height);

        for (int x = 0; x < xyz.width; x++)
        {
            for (int y = 0; y < xyz.height; y++)
            {
                int i1 = (x + (y) * xyz.width);
                int i2 = (x + (xyz.height - y - 1) * xyz.width);
                singleChannel[i1] = xyz.data[i2 * 3 + 2];
            }
        }

        uint8_t* byteData = reinterpret_cast<uint8_t*>(singleChannel.data());

        for (size_t i = 0; i < (xyz.width * xyz.height) * sizeof(float); i++)
        {
            img->data[i] = byteData[i];
        }

        img->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
        img->width = xyz.width;
        img->height = xyz.height;
        img->header.stamp = ros::Time::now();
        img->header.frame_id = tfFrame + "_optical_frame";
        img->step = xyz.width * sizeof(float);
    }

    void RaveCamera2ROS::ColorBuffer2ROSImg(const FrameBuffer<float, 3>& rgb, sensor_msgs::Image* img)
    {
        img->data.resize(rgb.width * rgb.height * 3);
        assert(rgb.data.size() == rgb.width * rgb.height * 3);
        for (int x = 0; x < rgb.width; x++)
        {
            for (int y = 0; y < rgb.height; y++)
            {
                int i1 = (x + (y) * rgb.width);
                int i2 = (x + (rgb.height - y - 1) * rgb.width);
                // b
                img->data[i1 * 3]     = static_cast<uint8_t>((255.0f) * rgb.data[i2 * 3 + 2]);
                // g
                img->data[i1 * 3 + 1] = static_cast<uint8_t>((255.0f) * rgb.data[i2 * 3 + 1]);
                // r
                img->data[i1 * 3 + 2] = static_cast<uint8_t>((255.0f) * rgb.data[i2 * 3]);
            }
        }
        img->width = rgb.width;
        img->height = rgb.height;
        img->encoding = sensor_msgs::image_encodings::BGR8;
        img->header.stamp = ros::Time::now();
        img->header.frame_id = tfFrame + "_optical_frame";
        img->step = rgb.width * 3 * sizeof(uint8_t);
    }

    sensor_msgs::CameraInfo RaveCamera2ROS::GetDepthCameraInfo()
    {
        sensor_msgs::CameraInfo toReturn;
        toReturn.width = renderer.depthBuffer.width;
        toReturn.height = renderer.depthBuffer.height;
        toReturn.K[0] = geomData->KK.fx;
        toReturn.K[1] = 0;
        toReturn.K[2] = geomData->KK.cx;
        toReturn.K[4] = geomData->KK.fy;
        toReturn.K[5] = geomData->KK.cy;
        toReturn.K[6] = 0;
        toReturn.K[7] = 0;
        toReturn.K[8] = 1;
        toReturn.P[0] = geomData->KK.fx;
        toReturn.P[1] = 0;
        toReturn.P[2] = geomData->KK.cx;
        toReturn.P[3] = 0;
        toReturn.P[4] = 0;
        toReturn.P[5] = geomData->KK.fy;
        toReturn.P[6] = geomData->KK.cy;
        toReturn.P[7] = 0;
        toReturn.P[8] = 0;
        toReturn.P[9] = 1;
        toReturn.header.frame_id = tfFrame;
        return toReturn;
    }

    sensor_msgs::CameraInfo RaveCamera2ROS::GetColorCameraInfo()
    {
        sensor_msgs::CameraInfo toReturn;
        toReturn.width = renderer.colorBuffer.width;
        toReturn.height = renderer.colorBuffer.height;
        toReturn.K[0] = geomData->KK.fx;
        toReturn.K[1] = 0;
        toReturn.K[2] = geomData->KK.cx;
        toReturn.K[4] = geomData->KK.fy;
        toReturn.K[5] = geomData->KK.cy;
        toReturn.K[6] = 0;
        toReturn.K[7] = 0;
        toReturn.K[8] = 1;
        toReturn.P[0] = geomData->KK.fx;
        toReturn.P[1] = 0;
        toReturn.P[2] = geomData->KK.cx;
        toReturn.P[3] = 0;
        toReturn.P[4] = 0;
        toReturn.P[5] = geomData->KK.fy;
        toReturn.P[6] = geomData->KK.cy;
        toReturn.P[7] = 0;
        toReturn.P[8] = 0;
        toReturn.P[9] = 1;
        toReturn.header.frame_id = tfFrame;
        return toReturn;
    }

    void RaveCamera2ROS::SendTransforms()
    {
        tf::StampedTransform transform;
        transform.child_frame_id_ = tfFrame;
        transform.frame_id_ = baseFrame;
        transform.stamp_ = ros::Time::now();
        OpenRAVE::Transform orTransform = GetTransform();

        transform.setOrigin(tf::Vector3(orTransform.trans.x, orTransform.trans.y, orTransform.trans.z));
        transform.setRotation(tf::Quaternion(orTransform.rot.y, orTransform.rot.z, orTransform.rot.w, orTransform.rot.x));

        tf::StampedTransform opticalTransform;
        opticalTransform.setOrigin(tf::Vector3(orTransform.trans.x, orTransform.trans.y, orTransform.trans.z));
        opticalTransform.setRotation(tf::Quaternion(orTransform.rot.y, orTransform.rot.z, orTransform.rot.w, orTransform.rot.x));
        opticalTransform.frame_id_ = baseFrame;
        opticalTransform.child_frame_id_ = tfFrame + "_optical_frame";
        opticalTransform.stamp_ = ros::Time::now();

        tfBroadcaster.sendTransform(transform);
        tfBroadcaster.sendTransform(opticalTransform);
    }

    void RaveCamera2ROS::PublishImages()
    {
        sensor_msgs::CameraInfo depthInfo = GetDepthCameraInfo();
        sensor_msgs::CameraInfo colorInfo = GetColorCameraInfo();
        DepthBuffer2ROSImg(renderer.depthBuffer, lastDepthImage.get());
        ColorBuffer2ROSImg(renderer.colorBuffer, lastColorImage.get());
        depthCameraPub.publish(*lastDepthImage, depthInfo, ros::Time::now());
        colorCameraPub.publish(*lastColorImage, colorInfo, ros::Time::now());
    }

    void RaveCamera2ROS::PublishCloud()
    {
        cloudGenerator.GenerateCloud(renderer.depthBuffer, renderer.colorBuffer, synthCloud, tfFrame,  geomData->KK.fx, geomData->KK.fy, geomData->KK.cx, geomData->KK.cy);
        cloudGenerator.PublishCloud(synthCloud);
    }

    bool RaveCamera2ROS::SimulationStep(OpenRAVE::dReal fTimeElapsed)
    {
        RaveCamera::SimulationStep(fTimeElapsed);
        SendTransforms();
        PublishImages();
        PublishCloud();
        return true;
    }

}
