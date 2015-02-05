#ifndef CLOUDGENERATOR_H_
#define CLOUDGENERATOR_H_

#include <sensor_msgs/PointCloud2.h>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <offscreen_render/OffscreenRenderer.h>

namespace offscreen_render
{
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

    class CloudGenerator
    {
        public:
            CloudGenerator();
            virtual ~CloudGenerator();

            void Initialize(ros::NodeHandle& nh, const std::string& topic);
            PointCloud::Ptr NewCloud()
            {
                return PointCloud::Ptr(new PointCloud());
            }
            inline void PublishCloud(PointCloud::Ptr cloud)
            {
                pointCloudPublisher.publish(cloud);
            }
            void GenerateCloud(const FrameBuffer& buffer, PointCloud::Ptr cloud, const std::string& frame);
            ros::Publisher pointCloudPublisher;
    };

}

#endif // CLOUDGENERATOR_H_ 
