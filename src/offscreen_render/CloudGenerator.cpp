#include <offscreen_render/CloudGenerator.h>
#include <pcl_conversions/pcl_conversions.h>

namespace offscreen_render
{

    CloudGenerator::CloudGenerator()
    {
        // TODO Auto-generated constructor stub

    }

    CloudGenerator::~CloudGenerator()
    {
        // TODO Auto-generated destructor stub
    }

    void CloudGenerator::Initialize(ros::NodeHandle& nh, const std::string& topic)
    {
        pointCloudPublisher = nh.advertise<PointCloud>(topic.c_str(), 1);
    }

    void CloudGenerator::GenerateCloud(const FrameBuffer<float, 3>& buffer, const FrameBuffer<float, 3>& colorBuffer,  PointCloud::Ptr cloud, const std::string& frame, const float& fx, const float& fy, const float& cx, const float& cy)
    {
        cloud->header.frame_id = frame;
        pcl_conversions::toPCL(ros::Time::now(), cloud->header.stamp);
        cloud->is_dense = false;
        cloud->points.clear();
        size_t i = 0;
        PointCloud::PointType point;
        for (int r = 0; r < buffer.width; r++)
        {
            for (int c = 0; c < buffer.height; c++)
            {

                const float& z = buffer.data[i * 3 + 2];
                if(z > 0)
                {
                    const float& x = buffer.data[i * 3 + 0];
                    const float& y = buffer.data[i * 3 + 1];
                    point.x = x;
                    point.y = y;
                    point.z = z;
                    point.r = static_cast<uint8_t>(255 * colorBuffer.data[i * 3 + 0]);
                    point.g = static_cast<uint8_t>(255 * colorBuffer.data[i * 3 + 1]);
                    point.b = static_cast<uint8_t>(255 * colorBuffer.data[i * 3 + 2]);
                    point.a = 255;
                    cloud->points.push_back(point);
                }
                i++;
            }
        }
    }
}
