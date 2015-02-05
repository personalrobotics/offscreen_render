#include <offscreen_render/CloudGenerator.h>

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

    void CloudGenerator::GenerateCloud(const FrameBuffer& buffer, PointCloud::Ptr cloud, const std::string& frame)
    {
        cloud->header.frame_id = frame;
        cloud->header.stamp = ros::Time::now().toNSec();
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
                    cloud->points.push_back(point);
                }
                i++;
            }
        }
    }
}
