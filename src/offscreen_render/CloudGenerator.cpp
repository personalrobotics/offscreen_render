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

    void CloudGenerator::GenerateCloud(const FrameBuffer<float, 3>& buffer, const FrameBuffer<float, 3>& colorBuffer,  PointCloud::Ptr cloud, const std::string& frame, const float& fx, const float& fy, const float& cx, const float& cy)
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
                    point.x = x - 0.02f;
                    point.y = y;
                    point.z = z + 0.005f;
                    point.r = colorBuffer.data[i * 3 + 0];
                    point.g = colorBuffer.data[i * 3 + 1];
                    point.b = colorBuffer.data[i * 3 + 2];
                    point.a = 255;
                    cloud->points.push_back(point);
                }
                i++;
            }
        }
    }
}
