#ifndef RAVEARMTRACKER_H_
#define RAVEARMTRACKER_H_

#include <ros/ros.h>
#include <ros/package.h>
#include <openrave/openrave.h>
#include <offscreen_render/OffscreenRenderer.h>
#include <offscreen_render/ROSCamera.h>
#include <offscreen_render/RaveBridge.h>
#include <offscreen_render/CloudGenerator.h>
#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/tracker.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <GLFW/glfw3.h>
#include <thread>

namespace offscreen_render
{

    class RaveArmTracker
    {
        public:
            typedef pcl::search::Octree<PointCloud::PointType> Octree;
            RaveArmTracker();
            RaveArmTracker(const ros::NodeHandle& handle);
            virtual ~RaveArmTracker();
            void PointCloudCallback(const PointCloud::ConstPtr& pointCloud);

            void BeginTracking(const OpenRAVE::RobotBase::ManipulatorPtr& manipulator);
            void EndTracking();
            void TrackThread();

            bool Initialize(
                     const std::string& depthInfoTopic,
                     const std::string& pointCloudTopic,
                     const std::string& fixedFrame = "/map",
                     const float& near = 0.01f,
                     const float& far = 0.01f,
                     const float& downsampleSize = 0.005f,
                     const float& octreeLeaf = 0.005f,
                     const float& searchRadius = 0.1f);

            OpenRAVE::EnvironmentBasePtr env;
            OpenRAVE::RobotBasePtr robot;
            ros::NodeHandle node;
            ros::Subscriber pointCloudSub;
            OffscreenRenderer renderer;
            Shader depthShader;
            Shader colorShader;
            std::shared_ptr<ROSCamera> depthCamera;
            RaveBridge bridge;
            CloudGenerator cloudGenerator;
            PointCloud::ConstPtr lastSensorCloud;
            PointCloud::Ptr filteredSensorCloud;
            PointCloud::Ptr synthCloud;
            PointCloud::Ptr filteredSynthCloud;
            std::shared_ptr<Octree> octree;
            pcl::VoxelGrid<PointCloud::PointType> gridFilter;
            GLFWwindow* window;
            bool initialized;
            bool needsUpdate;
            bool cancelThreads;
            std::thread trackThread;
            float searchRadius;
    };

}

#endif // RAVEARMTRACKER_H_ 
