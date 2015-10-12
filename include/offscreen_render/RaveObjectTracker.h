#ifndef RAVEOBJECTTRACKER_H_
#define RAVEOBJECTTRACKER_H_

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
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <GLFW/glfw3.h>

namespace offscreen_render
{
    typedef pcl::tracking::KLDAdaptiveParticleFilterOMPTracker<PointCloud::PointType, pcl::tracking::ParticleXYZRPY> Tracker;
    class RaveObjectTracker
    {
        public:
            RaveObjectTracker();
            RaveObjectTracker(const ros::NodeHandle& handle);
            virtual ~RaveObjectTracker();

            bool Initialize(const std::string& depthInfoTopic,
                            const std::string& pointCloudTopic,
                            const std::string& fixedFrame = "/map",
                            const float near = 0.01,
                            const float far = 100.0f,
                            const int& numParticles = 500,
                            const float& downsampleSize = 0.005f,
                            const float& maxDistance = 0.01f,
                            const float& octreeLeaf = 0.005f,
                            const float& delta = 0.99f,
                            const float& epsilon = 0.2f,
                            const float& binSize = 0.1f,
                            const float& covariance = 0.000225f,
                            const float& resampleLikelihood = 0.00f
                            );


            void PointCloudCallback(const PointCloud::ConstPtr& pointCloud);

            void BeginTracking(const OpenRAVE::KinBodyPtr& body);
            void EndTracking();
            void Track();
            void TrackThread();

            void TrackObject(const OpenRAVE::KinBodyPtr& body, int iterations);

            ros::NodeHandle node;
            ros::Subscriber pointCloudSub;
            OffscreenRenderer renderer;
            Shader depthShader;
            Shader colorShader;
            boost::shared_ptr<ROSCamera> depthCamera;
            RaveBridge bridge;
            CloudGenerator cloudGenerator;
            pcl::VoxelGrid<PointCloud::PointType> gridFilter;
            pcl::tracking::ApproxNearestPairPointCloudCoherence<PointCloud::PointType>::Ptr coherence;
            boost::shared_ptr<pcl::tracking::DistanceCoherence<PointCloud::PointType> > distanceCoherence;
            boost::shared_ptr<pcl::search::Octree<PointCloud::PointType> > search;
            PointCloud::ConstPtr lastSensorCloud;
            PointCloud::Ptr filteredSensorCloud;
            PointCloud::Ptr synthCloud;
            PointCloud::Ptr filteredSynthCloud;
            boost::shared_ptr<Tracker> tracker;
            GLFWwindow* window;
            bool initialized;
            bool needsUpdate;
            bool cancelThreads;
            boost::thread trackThread;
            int numIters;
    };

}

#endif // RAVEOBJECTTRACKER_H_ 
