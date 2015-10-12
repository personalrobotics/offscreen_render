#include <offscreen_render/RaveArmTracker.h>

namespace offscreen_render
{

    RaveArmTracker::RaveArmTracker() :
            cancelThreads(false),
            needsUpdate(false),
            window(0x0),
            initialized(false)
    {

    }

    RaveArmTracker::RaveArmTracker(const ros::NodeHandle& handle) :
        node(handle),
        needsUpdate(false),
        window(0x0),
        initialized(false),
        cancelThreads(false)
    {

    }

    RaveArmTracker::~RaveArmTracker()
    {

    }


    void RaveArmTracker::TrackThread()
    {
        while(!cancelThreads)
        {
           if(needsUpdate &&
              lastSensorCloud.get() &&
              synthCloud.get() &&
              !lastSensorCloud->points.empty() &&
              !synthCloud->points.empty())
           {
               gridFilter.setInputCloud(lastSensorCloud);
               gridFilter.filter(*filteredSensorCloud);
               gridFilter.setInputCloud (synthCloud);
               gridFilter.filter(*filteredSynthCloud);
               filteredSensorCloud->header.frame_id = depthCamera->frame;

               Transform worldToSensorCloud;
               bool foundTransform = depthCamera->LookupTransform(lastSensorCloud->header.frame_id, worldToSensorCloud);

               if(!foundTransform)
                   continue;

               Transform cameraToSensorCloud = depthCamera->transform.inverse() * worldToSensorCloud;
               pcl::transformPointCloud(*filteredSensorCloud, *filteredSensorCloud, cameraToSensorCloud);

               octree->setInputCloud(filteredSensorCloud);

               for (size_t p = 0; p < filteredSynthCloud->points.size(); p++)
               {
                   const PointCloud::PointType& point = filteredSynthCloud->points.at(p);
                   std::vector<int> result;
                   std::vector<float> squaredDists;
                   int numFound = octree->radiusSearch(point, searchRadius, result, squaredDists);

                   if(!numFound) continue;

                   float mindist = std::numeric_limits<float>::max();
                   int minIdx = -1;
                   for (int i = 0; i < numFound; i++)
                   {
                       if (squaredDists[i] < mindist)
                       {
                           mindist = squaredDists[i];
                           minIdx = result[i];
                       }
                   }

                   const PointCloud::PointType& nearestNeighbor = filteredSynthCloud->points[minIdx];
                   int linkIdx = bridge.GetColorIndex(nearestNeighbor.r, nearestNeighbor.g, nearestNeighbor.b, nearestNeighbor.a);

                   // TODO: now you have all you need for the optimization. So do it.

               }

               needsUpdate = false;
           }
        }
    }

    bool RaveArmTracker::Initialize(
             const std::string& depthInfoTopic,
             const std::string& pointCloudTopic,
             const std::string& fixedFrame,
             const float& near,
             const float& far,
             const float& downsampleSize,
             const float& octreeLeaf,
             const float& searchRadius)
    {
        if (!glfwInit())
        {
            RAVELOG_ERROR("Failed to initialize OpenGL");
            return false;
        }
        window = glfwCreateWindow(1, 1, "Object Tracker", NULL, NULL);

        if(!window)
        {
            glfwTerminate();
            RAVELOG_ERROR("Failed to create offscreen window");
            return false;
        }
        glfwMakeContextCurrent(window);
        glfwSwapInterval(0);
        glfwHideWindow(window);
        GLenum err = glewInit();
        if (err != GLEW_OK)
        {
            RAVELOG_ERROR("Failed to initialize GLEW.");
            return false;
        }
        if (!GLEW_VERSION_2_1)
        {
            RAVELOG_ERROR("Need GLEW 2.1 or higher.");
            return false;
        }

        pointCloudSub = node.subscribe<PointCloud>(pointCloudTopic, 1, &RaveArmTracker::PointCloudCallback, this);

        std::string shaders = ros::package::getPath("offscreen_render") + "/shaders/";

        if(!depthShader.LoadFromFile(shaders + "FragmentShader.glsl", shaders + "VertexShader.glsl"))
        {
            RAVELOG_ERROR("Failed to load shaders.");
            return false;
        }

        if(!colorShader.LoadFromFile(shaders + "ColorFragmentShader.glsl", shaders + "VertexShader.glsl"))
        {
            RAVELOG_ERROR("Failed to load color shaders.");
            return false;
        }

        renderer.colorShader = &colorShader;
        renderer.depthShader = &depthShader;
        depthCamera.reset(new ROSCamera(node, depthInfoTopic, fixedFrame, near, far));
        int tries = 0;
        while (!depthCamera->hasTransform && tries < 1000)
        {
            ros::spinOnce();
            usleep(1000);
            tries++;
        }

        if(tries >= 1000)
        {
            RAVELOG_ERROR("Failed to connect to camera %s\n", depthInfoTopic.c_str());
            return false;
        }

        renderer.Initialize((int)depthCamera->width, (int)depthCamera->height);
        cloudGenerator.Initialize(node, "synth_point_cloud");
        octree.reset(new Octree(octreeLeaf));
        gridFilter.setLeafSize(downsampleSize, downsampleSize, downsampleSize);
        initialized = true;
        this->searchRadius = searchRadius;
        return true;
    }

    void RaveArmTracker::PointCloudCallback(const PointCloud::ConstPtr& pointCloud)
    {
        lastSensorCloud = pointCloud;
    }

    void RaveArmTracker::EndTracking()
    {
        bridge.models.clear();
        bridge.bodies.clear();
        cancelThreads = true;
        trackThread.join();
    }

    void RaveArmTracker::BeginTracking(const OpenRAVE::RobotBase::ManipulatorPtr& body)
    {
        bridge.CreateModels(depthShader, body);

        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LESS);
        glFrontFace(GL_CCW);
        glEnable(GL_CULL_FACE);
        glCullFace(GL_BACK);
        glfwMakeContextCurrent(window);
        synthCloud.reset(new PointCloud());
        filteredSensorCloud.reset(new PointCloud());
        filteredSynthCloud.reset(new PointCloud());
        cancelThreads = false;
        needsUpdate = false;
        trackThread = boost::thread(boost::bind(&RaveArmTracker::TrackThread, this));
    }
}
