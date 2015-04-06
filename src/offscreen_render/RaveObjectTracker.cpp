#include <offscreen_render/RaveObjectTracker.h>
#include <pcl/common/transforms.h>
namespace offscreen_render
{

    // This defines 128 possible objects to track by their color ID. They are set up to look as different as possible for debugging purposes,
    // perhaps its better to just treat the color as their ID directly?
    uint32_t RaveBridge::colorTable[128] =
    {
            0xFFFFFFFF, 0x000011FF, 0xFF0000FF, 0x00FF00FF, 0x0000FFFF, 0xFFFF00FF, 0x00FFFFFF, 0xFF00FFFF, 0x808080FF, 0xFF8080FF, 0x80FF80FF, 0x8080FFFF, 0x008080FF, 0x800080FF, 0x808000FF, 0xFFFF80FF, 0x80FFFFFF, 0xFF80FFFF, 0xFF0080FF, 0x80FF00FF,
            0x0080FFFF, 0x00FF80FF, 0x8000FFFF, 0xFF8000FF, 0x000080FF, 0x800000FF, 0x008000FF, 0x404040FF, 0xFF4040FF, 0x40FF40FF, 0x4040FFFF, 0x004040FF, 0x400040FF, 0x404000FF, 0x804040FF, 0x408040FF, 0x404080FF, 0xFFFF40FF, 0x40FFFFFF,
            0xFF40FFFF, 0xFF0040FF, 0x40FF00FF, 0x0040FFFF, 0xFF8040FF, 0x40FF80FF, 0x8040FFFF, 0x00FF40FF, 0x4000FFFF, 0xFF4000FF, 0x000040FF, 0x400000FF, 0x004000FF, 0x008040FF, 0x400080FF, 0x804000FF, 0x80FF40FF, 0x4080FFFF, 0xFF4080FF,
            0x800040FF, 0x408000FF, 0x004080FF, 0x808040FF, 0x408080FF, 0x804080FF, 0xC0C0C0FF, 0xFFC0C0FF, 0xC0FFC0FF, 0xC0C0FFFF, 0x00C0C0FF, 0xC000C0FF, 0xC0C000FF, 0x80C0C0FF, 0xC080C0FF, 0xC0C080FF, 0x40C0C0FF, 0xC040C0FF, 0xC0C040FF,
            0xFFFFC0FF, 0xC0FFFFFF, 0xFFC0FFFF, 0xFF00C0FF, 0xC0FF00FF, 0x00C0FFFF, 0xFF80C0FF, 0xC0FF80FF, 0x80C0FFFF, 0xFF40C0FF, 0xC0FF40FF, 0x40C0FFFF, 0x00FFC0FF, 0xC000FFFF, 0xFFC000FF, 0x0000C0FF, 0xC00000FF, 0x00C000FF, 0x0080C0FF,
            0xC00080FF, 0x80C000FF, 0x0040C0FF, 0xC00040FF, 0x40C000FF, 0x80FFC0FF, 0xC080FFFF, 0xFFC080FF, 0x8000C0FF, 0xC08000FF, 0x00C080FF, 0x8080C0FF, 0xC08080FF, 0x80C080FF, 0x8040C0FF, 0xC08040FF, 0x40C080FF, 0x40FFC0FF, 0xC040FFFF,
            0xFFC040FF, 0x4000C0FF, 0xC04000FF, 0x00C040FF, 0x4080C0FF, 0xC04080FF, 0x80C040FF, 0x4040C0FF, 0xC04040FF, 0x40C040FF, 0x202020FF, 0xFF2020FF, 0x20FF20FF
    };

    RaveObjectTracker::RaveObjectTracker() :
            numIters(0),
            needsUpdate(false),
            cancelThreads(false)
    {
        initialized = false;
        window = NULL;
    }

    RaveObjectTracker::RaveObjectTracker(const ros::NodeHandle& handle) :
        node(handle),
        numIters(0),
        needsUpdate(false)
    {
        initialized = false;
        window = NULL;
    }

    RaveObjectTracker::~RaveObjectTracker()
    {
        if(initialized)
        {
            glfwTerminate();
        }
    }

    void RaveObjectTracker::PointCloudCallback(const PointCloud::ConstPtr& pointCloud)
    {
        lastSensorCloud = pointCloud;
    }

    bool RaveObjectTracker::Initialize(
                    const std::string& depthInfoTopic,
                    const std::string& pointCloudTopic,
                    const std::string& fixedFrame,
                    const float near,
                    const float far,
                    const int& numParticles,
                    const float& downsampleSize,
                    const float& maxDistance,
                    const float& octreeLeaf,
                    const float& delta,
                    const float& epsilon,
                    const float& binSize,
                    const float& covariance,
                    const float& resampleLikelihood
                    )
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

        pointCloudSub = node.subscribe<PointCloud>(pointCloudTopic, 1, &RaveObjectTracker::PointCloudCallback, this);

        std::string shaders = ros::package::getPath("offscreen_render") + "/shaders/";

        if(!depthShader.LoadFromFile(shaders + "FragmentShader.glsl", shaders + "VertexShader.glsl"))
        {
            RAVELOG_ERROR("Failed to load depth shaders.");
            return false;
        }

        if(!colorShader.LoadFromFile(shaders + "ColorFragmentShader.glsl", shaders + "VertexShader.glsl"))
        {
            RAVELOG_ERROR("Failed to load color shaders.");
            return false;
        }

        renderer.depthShader = &depthShader;
        renderer.colorShader = &colorShader;

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

        tracker.reset(new Tracker());
        tracker->setDelta(delta);
        tracker->setEpsilon(epsilon);

        pcl::tracking::ParticleXYZRPY bin_size;
        bin_size.x = binSize;
        bin_size.y = binSize;
        bin_size.z = binSize;
        bin_size.roll = binSize;
        bin_size.pitch = binSize;
        bin_size.yaw = binSize;
        tracker->setMaximumParticleNum(numParticles);
        tracker->setBinSize(bin_size);
        tracker->setTrans(Transform::Identity());

        std::vector<double> default_step_covariance = std::vector<double> (6, covariance);
         default_step_covariance[3] *= 100.0;
         default_step_covariance[4] *= 100.0;
         default_step_covariance[5] *= 100.0;
        std::vector<double> initial_noise_covariance = std::vector<double> (6, 0.001);
        std::vector<double> default_initial_mean = std::vector<double> (6, 0.0);

        tracker->setStepNoiseCovariance (default_step_covariance);
        tracker->setInitialNoiseCovariance (initial_noise_covariance);
        tracker->setInitialNoiseMean (default_initial_mean);
        tracker->setParticleNum (numParticles);
        tracker->setResampleLikelihoodThr(resampleLikelihood);
        tracker->setUseNormal(false);
        tracker->setIterationNum(1);
        coherence.reset(new pcl::tracking::ApproxNearestPairPointCloudCoherence<PointCloud::PointType> ());
        distanceCoherence.reset(new pcl::tracking::DistanceCoherence<PointCloud::PointType> ());
        coherence->addPointCoherence (distanceCoherence);

        search.reset(new pcl::search::Octree<PointCloud::PointType> (octreeLeaf));
        coherence->setSearchMethod (search);
        coherence->setMaximumDistance (maxDistance);

        tracker->setCloudCoherence (coherence);
        gridFilter.setLeafSize(downsampleSize, downsampleSize, downsampleSize);
        initialized = true;
        return true;
    }


    void RaveObjectTracker::BeginTracking(const OpenRAVE::KinBodyPtr& body)
    {
        bridge.CreateModels(depthShader, body, OpenRAVE::Vector(1, 1, 1, 1));

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
        numIters = 0;
        trackThread = std::thread(std::bind(&RaveObjectTracker::TrackThread, this));
    }

    void RaveObjectTracker::EndTracking()
    {
        bridge.models.clear();
        bridge.bodies.clear();
        cancelThreads = true;
        trackThread.join();
    }

    void RaveObjectTracker::TrackThread()
    {
        while(!cancelThreads)
        {
            if(needsUpdate && lastSensorCloud.get() && synthCloud.get() && lastSensorCloud->points.size() > 0 && synthCloud->points.size() > 0)
            {
                gridFilter.setInputCloud (lastSensorCloud);
                gridFilter.filter(*filteredSensorCloud);
                gridFilter.setInputCloud (synthCloud);
                gridFilter.filter(*filteredSynthCloud);
                filteredSensorCloud->header.frame_id = depthCamera->frame;

                Transform worldToSensorCloud;
                bool foundTransform = depthCamera->LookupTransform(lastSensorCloud->header.frame_id, worldToSensorCloud);

                if(foundTransform)
                {
                   Transform cameraToSensorCloud = depthCamera->transform.inverse() * worldToSensorCloud;
                   pcl::transformPointCloud(*filteredSensorCloud, *filteredSensorCloud, cameraToSensorCloud);
                   for (OpenRAVE::KinBodyPtr& model : bridge.bodies)
                   {
                       Transform worldToObject = ORToTransform(model->GetTransform());
                       Transform cameraToObject = depthCamera->transform.inverse() * worldToObject;
                       pcl::transformPointCloud(*filteredSynthCloud, *filteredSynthCloud, cameraToObject.inverse());

                       tracker->setTrans(cameraToObject);
                       tracker->setInputCloud(filteredSensorCloud);
                       tracker->setReferenceCloud(filteredSynthCloud);
                       tracker->compute();

                       Transform tf = tracker->getResult().toEigenMatrix();
                       worldToObject = depthCamera->transform * tf;
                       model->GetEnv()->GetMutex().lock();
                       model->SetTransform(TransformToOR(worldToObject));
                       model->GetEnv()->GetMutex().unlock();
                       needsUpdate = false;
                       numIters++;
                   }
                }
            }
            usleep(1000);
        }
    }

    void RaveObjectTracker::Track()
    {
        if(needsUpdate) return;

        ros::spinOnce();
        glfwMakeContextCurrent(window);
        glfwPollEvents();
        renderer.projectionMatrix = GetPerspectiveMatrix(depthCamera->fx, depthCamera->fy, depthCamera->cx, depthCamera->cy, depthCamera->near, depthCamera->far, depthCamera->width, depthCamera->height);
        renderer.viewMatrix = GetViewMatrix(depthCamera->transform);

        renderer.models.clear();
        bridge.UpdateModels();
        bridge.GetAllModels(renderer.models);
        renderer.Draw();
        cloudGenerator.GenerateCloud(renderer.depthBuffer, renderer.colorBuffer, synthCloud, depthCamera->frame, depthCamera->fx, depthCamera->fy, depthCamera->cx, depthCamera->cy);
        cloudGenerator.PublishCloud(synthCloud);
        needsUpdate = true;

    }

    void RaveObjectTracker::TrackObject(const OpenRAVE::KinBodyPtr& body, int iterations)
    {
        if(!initialized)
        {
            RAVELOG_ERROR("Not initialized.");
            return;
        }
        BeginTracking(body);
        while(numIters < iterations)
        {
            if(glfwWindowShouldClose(window))
            {
                EndTracking();
                return;
            }

            Track();
        }
        EndTracking();
    }
}
