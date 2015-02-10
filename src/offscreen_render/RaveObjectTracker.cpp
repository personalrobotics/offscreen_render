#include <offscreen_render/RaveObjectTracker.h>
#include <pcl/common/transforms.h>
namespace offscreen_render
{

    RaveObjectTracker::RaveObjectTracker()
    {
        initialized = false;
        window = NULL;
    }

    RaveObjectTracker::RaveObjectTracker(const ros::NodeHandle& handle) :
        node(handle)
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
            RAVELOG_ERROR("Failed to load shaders.");
            return false;
        }

        renderer.shader = &depthShader;
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
        bridge.CreateModels(depthShader, body);

        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LESS);
        glFrontFace(GL_CCW);
        glEnable(GL_CULL_FACE);
        glCullFace(GL_BACK);
        glfwMakeContextCurrent(window);
        synthCloud.reset(new PointCloud());
        filteredSensorCloud.reset(new PointCloud());

    }

    void RaveObjectTracker::EndTracking()
    {
        bridge.models.clear();
        bridge.bodies.clear();
    }

    void RaveObjectTracker::Track()
    {
        ros::spinOnce();
        glfwPollEvents();
        renderer.projectionMatrix = GetPerspectiveMatrix(depthCamera->fx, depthCamera->fy, depthCamera->cx, depthCamera->cy, depthCamera->near, depthCamera->far, depthCamera->width, depthCamera->height);
        renderer.viewMatrix = GetViewMatrix(depthCamera->transform);

        renderer.models.clear();
        bridge.UpdateModels();
        bridge.GetAllModels(renderer.models);
        renderer.Draw();
        cloudGenerator.GenerateCloud(renderer.buffer, synthCloud, depthCamera->frame, depthCamera->fx, depthCamera->fy, depthCamera->cx, depthCamera->cy);
        cloudGenerator.PublishCloud(synthCloud);

        if(lastSensorCloud.get() && synthCloud.get() && lastSensorCloud->points.size() > 0 && synthCloud->points.size() > 0)
        {
            gridFilter.setInputCloud (lastSensorCloud);
            gridFilter.filter(*filteredSensorCloud);
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
                   pcl::transformPointCloud(*synthCloud, *synthCloud, cameraToObject.inverse());

                   tracker->setTrans(cameraToObject);
                   tracker->setInputCloud(filteredSensorCloud);
                   tracker->setReferenceCloud(synthCloud);
                   tracker->compute();

                   Transform tf = tracker->getResult().toEigenMatrix();
                   worldToObject = depthCamera->transform * tf;
                   model->SetTransform(TransformToOR(worldToObject));
               }
            }
        }
    }

    void RaveObjectTracker::TrackObject(const OpenRAVE::KinBodyPtr& body, int iterations)
    {
        if(!initialized)
        {
            RAVELOG_ERROR("Not initialized.");
            return;
        }
        BeginTracking(body);
        for (int i = 0; i < iterations; i++)
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
