#include <offscreen_render/OffscreenRenderer.h>
#include <offscreen_render/ROSCamera.h>
#include <offscreen_render/RaveBridge.h>
#include <offscreen_render/CloudGenerator.h>
#include <ros/ros.h>
#include <openrave/openrave.h>
#include <openrave-core.h>
#include <thread>
#include <functional>
#include <GLFW/glfw3.h>
#include <pcl/registration/icp.h>
#include <pcl/tracking/tracking.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/particle_filter_omp.h>
#include <pcl/common/common.h>

#include <pcl/tracking/coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/hsv_color_coherence.h>
#include <pcl/tracking/normal_coherence.h>

#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>

using namespace offscreen_render;
OpenRAVE::EnvironmentBasePtr environment;
PointCloud::ConstPtr lastCloud;
void ViewerThread()
{
    OpenRAVE::ViewerBasePtr viewer = OpenRAVE::RaveCreateViewer(environment, "rviz");
    environment->Add(viewer);
    bool showgui = true;
    viewer->main(showgui);
}

void PointCloudCallback(const PointCloud::ConstPtr& pointCloud)
{
    lastCloud = pointCloud;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename PointT> void calcBoundingBox(const pcl::PointCloud<PointT>& cloud, double &x_min, double &x_max, double &y_min, double &y_max, double &z_min, double &z_max)
{
    x_min = y_min = z_min = std::numeric_limits<double>::max();
    x_max = y_max = z_max = -std::numeric_limits<double>::max();

    PointT Pmin, Pmax;

    pcl::getMinMax3D(cloud, Pmin, Pmax);
    if (x_min > Pmin.x)
        x_min = Pmin.x;
    if (x_max < Pmax.x)
        x_max = Pmax.x;
    if (y_min > Pmin.y)
        y_min = Pmin.y;
    if (y_max < Pmax.y)
        y_max = Pmax.y;
    if (z_min > Pmin.z)
        z_min = Pmin.z;
    if (z_max < Pmax.z)
        z_max = Pmax.z;
}

int main(int argc, char** argv)
{
    // Initialize ROS
    ros::init(argc, argv, "offscreen_renderer");
    ros::NodeHandle node("~");
    ros::Subscriber pointCloudSub = node.subscribe<PointCloud>("/camera/depth_registered/points", 1, PointCloudCallback);
    // Initialize OpenGL
    GLFWwindow* window;
    if (!glfwInit())
        return -1;

    window = glfwCreateWindow(640, 480, "Hello World", NULL, NULL);
    if (!window)
    {
        glfwTerminate();
        return -1;
    }

    // Initialize OpenRAVE
    OpenRAVE::RaveInitialize(true);
    environment = OpenRAVE::RaveCreateEnvironment();
    std::thread viewThread(ViewerThread);
    environment->Load("/homes/mklingen/prdev_catkin/src/pr-ordata/data/objects/pop_tarts.kinbody.xml");

    glfwMakeContextCurrent(window);

    GLenum err = glewInit();
    if (err != GLEW_OK)
    {
        printf("Failed to initialize GLEW.\n");
        exit(1);
    }
    if (!GLEW_VERSION_2_1)
    {
        printf("Need GLEW 2.1 or higher.\n");
        exit(1);
    }

    OffscreenRenderer renderer;
    Shader depthShader;
    depthShader.LoadFromFile("./shaders/FragmentShader.glsl", "./shaders/VertexShader.glsl");
    renderer.shader = &depthShader;
    ROSCamera camera(node, "/camera/depth/camera_info", "/map", 0.1f, 100.0f);

    OpenRAVE::KinBodyPtr fuze = environment->GetKinBody("pop_tarts");
    OpenRAVE::Transform fuzeTF = fuze->GetTransform();
    fuzeTF.trans = OpenRAVE::Vector(0.75, -0.15, -0.1);
    fuze->SetTransform(fuzeTF);

    RaveBridge bridge;
    bridge.CreateModels(depthShader, fuze);

    while (!camera.hasTransform)
    {
        ros::spinOnce();
        usleep(1000);
    }
    renderer.Initialize((int)camera.width, (int)camera.height);
    CloudGenerator generator;
    generator.Initialize(node, "synth_point_cloud");
    PointCloud::Ptr cloud = generator.NewCloud();

    // Enable depth test
    glEnable(GL_DEPTH_TEST);
    // Accept fragment if it closer to the camera than the former one
    glDepthFunc(GL_LESS);
    glFrontFace(GL_CCW);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    pcl::tracking::KLDAdaptiveParticleFilterOMPTracker<PointCloud::PointType, pcl::tracking::ParticleXYZRPY> tracker;
    tracker.setMaximumParticleNum(500);
    tracker.setDelta(0.99);
    tracker.setEpsilon(0.2);
    pcl::tracking::ParticleXYZRPY bin_size;
    bin_size.x = 0.1f;
    bin_size.y = 0.1f;
    bin_size.z = 0.1f;
    bin_size.roll = 0.1f;
    bin_size.pitch = 0.1f;
    bin_size.yaw = 0.1f;
    tracker.setBinSize(bin_size);
    tracker.setTrans(Transform::Identity());
    std::vector<double> default_step_covariance = std::vector<double> (6, 0.015 * 0.015);
    default_step_covariance[3] *= 100.0;
    default_step_covariance[4] *= 100.0;
    default_step_covariance[5] *= 100.0;

    std::vector<double> initial_noise_covariance = std::vector<double> (6, 0.001);
    std::vector<double> default_initial_mean = std::vector<double> (6, 0.0);

    tracker.setStepNoiseCovariance (default_step_covariance);
    tracker.setInitialNoiseCovariance (initial_noise_covariance);
    tracker.setInitialNoiseMean (default_initial_mean);
    tracker.setIterationNum (1);

    tracker.setParticleNum (400);
    tracker.setResampleLikelihoodThr(0.00);
    tracker.setUseNormal (false);

    // setup coherences
    pcl::tracking::ApproxNearestPairPointCloudCoherence<PointCloud::PointType>::Ptr coherence = pcl::tracking::ApproxNearestPairPointCloudCoherence<PointCloud::PointType>::Ptr
      (new pcl::tracking::ApproxNearestPairPointCloudCoherence<PointCloud::PointType> ());

    boost::shared_ptr<pcl::tracking::DistanceCoherence<PointCloud::PointType> > distance_coherence
      = boost::shared_ptr<pcl::tracking::DistanceCoherence<PointCloud::PointType> > (new pcl::tracking::DistanceCoherence<PointCloud::PointType> ());
    coherence->addPointCoherence (distance_coherence);

    //boost::shared_ptr<pcl::search::KdTree<RefPointType> > search (new pcl::search::KdTree<RefPointType> (false));
    boost::shared_ptr<pcl::search::Octree<PointCloud::PointType> > search (new pcl::search::Octree<PointCloud::PointType> (0.01));
    //boost::shared_ptr<pcl::search::OrganizedNeighbor<RefPointType> > search (new pcl::search::OrganizedNeighbor<RefPointType>);
    coherence->setSearchMethod (search);
    coherence->setMaximumDistance (0.01);

    tracker.setCloudCoherence (coherence);
    glfwSetWindowSize(window, camera.width, camera.height);
    PointCloud::Ptr particleClouds = generator.NewCloud();
    float t = 0;
    while (!glfwWindowShouldClose(window))
    {
        particleClouds->header.frame_id = camera.frame;
        particleClouds->header.stamp = ros::Time::now().toNSec();
        t += 0.01f;
        ros::spinOnce();
        renderer.projectionMatrix = GetPerspectiveMatrix(camera.fx, camera.fy, camera.cx, camera.cy, camera.near, camera.far, camera.width, camera.height);
        renderer.viewMatrix = GetViewMatrix(camera.transform);
        //renderer.viewMatrix = GetLookAt(Vec3(sin(t), cos(t), 0), Vec3(0, 0, 0), Vec3(0, 0, 1));
        renderer.models.clear();
        bridge.UpdateModels();
        bridge.GetAllModels(renderer.models);
        renderer.Draw();
        generator.GenerateCloud(renderer.buffer, cloud, camera.frame);
        glfwSwapBuffers(window);
        glfwPollEvents();


        generator.PublishCloud(cloud);
        if(lastCloud.get() && cloud->points.size() > 0 && lastCloud->points.size() > 0)
        {
            particleClouds->points.clear();
            printf("Starting track %lu to %lu...\n", cloud->points.size(), lastCloud->points.size());
            double x_min, x_max, y_min, y_max, z_min, z_max;
            calcBoundingBox(*cloud, x_min, x_max, y_min, y_max, z_min, z_max);
            printf("%f %f %f %f %f %f\n", x_min, x_max, y_min, y_max, z_min, z_max);
            Transform worldToObject = ORToTransform(fuze->GetTransform());
            Transform cameraToObject = camera.transform.inverse() * worldToObject;
            pcl::transformPointCloud(*cloud, *cloud, cameraToObject.inverse());
            tracker.setTrans(cameraToObject);
            tracker.setInputCloud(lastCloud);
            tracker.setReferenceCloud(cloud);
            tracker.getCloudCoherence()->setTargetCloud(lastCloud);
            tracker.compute();

            auto particles =tracker.getParticles();
            for(size_t i = 0; i < tracker.getParticleNum(); i++)
            {
                const pcl::tracking::ParticleXYZRPY& particle = particles->points[i];
                PointCloud::PointType point;
                point.x  = particle.x;
                point.y = particle.y;
                point.z = particle.z;
                particleClouds->points.push_back(point);
            }

            Transform tf = tracker.getResult().toEigenMatrix();
            std::stringstream ss;
            ss << tf.matrix();
            printf("%s\n", ss.str().c_str());

            worldToObject = camera.transform * tf;
            fuze->SetTransform(TransformToOR(worldToObject));

        }



    }
    viewThread.join();
    glfwTerminate();
    return 0;
}

/*
// Include standard headers
#include <stdio.h>
#include <stdlib.h>

// Include GLEW
#include <GL/glew.h>

// Include GLFW
#include <GLFW/glfw3.h>
 #include <offscreen_render/Geometry.h>
 #include <offscreen_render/Shader.h>
#include <offscreen_render/VertexBuffer.h>

GLFWwindow* window;
 using namespace offscreen_render;
int main(void)
{
    // Initialise GLFW
    if (!glfwInit())
    {
        fprintf( stderr, "Failed to initialize GLFW\n");
        return -1;
    }

    glfwWindowHint(GLFW_SAMPLES, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // Open a window and create its OpenGL context
    window = glfwCreateWindow(640, 480, "Tutorial 04 - Colored Cube", NULL, NULL);
    if (window == NULL)
    {
        fprintf( stderr, "Failed to open GLFW window. If you have an Intel GPU, they are not 3.3 compatible. Try the 2.1 version of the tutorials.\n");
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);

    // Initialize GLEW
    glewExperimental = true; // Needed for core profile
    if (glewInit() != GLEW_OK)
    {
        fprintf(stderr, "Failed to initialize GLEW\n");
        return -1;
    }

    // Ensure we can capture the escape key being pressed below
    glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);

    // Dark blue background
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

    // Enable depth test
    glEnable(GL_DEPTH_TEST);
    // Accept fragment if it closer to the camera than the former one
    glDepthFunc(GL_LESS);

    Shader shader;
    shader.LoadFromFile("./shaders/FragmentShader.glsl", "./shaders/VertexShader.glsl");

    // Create and compile our GLSL program from the shaders
    GLuint programID = shader.programID;


    // Our vertices. Tree consecutive floats give a 3D vertex; Three consecutive vertices give a triangle.
    // A cube has 6 faces with 2 triangles each, so this makes 6*2=12 triangles, and 12*3 vertices
    static const GLfloat g_vertex_buffer_data[] =
    {
            -1.0f, -1.0f, -1.0f,
            -1.0f, -1.0f, 1.0f,
            -1.0f, 1.0f, 1.0f,
            1.0f, 1.0f, -1.0f,
            -1.0f, -1.0f, -1.0f,
            -1.0f, 1.0f, -1.0f,
            1.0f, -1.0f, 1.0f,
            -1.0f, -1.0f, -1.0f,
            1.0f, -1.0f, -1.0f,
            1.0f, 1.0f, -1.0f,
            1.0f, -1.0f, -1.0f,
            -1.0f, -1.0f, -1.0f,
            -1.0f, -1.0f, -1.0f,
            -1.0f, 1.0f, 1.0f,
            -1.0f, 1.0f, -1.0f,
            1.0f, -1.0f, 1.0f,
            -1.0f, -1.0f, 1.0f,
            -1.0f, -1.0f, -1.0f,
            -1.0f,  1.0f, 1.0f,
            -1.0f, -1.0f, 1.0f,
            1.0f, -1.0f, 1.0f,
            1.0f, 1.0f, 1.0f,
            1.0f,  -1.0f, -1.0f,
            1.0f, 1.0f, -1.0f,
            1.0f, -1.0f, -1.0f,
            1.0f, 1.0f, 1.0f,
            1.0f,  -1.0f, 1.0f,
            1.0f,  1.0f, 1.0f,
            1.0f,  1.0f, -1.0f,
            -1.0f,  1.0f, -1.0f,
            1.0f, 1.0f, 1.0f,
            -1.0f,  1.0f, -1.0f,
            -1.0f,  1.0f, 1.0f,
            1.0f,  1.0f, 1.0f,
            -1.0f, 1.0f, 1.0f,
            1.0f, -1.0f, 1.0f };

    // One color for each vertex. They were generated randomly.
    static const GLfloat g_color_buffer_data[] =
    { 0.583f, 0.771f, 0.014f, 0.609f, 0.115f, 0.436f, 0.327f, 0.483f, 0.844f, 0.822f, 0.569f, 0.201f, 0.435f, 0.602f, 0.223f, 0.310f, 0.747f, 0.185f, 0.597f, 0.770f, 0.761f, 0.559f, 0.436f, 0.730f, 0.359f, 0.583f, 0.152f, 0.483f, 0.596f, 0.789f,
            0.559f, 0.861f, 0.639f, 0.195f, 0.548f, 0.859f, 0.014f, 0.184f, 0.576f, 0.771f, 0.328f, 0.970f, 0.406f, 0.615f, 0.116f, 0.676f, 0.977f, 0.133f, 0.971f, 0.572f, 0.833f, 0.140f, 0.616f, 0.489f, 0.997f, 0.513f, 0.064f, 0.945f, 0.719f,
            0.592f, 0.543f, 0.021f, 0.978f, 0.279f, 0.317f, 0.505f, 0.167f, 0.620f, 0.077f, 0.347f, 0.857f, 0.137f, 0.055f, 0.953f, 0.042f, 0.714f, 0.505f, 0.345f, 0.783f, 0.290f, 0.734f, 0.722f, 0.645f, 0.174f, 0.302f, 0.455f, 0.848f, 0.225f,
            0.587f, 0.040f, 0.517f, 0.713f, 0.338f, 0.053f, 0.959f, 0.120f, 0.393f, 0.621f, 0.362f, 0.673f, 0.211f, 0.457f, 0.820f, 0.883f, 0.371f, 0.982f, 0.099f, 0.879f };

    VertexBuffer cubeBuffer;

    for(unsigned short i = 0; i < 12 * 3; i++)
    {
        cubeBuffer.position_data.push_back(g_vertex_buffer_data[i * 3 + 0]);
        cubeBuffer.position_data.push_back(g_vertex_buffer_data[i * 3 + 1]);
        cubeBuffer.position_data.push_back(g_vertex_buffer_data[i * 3 + 2]);
        cubeBuffer.color_data.push_back(g_color_buffer_data[i * 3 + 0]);
        cubeBuffer.color_data.push_back(g_color_buffer_data[i * 3 + 1]);
        cubeBuffer.color_data.push_back(g_color_buffer_data[i * 3 + 2]);
        cubeBuffer.index_data.push_back(i);
    }


    cubeBuffer.Initialize(shader);


    glFrontFace(GL_CCW);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);

    float t = 0;
    do
    {
        t += 0.01f;
        // Clear the screen
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Use our shader
        shader.Begin();
        Mat4x4 m1 = GetPerspectiveMatrix(575.0f, 575.0f, 314.5f, 235.0f, 0.1f, 100.0f, 640, 480);
        shader.SetProjectionMatrix(m1);
        shader.SetViewMatrix(GetLookAt(Vec3(10 * sin(t), 3, -10 * cos(t)), Vec3(0, 0, 0), Vec3(0, 1, 0)));
        shader.SetWorldMatrix(Mat4x4::Identity());


        cubeBuffer.Begin();
        cubeBuffer.Draw();
        cubeBuffer.End();

        shader.End();

        // Swap buffers
        glfwSwapBuffers(window);
        glfwPollEvents();

    } // Check if the ESC key was pressed or the window was closed
    while (glfwGetKey(window, GLFW_KEY_ESCAPE) != GLFW_PRESS && glfwWindowShouldClose(window) == 0);


    // Close OpenGL window and terminate GLFW
    glfwTerminate();

    return 0;
}
*/
