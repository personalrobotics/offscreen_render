/*
 * RaveCamera.cpp
 *
 *  Created on: Apr 6, 2015
 *      Author: mklingen
 */

#include <openrave/openrave.h>
#include <offscreen_render/RaveCamera.h>
#include <offscreen_render/Geometry.h>
#include <offscreen_render/Conversions.h>
#include <ros/package.h>

namespace offscreen_render
{
    RaveCamera::RaveCamera(OpenRAVE::EnvironmentBasePtr env) :
            OpenRAVE::SensorBase(env),
            isRunning(false),
            isInitialized(false),
            window(0x0)
    {
       geomData = new OpenRAVE::SensorBase::CameraGeomData();
       geometry.reset(geomData);
       near = 0.01f;
       far = 10.0f;
    }

    RaveCamera::~RaveCamera()
    {
        if(isInitialized)
        {
            glfwTerminate();
        }
    }

    bool RaveCamera::Initialize()
    {
       isInitialized = true;

       if (!glfwInit())
       {
           RAVELOG_ERROR("Failed to initialize OpenGL");
           return false;
       }

       window = glfwCreateWindow(1, 1, "Offscreen Window", NULL, NULL);

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

        renderer.Initialize((int)geomData->width, (int)geomData->height);

        return true;

    }

    int RaveCamera::Configure(OpenRAVE::SensorBase::ConfigureCommand command, bool blocking)
    {
        switch (command)
        {
            case OpenRAVE::SensorBase::CC_PowerOn:
                isRunning = true;
                if(!isInitialized)
                {
                    if(Initialize())
                    {
                        return 0;
                    }
                    else return -1;
                }
                return 0;
            case OpenRAVE::SensorBase::CC_PowerOff:
                isRunning = false;
                return 0;
            case OpenRAVE::SensorBase::CC_PowerCheck:
                return static_cast<int>(isRunning);
            default:
                return 0;
        }
    }

    void RaveCamera::SetIntrinsics(float fx, float fy, float cx, float cy)
    {
        geomData->KK.fx = fx;
        geomData->KK.fy = fy;
        geomData->KK.cx = cx;
        geomData->KK.cy = cy;
    }

    OpenRAVE::SensorBase::SensorGeometryPtr RaveCamera::GetSensorGeometry(SensorType type)
    {
        if (type != OpenRAVE::SensorBase::ST_Camera)
        {
            RAVELOG_ERROR("Only camera sensor geometry is valid.\n");
            return OpenRAVE::SensorBase::SensorGeometryPtr();
        }
        return geometry;
    }

    OpenRAVE::SensorBase::SensorDataPtr RaveCamera::CreateSensorData(OpenRAVE::SensorBase::SensorType type)
    {
        if (type != OpenRAVE::SensorBase::ST_Camera)
        {
            RAVELOG_ERROR("Only camera sensor geometry is valid.\n");
            return OpenRAVE::SensorBase::SensorDataPtr();
        }
        OpenRAVE::SensorBase::CameraSensorData* data = new OpenRAVE::SensorBase::CameraSensorData();
        data->vimagedata.resize(geomData->width * geomData->height * 3);
        data->__trans = transform;
        return OpenRAVE::SensorBase::SensorDataPtr(data);
    }

    bool RaveCamera::GetSensorData(OpenRAVE::SensorBase::SensorDataPtr psensordata)
    {
        // TODO: Implement
        return false;
    }

    bool RaveCamera::Supports(OpenRAVE::SensorBase::SensorType type)
    {
        return type == OpenRAVE::SensorBase::ST_Camera;
    }

    void RaveCamera::SetTransform(OpenRAVE::Transform const &trans)
    {
        transform = trans;
    }

    OpenRAVE::Transform RaveCamera::GetTransform()
    {
        return transform;
    }

    bool RaveCamera::SimulationStep(OpenRAVE::dReal fTimeElapsed)
    {
        if (isInitialized && isRunning)
        {
            glEnable(GL_DEPTH_TEST);
            glDepthFunc(GL_LESS);
            glFrontFace(GL_CCW);
            glEnable(GL_CULL_FACE);
            glCullFace(GL_BACK);
            glfwMakeContextCurrent(window);
            renderer.projectionMatrix = GetPerspectiveMatrix(geomData->KK.fx, geomData->KK.fy, geomData->KK.cx, geomData->KK.cy, near, far, geomData->width, geomData->height);
            renderer.viewMatrix = GetViewMatrix(ORToTransform(transform));

            renderer.models.clear();
            bridge.UpdateModels();
            bridge.GetAllModels(renderer.models);
            renderer.Draw();
            return true;
        }

        return false;
    }

}
