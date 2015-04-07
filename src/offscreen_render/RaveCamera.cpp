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

       RegisterCommand("setintrinsic",boost::bind(&RaveCamera::_SetIntrinsics,this,_1,_2),
                                "Set the intrinsic parameters of the camera (fx,fy,cx,cy,near,far).");
       RegisterCommand("setdims",boost::bind(&RaveCamera::_SetSize,this,_1,_2),
                                "Set the dimensions of the image (width,height)");
       RegisterCommand("addbody",boost::bind(&RaveCamera::_AddKinBody,this,_1,_2),
                                "Add a kinbody to the render scene with the given 24 bit color (name, r, g, b)");
    }

    RaveCamera::~RaveCamera()
    {
        if(isInitialized)
        {
            glfwTerminate();
        }
    }

    void RaveCamera::SetSize(int w, int h)
    {
        geomData->width = w;
        geomData->height = h;

        if (isInitialized)
        {
            RAVELOG_WARN("Can't set parameters after the camera is initialized. Please create a new camera.");
        }
    }

    bool RaveCamera::_SetIntrinsics(std::ostream& out, std::istream& in)
    {
        float fx, fy, cx, cy, n, f;
        in >> fx >> fy >> cx >> cy >> n >> f;
        near = n;
        far = f;
        SetIntrinsics(fx, fy, cx, cy);
        return true;
    }

    bool RaveCamera::_SetSize(std::ostream& out, std::istream& in)
    {
        int w, h;
        in >> w >> h;
        SetSize(w, h);
        return true;
    }

    void RaveCamera::AddKinBody(const std::string& name, float r, float g, float b)
    {
        OpenRAVE::KinBodyPtr body = GetEnv()->GetKinBody(name);

        if(body.get())
        {
            printf("Adding body %s with color %f %f %f\n", name.c_str(), r, g, b);
            bridge.CreateModels(*renderer.colorShader, body, OpenRAVE::Vector(r / 255.0f, g / 255.0f, b / 255.0f, 1.0));
        }
        else
        {
            RAVELOG_ERROR("Couldn't find body with name %s\n", name.c_str());
        }
    }

    bool RaveCamera::_AddKinBody(std::ostream& out, std::istream& in)
    {
        std::string name;
        float r, g, b;
        in >> name >> r >> g >> b;
        AddKinBody(name, r, g, b);
        return true;
    }

    bool RaveCamera::Initialize()
    {
       isInitialized = true;

       if (!glfwInit())
       {
           RAVELOG_ERROR("Failed to initialize OpenGL");
           return false;
       }

       window = glfwCreateWindow(640, 480, "Offscreen Window", NULL, NULL);

       if(!window)
        {
            glfwTerminate();
            RAVELOG_ERROR("Failed to create offscreen window");
            return false;
        }
       glfwMakeContextCurrent(window);
       glfwSwapInterval(0);
       //glfwHideWindow(window);
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

        if (!GLEW_ARB_framebuffer_object)
        {
            RAVELOG_ERROR("Glew does not support framebuffer\n");
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

        RAVELOG_INFO("Done loading shaders.\n");

        renderer.depthShader = &depthShader;
        renderer.colorShader = &colorShader;

        RAVELOG_INFO("Initializing\n");
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

        if (isInitialized)
        {
            RAVELOG_WARN("Can't set parameters after the camera is initialized. Please create a new camera.");
        }
    }

    OpenRAVE::SensorBase::SensorGeometryPtr RaveCamera::GetSensorGeometry(SensorType type)
    {
        if (type != OpenRAVE::SensorBase::ST_Camera && type != OpenRAVE::SensorBase::ST_Invalid)
        {
            RAVELOG_ERROR("Only camera sensor geometry is valid.\n");
            return OpenRAVE::SensorBase::SensorGeometryPtr();
        }
        return geometry;
    }

    OpenRAVE::SensorBase::SensorDataPtr RaveCamera::CreateSensorData(OpenRAVE::SensorBase::SensorType type)
    {
        if (type != OpenRAVE::SensorBase::ST_Camera && type != OpenRAVE::SensorBase::ST_Invalid)
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
        if(psensordata->GetType() != OpenRAVE::SensorBase::ST_Camera && psensordata->GetType() != OpenRAVE::SensorBase::ST_Invalid)
        {
            RAVELOG_ERROR("Only camera sensor is valid!\n");
            return false;
        }
        else
        {
            CameraSensorData* cameraSensor = dynamic_cast<CameraSensorData*>(psensordata.get());

            if(!cameraSensor)
            {
                RAVELOG_ERROR("Unable to cast to CameraSensorData.\n");
                return false;
            }

            size_t renderSize = renderer.colorBuffer.data.size();
            size_t imageSize = cameraSensor->vimagedata.size();

            if(renderSize != imageSize)
            {
                RAVELOG_ERROR("Expected render size of %lu, but got %lu. Are the resolutions setup up correctly?\n", imageSize, renderSize);
                return false;
            }

            std::memcpy(cameraSensor->vimagedata.data(), renderer.colorBuffer.data.data(), renderSize * sizeof(uint8_t));
            return true;
        }

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
        if (isInitialized && isRunning && geomData->width > 0 && geomData->height > 0)
        {
            glEnable(GL_DEPTH_TEST);
            glDepthFunc(GL_LESS);
            glFrontFace(GL_CCW);
            glEnable(GL_CULL_FACE);
            glCullFace(GL_BACK);
            glfwMakeContextCurrent(window);
            glfwPollEvents();
            glViewport(0, 0, 640, 480);
            renderer.projectionMatrix = GetPerspectiveMatrix(geomData->KK.fx, geomData->KK.fy, geomData->KK.cx, geomData->KK.cy, near, far, geomData->width, geomData->height);
            renderer.viewMatrix = GetViewMatrix(ORToTransform(transform));

            renderer.models.clear();
            bridge.UpdateModels();
            bridge.GetAllModels(renderer.models);
            renderer.Draw();
            glfwSwapBuffers(window);
            return true;
        }
        else if(isInitialized && isRunning && (geomData->width == 0 || geomData->height == 0))
        {
            RAVELOG_ERROR("Tried simulating camera that has no size.");
        }

        return true;
    }

}
