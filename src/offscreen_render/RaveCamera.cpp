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

#define CMD(name, fn, comment) RegisterCommand(name, boost::bind(&RaveCamera::fn, this, _1, _2), comment)

namespace offscreen_render
{
    RaveCamera::RaveCamera(OpenRAVE::EnvironmentBasePtr env) :
            OpenRAVE::SensorBase(env), isRunning(false), isInitialized(false), window(0x0)
    {
        geomData = new OpenRAVE::SensorBase::CameraGeomData();
        geometry.reset(geomData);
        near = 0.01f;
        far = 10.0f;

        CMD("setintrinsic", _SetIntrinsics,
                "<fx,fy,cx,cy,near,far>. Set the intrinsic parameters of the camera.");
        CMD("setdims", _SetSize,
                "<width,height>. Set the dimensions of the image.");
        CMD("addbody", _AddKinBody,
                "<name, r, g, b>. Add a kinbody to the render scene with the given 24 bit color.");
        CMD("removebody", _RemoveKinBody,
                "<name>. Remove a kinbody.");
        CMD("clearbodies", _ClearBodies,
                "Clear all kinbodies.");
        CMD("get_kinbody_link_mesh_ids", _GetKinbodyLinkMeshIDs,
                "<name> -> (ids) Gets all of the link IDs of kinbody.");
        CMD("get_num_link_geometries", _GetNumLinkGeometries,
                "<name, link id> -> (num geometries) Gets the number of geometries of a kinbody link.");
        CMD("get_link_mesh_positions", _GetLinkMeshPositions,
                "<name, link id, geometry num> -> (x, y, z positions...) Gets the vertex positions associated with a link geometry.");
        CMD("get_link_mesh_colors", _GetLinkMeshColors,
                "<name, link id, geometry num> -> (r, g, b colors...) Gets the vertex colors associated with a link geometry.");
        CMD("get_link_mesh_indices", _GetLinkMeshIndices,
                "<name, link id, geometry num> -> (indicies...) Gets the triangle indices associated with a link geometry.");
        CMD("set_link_mesh_positions", _SetLinkMeshPositions,
                "<name, link id, geometry num, x, y, z positions...> Sets the vertex positions associated with a link geometry.");
        CMD("set_link_mesh_colors", _SetLinkMeshColors,
                "<name, link id, geometry num, r, g, b colors...> Sets the vertex colors associated with a link geometry.");
        CMD("set_link_mesh_indices", _SetLinkMeshIndices, ""
                "<name, link id, geometry num, indicies...> Sets the triangle indices associated with a link geometry.");
    }

    RaveCamera::~RaveCamera()
    {
        if (isInitialized)
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

        if (body.get())
        {
            bridge.CreateModels(*renderer.colorShader, body, OpenRAVE::Vector(r / 255.0f, g / 255.0f, b / 255.0f, 1.0));
        }
        else
        {
            RAVELOG_ERROR("Couldn't find body with name %s\n", name.c_str());
        }
    }

    class RemoveBodyPredicate
    {
        public:
            bool predicate(OpenRAVE::KinBodyPtr body, const offscreen_render::RaveBridge::RaveModel& model)
            {
                return model.body == body;
            }

            bool operator() ( const offscreen_render::RaveBridge::RaveModel& model)
            {
                    return predicate(_body, model);
            }
            OpenRAVE::KinBodyPtr _body;

    };

    void RaveCamera::RemoveKinBody(const std::string& name)
    {
        OpenRAVE::KinBodyPtr body = GetEnv()->GetKinBody(name);
        RemoveBodyPredicate predicate;
        predicate._body = body;

        if (body.get())
        {

            std::vector<OpenRAVE::KinBodyPtr>::iterator bodies_end = std::remove(bridge.bodies.begin(), bridge.bodies.end(), body);
            bridge.bodies.erase(bodies_end);

            std::vector<RaveBridge::RaveModel>::iterator  end = std::remove_if
            (
                    bridge.models.begin(), bridge.models.end(),
                    predicate
            );

            bridge.models.erase(end);
        }
        else
        {
            RAVELOG_ERROR("Couldn't find body with name %s\n", name.c_str());
        }
    }

    void RaveCamera::ClearBodies()
    {
        bridge.models.clear();
        bridge.bodies.clear();
    }

    // Gets the models associated with the given kinbody. Returns false if kinbody doesn't exist.
    bool RaveCamera::GetModels(const std::string& name, std::vector<size_t>& models)
    {
        models.clear();

        for (size_t i = 0; i < bridge.models.size(); i++)
        {
            RaveBridge::RaveModel& model = bridge.models.at(i);

            if (model.body->GetName() == name)
            {
                models.push_back(i);
            }
        }
        return !models.empty();
    }

    // Gets the mesh model associated with the given kinbody and link. Returns false if no such model exists.
    bool RaveCamera::GetModel(const std::string& name, int link, size_t* model)
    {
        if (!model)
        {
            RAVELOG_ERROR("Nulll pointer passed to RaveCamera::GetModel\n");
            return false;
        }

        for (size_t i = 0; i < bridge.models.size(); i++)
        {
            if (bridge.models.at(i).body->GetName() == name &&
                bridge.models.at(i).link->GetIndex() == link)
            {
                *model = i;
                return true;
            }
        }
        RAVELOG_ERROR("No mesh exists for kinbody %s, link %d", name.c_str(), link);
        return false;
    }

    bool RaveCamera::_GetKinbodyLinkMeshIDs(std::ostream& out, std::istream& in)
    {
        std::string name;
        in >> name;

        std::vector<size_t> models;
        if (!GetModels(name, models))
        {
            return false;
        }

        for (size_t i = 0; i < models.size(); i++)
        {
            out << bridge.models.at(models[i]).link->GetIndex() << " ";
        }
        return true;
    }

    bool RaveCamera::_GetNumLinkGeometries(std::ostream& out, std::istream& in)
    {
        std::string name;
        int link;
        in >> name;
        in >> link;

        size_t idx = 0;
        if (!GetModel(name, link, &idx))
        {
            return false;
        }
        out << bridge.models.at(idx).models.size();
        return true;
    }

    bool RaveCamera::_GetLinkMeshPositions(std::ostream& out, std::istream& in)
    {
        std::string name = "";
        int link = 0;
        int geomIdx = 0;
        in >> name;
        in >> link;
        in >> geomIdx;

        size_t idx = 0;
        if (!GetModel(name, link, &idx))
        {
            RAVELOG_ERROR("Coldn't find kinbody %s, link %d", name.c_str(), link);
            return false;
        }

        if (geomIdx < bridge.models.at(idx).models.size() && geomIdx >= 0)
        {
            OutputBuffer(out, bridge.models.at(idx).models.at(geomIdx).buffer->position_data);
        }
        else
        {
            RAVELOG_ERROR("Couldn't find geometry %d. Kinbody %s, link %d has %lu geometries\n",
                    geomIdx, name.c_str(), link, bridge.models.at(idx).models.size());
            return false;
        }
        return true;
    }

    bool RaveCamera::_GetLinkMeshColors(std::ostream& out, std::istream& in)
    {
        std::string name;
        int link;
        int geomIdx;
        in >> name;
        in >> link;
        in >> geomIdx;

        size_t idx = 0;
        if (!GetModel(name, link, &idx))
        {
            RAVELOG_ERROR("Coldn't find kinbody %s, link %d", name.c_str(), link);
            return false;
        }

        if (geomIdx < bridge.models.at(idx).models.size() && geomIdx >= 0)
        {
            OutputBuffer(out,  bridge.models.at(idx).models.at(geomIdx).buffer->color_data);
        }
        else
        {
            RAVELOG_ERROR("Couldn't find geometry %d. Kinbody %s, link %d has %lu geometries", name.c_str(), link, bridge.models.at(idx).models.size());
            return false;
        }
        return true;
    }

    bool RaveCamera::_GetLinkMeshIndices(std::ostream& out, std::istream& in)
    {
        std::string name;
        int link;
        int geomIdx;
        in >> name;
        in >> link;
        in >> geomIdx;

        size_t idx = 0;
        if (!GetModel(name, link, &idx))
        {
            RAVELOG_ERROR("Coldn't find kinbody %s, link %d", name.c_str(), link);
            return false;
        }

        if (geomIdx < bridge.models.at(idx).models.size() && geomIdx >= 0)
        {
            OutputBuffer(out, bridge.models.at(idx).models.at(geomIdx).buffer->index_data);
        }
        else
        {
            RAVELOG_ERROR("Couldn't find geometry %d. Kinbody %s, link %d has %lu geometries", name.c_str(), link, bridge.models.at(idx).models.size());
            return false;
        }
        return true;
    }


    bool RaveCamera::_SetLinkMeshPositions(std::ostream& out, std::istream& in)
    {
        std::string name;
        int link;
        int geomIdx;
        in >> name;
        in >> link;
        in >> geomIdx;

        size_t idx = 0;
        if (!GetModel(name, link, &idx))
        {
            RAVELOG_ERROR("Coldn't find kinbody %s, link %d", name.c_str(), link);
            return false;
        }

        if (geomIdx < bridge.models.at(idx).models.size() && geomIdx >= 0)
        {
            const Model& model = bridge.models.at(idx).models.at(geomIdx);

            std::vector<GLfloat> newPositions;
            InputBuffer(in, newPositions);

            if (newPositions.size() != model.buffer->position_data.size())
            {
                RAVELOG_WARN("Old model (kinbody %s, link %d, geometry %d) has %lu positions. "
                             "Overwriting with %lu new positions.", name.c_str(), link, geomIdx,
                             model.buffer->position_data.size(),
                             newPositions.size());
            }

            model.buffer->position_data = newPositions;
        }
        else
        {
            RAVELOG_ERROR("Couldn't find geometry %d. Kinbody %s, link %d has %lu geometries", name.c_str(), link, bridge.models.at(idx).models.size());
            return false;
        }
        return true;
    }

    bool RaveCamera::_SetLinkMeshColors(std::ostream& out, std::istream& in)
    {
        std::string name;
        int link;
        int geomIdx;
        in >> name;
        in >> link;
        in >> geomIdx;

        size_t idx = 0;
        if (!GetModel(name, link, &idx))
        {
            RAVELOG_ERROR("Coldn't find kinbody %s, link %d", name.c_str(), link);
            return false;
        }

        if (geomIdx < bridge.models.at(idx).models.size() && geomIdx >= 0)
        {
            const Model& model = bridge.models.at(idx).models.at(geomIdx);

            std::vector<GLfloat> newColors;
            InputBuffer(in, newColors);

            if (newColors.size() != model.buffer->color_data.size())
            {
                RAVELOG_WARN("Old model (kinbody %s, link %d, geometry %d) has %lu colors. "
                             "Overwriting with %lu new colors.", name.c_str(), link, geomIdx,
                             model.buffer->color_data.size(),
                             newColors.size());
            }

            model.buffer->color_data = newColors;
            model.buffer->Initialize(*renderer.colorShader);
        }
        else
        {
            RAVELOG_ERROR("Couldn't find geometry %d. Kinbody %s, link %d has %lu geometries", name.c_str(), link, bridge.models.at(idx).models.size());
            return false;
        }
        return true;
    }

    bool RaveCamera::_SetLinkMeshIndices(std::ostream& out, std::istream& in)
    {
        std::string name;
        int link;
        int geomIdx;
        in >> name;
        in >> link;
        in >> geomIdx;

        size_t idx = 0;
        if (!GetModel(name, link, &idx))
        {
            RAVELOG_ERROR("Coldn't find kinbody %s, link %d", name.c_str(), link);
            return false;
        }

        if (geomIdx < bridge.models.at(idx).models.size() && geomIdx >= 0)
        {
            const Model& model = bridge.models.at(idx).models.at(geomIdx);

            std::vector<GLushort> newIndicies;
            InputBuffer(in, newIndicies);

            if (newIndicies.size() != model.buffer->index_data.size())
            {
                RAVELOG_WARN("Old model (kinbody %s, link %d, geometry %d) has %lu indices. "
                             "Overwriting with %lu new colors.", name.c_str(), link, geomIdx,
                             model.buffer->index_data.size(),
                             newIndicies.size());
            }

            model.buffer->index_data = newIndicies;
        }
        else
        {
            RAVELOG_ERROR("Couldn't find geometry %d. Kinbody %s, link %d has %lu geometries", name.c_str(), link, bridge.models.at(idx).models.size());
            return false;
        }
        return true;
    }

    bool RaveCamera::_RemoveKinBody(std::ostream& out, std::istream& in)
    {
        std::string name;
        in >> name;
        RemoveKinBody(name);
        return true;
    }

    bool RaveCamera::_ClearBodies(std::ostream& out, std::istream& in)
    {
        ClearBodies();
        return true;
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
        if (isInitialized) return true;

        isInitialized = true;

        if (!glfwInit())
        {
            RAVELOG_ERROR("Failed to initialize OpenGL");
            return false;
        }

        window = glfwCreateWindow(1, 1, "Offscreen Window", NULL, NULL);

        if (!window)
        {
            glfwTerminate();
            RAVELOG_ERROR("Failed to create offscreen window");
            return false;
        }
        glfwHideWindow(window);
        glfwMakeContextCurrent(window);
        glfwSwapInterval(0);
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

        if (!depthShader.LoadFromFile(shaders + "FragmentShader.glsl", shaders + "VertexShader.glsl"))
        {
            RAVELOG_ERROR("Failed to load depth shaders.");
            return false;
        }

        if (!colorShader.LoadFromFile(shaders + "ColorFragmentShader.glsl", shaders + "VertexShader.glsl"))
        {
            RAVELOG_ERROR("Failed to load color shaders.");
            return false;
        }

        renderer.depthShader = &depthShader;
        renderer.colorShader = &colorShader;

        renderer.Initialize((int) geomData->width, (int) geomData->height);
        return true;

    }

    int RaveCamera::Configure(OpenRAVE::SensorBase::ConfigureCommand command, bool blocking)
    {
        switch (command)
        {
            case OpenRAVE::SensorBase::CC_PowerOn:
                isRunning = true;
                if (!isInitialized)
                {
                    if (Initialize())
                    {
                        return 0;
                    }
                    else
                        return -1;
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
        if (type != OpenRAVE::SensorBase::ST_Camera && type != OpenRAVE::SensorBase::ST_Camera && type != OpenRAVE::SensorBase::ST_Invalid)
        {
            RAVELOG_ERROR("Only camera sensor geometry is valid.\n");
            return OpenRAVE::SensorBase::SensorGeometryPtr();
        }
        return geometry;
    }

    OpenRAVE::SensorBase::SensorDataPtr RaveCamera::CreateSensorData(OpenRAVE::SensorBase::SensorType type)
    {
        if (type == OpenRAVE::SensorBase::ST_Camera || type == OpenRAVE::SensorBase::ST_Invalid)
        {
            OpenRAVE::SensorBase::CameraSensorData* data = new OpenRAVE::SensorBase::CameraSensorData();
            data->vimagedata.resize(geomData->width * geomData->height * 3);
            data->__trans = transform;
            return OpenRAVE::SensorBase::SensorDataPtr(data);
        }
        else if(type == OpenRAVE::SensorBase::ST_Laser)
        {
            OpenRAVE::SensorBase::LaserSensorData* data = new OpenRAVE::SensorBase::LaserSensorData();
            data->intensity.resize(geomData->width * geomData->height * 3);
            data->__trans = transform;
            return OpenRAVE::SensorBase::SensorDataPtr(data);
        }
        else
        {
            RAVELOG_ERROR("Only camera sensor geometry is valid.\n");
            return OpenRAVE::SensorBase::SensorDataPtr();
        }
    }

    bool RaveCamera::GetSensorData(OpenRAVE::SensorBase::SensorDataPtr psensordata)
    {
        if (psensordata->GetType() == OpenRAVE::SensorBase::ST_Camera || psensordata->GetType() == OpenRAVE::SensorBase::ST_Invalid)
        {
            CameraSensorData* cameraSensor = dynamic_cast<CameraSensorData*>(psensordata.get());

            if (!cameraSensor)
            {
                RAVELOG_ERROR("Unable to cast to CameraSensorData.\n");
                return false;
            }

            size_t renderSize = renderer.colorBuffer.data.size();
            size_t imageSize = cameraSensor->vimagedata.size();

            if (renderSize != imageSize)
            {
                RAVELOG_ERROR("Expected render size of %lu, but got %lu. Are the resolutions setup up correctly?\n", imageSize, renderSize);
                return false;
            }

            for (size_t i = 0; i < renderSize; i++)
            {
                cameraSensor->vimagedata[i] = static_cast<uint8_t>(renderer.colorBuffer.data.at(i) * 255.0f);
            }
            return true;
        }
        else if (psensordata->GetType() == OpenRAVE::SensorBase::ST_Laser)
        {
            LaserSensorData* depthSensor = dynamic_cast<LaserSensorData*>(psensordata.get());

            if (!depthSensor)
            {
                RAVELOG_ERROR("Unable to cast to CameraSensorData.\n");
                return false;
            }

            size_t renderSize = renderer.depthBuffer.data.size();
            size_t imageSize = depthSensor->intensity.size();

            if (renderSize != imageSize)
            {
                RAVELOG_ERROR("Expected render size of %lu, but got %lu. Are the resolutions setup up correctly?\n", imageSize, renderSize);
                return false;
            }

            for (size_t i = 0; i < renderSize; i++)
            {
                depthSensor->intensity[i] = static_cast<OpenRAVE::dReal>(renderer.depthBuffer.data.at(i));
            }
            return true;
        }
	else
        {
            RAVELOG_ERROR("Only camera sensor is valid!\n");
            return false;
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
        //while (!glfwWindowShouldClose(window))
        {
            if (isInitialized && isRunning && geomData->width > 0 && geomData->height > 0)
            {
                glfwMakeContextCurrent(window);
                glEnable(GL_DEPTH_TEST);
                glViewport(0, 0, geomData->width, geomData->height);
                glClearColor(0.0, 0.0, 0, 1.0);
                glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
                renderer.models.clear();
                bridge.UpdateModels();
                bridge.GetAllModels(renderer.models);
                renderer.projectionMatrix = GetPerspectiveMatrix(geomData->KK.fx, geomData->KK.fy, geomData->KK.cx, geomData->KK.cy, near, far, geomData->width, geomData->height);
                Transform mat = ORToTransform(transform);
                renderer.viewMatrix = GetViewMatrix(mat);
                renderer.Draw();
                glfwSwapBuffers(window);
                glfwPollEvents();
            }
            else if (isInitialized && isRunning && (geomData->width == 0 || geomData->height == 0))
            {
                RAVELOG_ERROR("Tried simulating camera that has no size.\n");
            }

        }

        return true;
    }

}
