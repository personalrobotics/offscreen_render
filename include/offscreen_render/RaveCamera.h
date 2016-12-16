/*
 * RaveCamera.h
 *
 *  Created on: Apr 6, 2015
 *      Author: mklingen
 */

#ifndef RAVECAMERA_H_
#define RAVECAMERA_H_

#include <openrave/sensor.h>
#include <offscreen_render/RaveBridge.h>
#include <offscreen_render/OffscreenRenderer.h>
#include <GLFW/glfw3.h>

namespace offscreen_render
{
    class RaveCamera : public OpenRAVE::SensorBase
    {
        public:
            RaveCamera(OpenRAVE::EnvironmentBasePtr env);
            virtual ~RaveCamera();

            // Sets the intrinsic parameters of the camera
            virtual void SetIntrinsics(float fx, float fy, float cx, float cy);
            // Sets the width and height of the camera in pixels
            virtual void SetSize(int w, int h);
            // Add a kinbody with the following color.
            virtual void AddKinBody(const std::string& name, float r, float g, float b);
            // Remove a kinbody with the given name
            virtual void RemoveKinBody(const std::string& name);
            // Remove all kinbodies
            virtual void ClearBodies();
            // Initialize the graphics engine
            virtual bool Initialize();
            // Gets the models associated with the given kinbody. Returns false if kinbody doesn't exist.
            virtual bool GetModels(const std::string& name, std::vector<size_t>& models);
            // Gets the mesh model associated with the given kinbody, link and geometry index.
            // Returns false if no such model exists.
            virtual bool GetModel(const std::string& name, int link, size_t* modelIdx);

            // Interface Commands
            bool _SetIntrinsics(std::ostream& out, std::istream& in);
            bool _SetSize(std::ostream& out, std::istream& in);
            bool _AddKinBody(std::ostream& out, std::istream& in);
            bool _RemoveKinBody(std::ostream& out, std::istream& in);
            bool _ClearBodies(std::ostream& out, std::istream& in);
            bool _GetKinbodyLinkMeshIDs(std::ostream& out, std::istream& in);
            bool _GetNumLinkGeometries(std::ostream& out, std::istream& in);
            bool _GetLinkMeshPositions(std::ostream& out, std::istream& in);
            bool _GetLinkMeshColors(std::ostream& out, std::istream& in);
            bool _GetLinkMeshIndices(std::ostream& out, std::istream& in);
            bool _SetLinkMeshPositions(std::ostream& out, std::istream& in);
            bool _SetLinkMeshColors(std::ostream& out, std::istream& in);
            bool _SetLinkMeshIndices(std::ostream& out, std::istream& in);

            // Overrides SensorBase
            virtual int Configure(OpenRAVE::SensorBase::ConfigureCommand, bool blocking = false);
            virtual OpenRAVE::SensorBase::SensorGeometryPtr GetSensorGeometry(SensorType type = ST_Invalid);
            virtual SensorDataPtr CreateSensorData(OpenRAVE::SensorBase::SensorType type = ST_Invalid);
            virtual bool GetSensorData(OpenRAVE::SensorBase::SensorDataPtr psensordata);
            virtual bool Supports(OpenRAVE::SensorBase::SensorType type);
            virtual void SetTransform(OpenRAVE::Transform const &trans);
            virtual OpenRAVE::Transform GetTransform();
            virtual bool SimulationStep(OpenRAVE::dReal fTimeElapsed);

        protected:
            template<typename T> void OutputBuffer(std::ostream& out, const std::vector<T>& buffer)
            {
                for(size_t i = 0; i < buffer.size(); i++)
                {
                    out << buffer[i];
                    if (i < buffer.size() - 1)
                    {
                        out << " ";
                    }
                }
            }

            template<typename T> void InputBuffer(std::istream& in, std::vector<T>& buffer)
            {
                while (in.good())
                {
                    T f;
                    in >> f;
                    buffer.push_back(f);
                }
            }

            OpenRAVE::SensorBase::CameraGeomData* geomData;
            OpenRAVE::SensorBase::SensorGeometryPtr geometry;
            OpenRAVE::Transform transform;
            bool isRunning;
            bool isInitialized;
            OffscreenRenderer renderer;
            RaveBridge bridge;
            GLFWwindow* window;
            Shader depthShader;
            Shader colorShader;
            float near;
            float far;
    };
}
#endif /* RAVECAMERA_H_ */
