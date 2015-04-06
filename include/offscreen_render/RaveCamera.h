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

            void SetIntrinsics(float fx, float fy, float cx, float cy);

            bool Initialize();

            virtual int Configure(OpenRAVE::SensorBase::ConfigureCommand, bool blocking = false);
            virtual OpenRAVE::SensorBase::SensorGeometryPtr GetSensorGeometry(SensorType type = ST_Invalid);
            virtual SensorDataPtr CreateSensorData(OpenRAVE::SensorBase::SensorType type = ST_Invalid);
            virtual bool GetSensorData(OpenRAVE::SensorBase::SensorDataPtr psensordata);
            virtual bool Supports(OpenRAVE::SensorBase::SensorType type);
            virtual void SetTransform(OpenRAVE::Transform const &trans);
            virtual OpenRAVE::Transform GetTransform();
            virtual bool SimulationStep(OpenRAVE::dReal fTimeElapsed);

        protected:
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
