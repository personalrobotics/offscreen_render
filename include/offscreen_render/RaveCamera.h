/*
 * RaveCamera.h
 *
 *  Created on: Apr 6, 2015
 *      Author: mklingen
 */

#ifndef RAVECAMERA_H_
#define RAVECAMERA_H_

#include <openrave/sensor.h>

namespace offscreen_render
{

    class RaveCamera : OpenRAVE::SensorBase
    {
        public:
            RaveCamera();
            virtual ~RaveCamera();

            virtual int Configure(OpenRAVE::SensorBase::ConfigureCommand, bool blocking = false);
            virtual OpenRAVE::SensorBase::SensorGeometryPtr GetSensorGeometry(SensorType type = ST_Invalid);
            virtual SensorDataPtr CreateSensorData(OpenRAVE::SensorBase::SensorType type = ST_Invalid);
            virtual bool GetSensorData(OpenRAVE::SensorBase::SensorDataPtr psensordata);
            virtual bool Supports(OpenRAVE::SensorBase::SensorType type);
            virtual void SetTransform(OpenRAVE::Transform const &trans);
            virtual OpenRAVE::Transform GetTransform();
            virtual bool SimulationStep(OpenRAVE::dReal fTimeElapsed);

    };

}
#endif /* RAVECAMERA_H_ */
