/*
 * RaveCamera.cpp
 *
 *  Created on: Apr 6, 2015
 *      Author: mklingen
 */

#include "RaveCamera.h"

namespace offscreen_render
{
    RaveCamera::RaveCamera()
    {
        // TODO Auto-generated constructor stub

    }

    RaveCamera::~RaveCamera()
    {
        // TODO Auto-generated destructor stub
    }

    int RaveCamera::Configure(OpenRAVE::SensorBase::ConfigureCommand, bool blocking)
    {

    }

    OpenRAVE::SensorBase::SensorGeometryPtr RaveCamera::GetSensorGeometry(SensorType type)
    {

    }

    OpenRAVE::SensorBase::SensorDataPtr RaveCamera::CreateSensorData(OpenRAVE::SensorBase::SensorType type)
    {
        if (type != OpenRAVE::SensorBase::ST_Camera)
        {
            RAVELOG_ERROR("Only camera sensor geometry is valid.\n");
            return OpenRAVE::SensorBase::SensorDataPtr();
        }
        //TODO: Implement
        return OpenRAVE::SensorBase::SensorDataPtr();
    }

    bool RaveCamera::GetSensorData(OpenRAVE::SensorBase::SensorDataPtr psensordata)
    {

    }

    bool RaveCamera::Supports(OpenRAVE::SensorBase::SensorType type)
    {

    }

    void RaveCamera::SetTransform(OpenRAVE::Transform const &trans)
    {

    }

    OpenRAVE::Transform RaveCamera::GetTransform()
    {

    }

    bool RaveCamera::SimulationStep(OpenRAVE::dReal fTimeElapsed)
    {

    }

}
