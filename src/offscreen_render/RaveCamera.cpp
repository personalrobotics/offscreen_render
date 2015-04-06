/*
 * RaveCamera.cpp
 *
 *  Created on: Apr 6, 2015
 *      Author: mklingen
 */

#include <openrave/openrave.h>
#include <offscreen_render/RaveCamera.h>

namespace offscreen_render
{
    RaveCamera::RaveCamera(OpenRAVE::EnvironmentBasePtr env) :
            OpenRAVE::SensorBase(env)
    {
        // TODO Auto-generated constructor stub

    }

    RaveCamera::~RaveCamera()
    {
        // TODO Auto-generated destructor stub
    }

    int RaveCamera::Configure(OpenRAVE::SensorBase::ConfigureCommand command, bool blocking)
    {
        //TODO: Implement
        return 0x0;
    }

    OpenRAVE::SensorBase::SensorGeometryPtr RaveCamera::GetSensorGeometry(SensorType type)
    {
        if (type != OpenRAVE::SensorBase::ST_Camera)
        {
            RAVELOG_ERROR("Only camera sensor geometry is valid.\n");
            return OpenRAVE::SensorBase::SensorGeometryPtr();
        }
        //TODO: Impement
        return OpenRAVE::SensorBase::SensorGeometryPtr();
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
        // TODO: Implement
        return false;
    }

    bool RaveCamera::Supports(OpenRAVE::SensorBase::SensorType type)
    {
        return type == OpenRAVE::SensorBase::ST_Camera;
    }

    void RaveCamera::SetTransform(OpenRAVE::Transform const &trans)
    {
        //TODO: Implement
    }

    OpenRAVE::Transform RaveCamera::GetTransform()
    {
        //TODO: Implement
        return OpenRAVE::Transform();
    }

    bool RaveCamera::SimulationStep(OpenRAVE::dReal fTimeElapsed)
    {
        //TODO: Implement
        return false;
    }

}
