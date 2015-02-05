#ifndef CONVERSIONS_H_
#define CONVERSIONS_H_

#include "Geometry.h"
#include <tf/tf.h>
#include <openrave/openrave.h>

namespace offscreen_render
{
    inline Transform TFToTransform(const tf::StampedTransform& stamped)
    {
        Transform toReturn;
        Mat3x3 rotMatrix;
        tf::Matrix3x3 rotation = stamped.getBasis();
        tf::Vector3 translation = stamped.getOrigin();
        rotMatrix << rotation[0][0], rotation[0][1], rotation[0][2],
                    rotation[1][0], rotation[1][1], rotation[1][2],
                    rotation[2][0], rotation[2][1], rotation[2][2];
        toReturn.linear() = rotMatrix;
        toReturn.translation() = Vec3(translation.x(), translation.y(), translation.z());
        return toReturn;
    }

    inline Vec3 ORToVec3(const OpenRAVE::Vector& orVec)
    {
        return Vec3(orVec.x, orVec.y, orVec.z);
    }

    inline Vec4 ORToVec4(const OpenRAVE::Vector& orVec)
    {
        return Vec4(orVec.x, orVec.y, orVec.z, orVec.w);
    }

    inline OpenRAVE::Vector QuaternionToOR(const Quaternion& quat)
    {
        return OpenRAVE::Vector(quat.w(), quat.x(), quat.y(), quat.z());
    }

    inline Quaternion ORToQuaternion(const OpenRAVE::Vector& orVec)
    {
        Quaternion q;
        q.x() = orVec.y;
        q.y() = orVec.z;
        q.z() = orVec.w;
        q.w() = orVec.x;
        return q;
    }

    inline Transform ORToTransform(const OpenRAVE::Transform& transform)
    {
        Transform toReturn;
        toReturn.translation() = ORToVec3(transform.trans);
        toReturn.linear() = ORToQuaternion(transform.rot).toRotationMatrix();
        return toReturn;
    }

    inline OpenRAVE::Transform TransformToOR(const Transform& transform)
    {
        OpenRAVE::Transform  toReturn;
        toReturn.trans = OpenRAVE::Vector(transform.translation().x(), transform.translation().y(), transform.translation().z());
        toReturn.rot = QuaternionToOR(Quaternion(transform.linear()));
        return toReturn;
    }
}



#endif // CONVERSIONS_H_ 
