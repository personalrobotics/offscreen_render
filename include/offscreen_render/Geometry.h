#ifndef GEOMETRY_H_
#define GEOMETRY_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

namespace offscreen_render
{
    typedef Eigen::Vector2f Vec2;
    typedef Eigen::Vector3f Vec3;
    typedef Eigen::Vector4f Vec4;
    typedef Eigen::Matrix3f Mat3x3;
    typedef Eigen::Matrix4f Mat4x4;
    typedef Eigen::Affine3f Transform;
    typedef Eigen::Quaternionf Quaternion;

    typedef std::vector<Vec2, Eigen::aligned_allocator<Vec2> > Vec2List;
    typedef std::vector<Vec3, Eigen::aligned_allocator<Vec3> > Vec3List;
    typedef std::vector<Vec4, Eigen::aligned_allocator<Vec4> > Vec4List;
    typedef std::vector<Mat3x3, Eigen::aligned_allocator<Mat3x3> > Mat3x3List;
    typedef std::vector<Mat4x4, Eigen::aligned_allocator<Mat4x4> > Mat4List;
    typedef std::vector<Transform, Eigen::aligned_allocator<Transform> > TransformList;
    typedef std::vector<Quaternion, Eigen::aligned_allocator<Quaternion> > QuaternionList;


    inline Mat4x4 GetPerspectiveMatrix(
            float fx, float fy,
            float cx, float cy,
            float near, float far,
            float width, float height)
    {
        Mat4x4 Result = Mat4x4::Zero();
        Result(0, 0) =  2. * fx / width;
        Result(1, 1) =  2. * fy / height;
        //Result(2, 1) =  2. * (0.5f - cx / width);
        //Result(2, 0) =  2. * (cy / height - 0.5f);
        Result(2, 2) = - (far + near) / (far - near);
        Result(3, 2) = - 1.0;
        Result(2, 3) = - (2.0 * far * near) / (far - near);

        return Result;
    }

    inline Mat4x4 GetViewMatrix(const Transform& transform)
    {
        Vec3 d = transform.linear().block(0, 2, 3, 1);
        Vec3 z = -d / d.norm();
        Vec3 up = -transform.linear().block(0, 1, 3, 1);
        Vec3 x = d.cross(up);
        x.normalize();
        Vec3 y = z.cross(x);
        Vec3 t = transform.translation();
        Mat4x4 tf = Mat4x4::Identity();
        tf.block(0, 0, 3, 1) = x;
        tf.block(0, 1, 3, 1) = y;
        tf.block(0, 2, 3, 1) = z;
        tf.block(0, 3, 3, 1) = t;
        Mat4x4 view =  tf.inverse().eval();
        return view;

        //return transform.matrix();
        /*
        Transform t;
  	    Quaternion q = Quaternion(transform.rotation()).conjugate();
        t.linear() = q.toRotationMatrix();
        t.translation() = -(t.linear() * transform.translation());
        */

        /*
        t.block(0, 3, 3, 1) = -transform.translation();
        t = t.transpose().eval();
        t.block(0, 0, 3, 3) = transform.linear();
        t.block(0, 2, 3, 1) *= -1;
        t.block(0, 1, 3, 1) *= -1;
        */
        //return t.matrix();
    }

    inline Mat4x4 GetFrustumMatrix(float left, float right,
            float bottom, float top, float near, float far)
    {
        Mat4x4 M = Mat4x4::Zero();
        M(0, 0) = 2.0f * near / (right - left);
        M(2, 0) = (right + left) / (right - left);
        M(1, 1) = 2.0f * near / (top - bottom);
        M(3, 1) =(top + bottom) / (top - bottom);
        M(2, 2) = -(far + near) / (far - near);
        M(3, 2) = -2.0 * near * far / (far - near);
        M(2, 3) = -1.0;
        return M;
    }

    inline Mat4x4 GetPerspectiveFOV(float fovYDegrees, float aspect, float zFar, float zNear)
    {
        float rad = fovYDegrees / 360.0f * M_PI;

        float theta = rad*0.5;
        float range = zFar - zNear;
        float invtan = 1./tan(theta);

        Mat4x4 p = Mat4x4::Zero();
        p(0,0) = invtan / aspect;
        p(1,1) = invtan;
        p(2,2) = -(zNear + zFar) / range;
        p(3,2) = -1;
        p(2,3) = -2 * zNear * zFar / range;
        p(3,3) = 0;

        return p;
    }

    inline Mat4x4 GetLookAt(Vec3 eye, Vec3 center, Vec3 up)
    {
        Vec3  f = (center - eye).normalized();
        Vec3  u = up.normalized();
        Vec3  s = f.cross(u).normalized();
        u = s.cross(f);

        Mat4x4 Result = Mat4x4::Identity();
        Result(0,0) = s.x();
        Result(1,0) = s.y();
        Result(2,0) = s.z();
        Result(0,1) = u.x();
        Result(1,1) = u.y();
        Result(2,1) = u.z();
        Result(0,2) =-f.x();
        Result(1,2) =-f.y();
        Result(2,2) =-f.z();
        Result(3,0) =-s.dot(eye);
        Result(3,1) =-u.dot(eye);
        Result(3,2) = f.dot(eye);
        return Result;
    }
}



#endif // GEOMETRY_H_
