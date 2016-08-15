#pragma once

#include <osg/Quat>
#include <osg/Vec3>


namespace util
{
    inline osg::Quat trRotationToQuat(const osg::Vec3& rotation)
    {
        osg::Quat v;
        v = osg::Quat(osg::DegreesToRadians(rotation.z()), osg::Vec3{0,0,1}) * v;
        v = osg::Quat(osg::DegreesToRadians(rotation.x()), osg::Vec3{1,0,0}) * v;
        v = osg::Quat(osg::DegreesToRadians(rotation.y()), osg::Vec3{0,1,0}) * v;
        return v;
    }


    constexpr float auToDeg(int16_t au)
    {
        return au / 65536.0f * 360;
    }
} // namespace util
