#pragma once

#include <openrave/openrave.h>

namespace raveurdf_utils {

inline OpenRAVE::Vector urdfRotationToRaveVector(const urdf::Rotation& rotation)
{
    return OpenRAVE::Vector(rotation.w, rotation.x, rotation.y, rotation.z);
}

inline OpenRAVE::Vector urdfVectorToRaveVector(const urdf::Vector3& vector)
{
    return OpenRAVE::Vector(vector.x, vector.y, vector.z);
}

inline OpenRAVE::Transform urdfPoseToRaveTransform(const urdf::Pose& pose)
{
    return OpenRAVE::Transform(urdfRotationToRaveVector(pose.rotation), urdfVectorToRaveVector(pose.position));
}

}  // namespace raveurdf_utils
