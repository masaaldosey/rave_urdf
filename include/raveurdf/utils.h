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

inline OpenRAVE::Vector urdfColorToRaveVector(const urdf::Color& color)
{
    return OpenRAVE::Vector(color.r, color.g, color.b, color.a);
}

inline std::pair<OpenRAVE::KinBody::JointType, bool> urdfJointTypeToRaveJointType(int type)
{
    switch (type) {
        case urdf::Joint::REVOLUTE:
            return std::make_pair(OpenRAVE::KinBody::JointRevolute, true);

        case urdf::Joint::PRISMATIC:
            return std::make_pair(OpenRAVE::KinBody::JointSlider, true);

        case urdf::Joint::FIXED:
            return std::make_pair(OpenRAVE::KinBody::JointHinge, false);

        case urdf::Joint::CONTINUOUS:
            return std::make_pair(OpenRAVE::KinBody::JointHinge, true);

        // TODO: fill the rest of these joint types in!
        case urdf::Joint::PLANAR:
            throw std::runtime_error("Planar joints are not supported.");

        case urdf::Joint::FLOATING:
            throw std::runtime_error("Floating joints are not supported.");

        case urdf::Joint::UNKNOWN:
            throw std::runtime_error("Unknown joints are not supported.");

        default:
            throw std::runtime_error(boost::str(boost::format("Unkonwn type of joint %d.") % type));
    }
}

}  // namespace raveurdf_utils
