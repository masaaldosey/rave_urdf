#pragma once

#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>
#include <openrave/openrave.h>

namespace raveurdf_utils {

/**
 * @brief converts a URDF rotation (quaternion) into an OpenRAVE vector.
 *
 * @param rotation the URDF rotation, represented by a quaternion (w, x, y, z).
 * @return an OpenRAVE::Vector representing the rotation in quaternion format.
 */
inline OpenRAVE::Vector urdfRotationToRaveVector(const urdf::Rotation& rotation)
{
    return OpenRAVE::Vector(rotation.w, rotation.x, rotation.y, rotation.z);
}

/**
 * @brief converts a URDF vector (3D vector) into an OpenRAVE vector.
 *
 * @param vector the URDF vector, represented by its x, y, and z components.
 * @return an OpenRAVE::Vector with the corresponding x, y, and z values.
 */
inline OpenRAVE::Vector urdfVectorToRaveVector(const urdf::Vector3& vector)
{
    return OpenRAVE::Vector(vector.x, vector.y, vector.z);
}

/**
 * @brief converts a URDF pose (position and orientation) into an OpenRAVE transform.
 *
 * @param pose the URDF pose, containing both position (vector) and rotation (quaternion).
 * @return an OpenRAVE::Transform combining the position and rotation.
 */
inline OpenRAVE::Transform urdfPoseToRaveTransform(const urdf::Pose& pose)
{
    return OpenRAVE::Transform(urdfRotationToRaveVector(pose.rotation), urdfVectorToRaveVector(pose.position));
}

/**
 * @brief converts a URDF color into an OpenRAVE vector.
 *
 * @param color the URDF color, represented by r, g, b, and a (alpha) values.
 * @return an OpenRAVE::Vector with r, g, b, and a components.
 */
inline OpenRAVE::Vector urdfColorToRaveVector(const urdf::Color& color)
{
    return OpenRAVE::Vector(color.r, color.g, color.b, color.a);
}

/**
 * @brief converts a URDF joint type to an OpenRAVE joint type and returns whether it is a freely moving joint.
 *
 * @param type the URDF joint type, represented as an integer.
 * @return a pair containing the corresponding OpenRAVE joint type and a boolean indicating if it is freely moving.
 * @throws std::runtime_error if the joint type is not supported or unknown.
 */
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

/**
 * @brief converts a vector of shared pointers to a constant version of the same type.
 *
 * @tparam T the type of the objects held by the shared pointers.
 * @param vconst a vector of non-constant shared pointers.
 * @return a vector of constant shared pointers of the same type.
 */
template <class T>
inline std::vector<boost::shared_ptr<T const> > convertToConstantSharedPointers(
    std::vector<boost::shared_ptr<T> > const& vconst)
{
    std::vector<boost::shared_ptr<T const> > v;
    v.reserve(vconst.size());

    BOOST_FOREACH (boost::shared_ptr<T> const& x, vconst) {
        v.push_back(x);
    }
    return v;
}

}  // namespace raveurdf_utils
