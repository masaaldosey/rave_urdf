#pragma once

#include <urdf_parser/urdf_parser.h>
#include <urdf/model.h>
#include <tinyxml2.h>

#include <openrave/openrave.h>
#include <openrave/plugin.h>
#include <boost/bind/bind.hpp>

#include <memory>
#include <string>

namespace raveurdf {

// TODO: update docstrings
class RaveURDF : public OpenRAVE::ModuleBase
{
public:
    RaveURDF(OpenRAVE::EnvironmentBasePtr penv);
    virtual ~RaveURDF() = default;

    bool loadURDF(std::ostream& soutput, std::istream& sinput);
    std::string loadModel(urdf::Model& urdf_model, TiXmlDocument& xml_doc, std::string path_to_urdf);

    void parseURDF(urdf::Model& model,
                   std::vector<OpenRAVE::KinBody::LinkInfoPtr>& link_infos,
                   std::vector<OpenRAVE::KinBody::JointInfoPtr>& joint_infos);

private:
    // reference to OpenRAVE environment
    OpenRAVE::EnvironmentBasePtr _env;
};
}  // namespace raveurdf
