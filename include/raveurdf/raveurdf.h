#pragma once

#include <urdf/model.h>
#include <tinyxml.h>

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

    bool loadURDFFile(std::ostream& soutput, std::istream& sinput);
    std::string loadRobotModel(urdf::Model& urdf_model, TiXmlDocument& xml_doc, std::string path_to_urdf);

    void parseURDF(urdf::Model& model,
                   std::vector<OpenRAVE::KinBody::LinkInfoPtr>& link_infos,
                   std::vector<OpenRAVE::KinBody::JointInfoPtr>& joint_infos);
    void processGeometryGroupTagsFromURDF(TiXmlDocument& xml_doc,
                                          std::vector<OpenRAVE::KinBody::LinkInfoPtr>& link_infos);

    int main(const std::string& cmd)
    {
        RAVELOG_INFO("URDF loader initialized with command: %s\n", cmd.c_str());
        return 0;
    }
};

class RaveURDFPlugin : public RavePlugin
{
public:
    OpenRAVE::InterfaceBasePtr CreateInterface(OpenRAVE::InterfaceType type,
                                               const std::string& interfacename,
                                               std::istream& sinput,
                                               OpenRAVE::EnvironmentBasePtr penv) override
    {
        if (type == OpenRAVE::PT_Module && interfacename == "raveurdf") {
            return OpenRAVE::InterfaceBasePtr(new RaveURDF(penv));
        }

        return OpenRAVE::InterfaceBasePtr();
    }

    const InterfaceMap& GetInterfaces() const override
    {
        static const RavePlugin::InterfaceMap interfaces = {
            {OpenRAVE::PT_Module, {"RaveURDF"}},
        };

        return interfaces;
    }
    const std::string& GetPluginName() const override
    {
        static const std::string pluginName = "RaveURDF";
        return pluginName;
    }
};
OPENRAVE_PLUGIN_API RavePlugin* CreatePlugin()
{
    return new RaveURDFPlugin();
}
OPENRAVE_PLUGIN_API void DestroyPlugin(RavePlugin* p)
{
    delete p;
}
}  // namespace raveurdf
