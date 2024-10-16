#pragma once

#include <urdf/model.h>
#include <tinyxml.h>

#include <openrave/openrave.h>
#include <openrave/plugin.h>
#include <boost/bind/bind.hpp>

#include <memory>
#include <string>

namespace raveurdf {

class RaveURDF : public OpenRAVE::ModuleBase
{
public:
    /**
     * @brief constructs a RaveURDF module within a given OpenRAVE environment.
     *
     * @param penv the environment where this URDF module will be instantiated.
     */
    RaveURDF(OpenRAVE::EnvironmentBasePtr penv);

    /**
     * @brief destroys the RaveURDF instance and releases any allocated resources.
     */
    virtual ~RaveURDF() = default;

    /**
     * @brief loads a URDF file and parses its contents to create an OpenRAVE model.
     *
     * @param soutput the output stream where any log or error messages will be written.
     * @param sinput the input stream from which the URDF file data will be read.
     * @return true if the URDF file was successfully loaded; false otherwise.
     */
    bool loadURDFFile(std::ostream& soutput, std::istream& sinput);

    /**
     * @brief loads a robot model from a URDF document and converts it into OpenRAVE format.
     *
     * @param urdf_model the URDF model object to populate.
     * @param xml_doc the XML document containing the URDF data.
     * @param path_to_urdf the path to the URDF file, used to resolve relative paths.
     * @return the name of the loaded robot model as a string.
     */
    std::string loadRobotModel(urdf::Model& urdf_model, TiXmlDocument& xml_doc, std::string path_to_urdf);

    /**
     * @brief parses the URDF model to extract link and joint information, storing it in OpenRAVE structures.
     *
     * @param model the URDF model to be parsed.
     * @param link_infos the vector where link information will be stored.
     * @param joint_infos the vector where joint information will be stored.
     */
    void parseURDF(urdf::Model& model,
                   std::vector<OpenRAVE::KinBody::LinkInfoPtr>& link_infos,
                   std::vector<OpenRAVE::KinBody::JointInfoPtr>& joint_infos);

    /**
     * @brief main entry point for the RaveURDF module, handling specific commands.
     *
     * @param cmd the command string to be executed by the module.
     * @return an integer status code; 0 indicates success.
     */
    int main(const std::string& cmd)
    {
        RAVELOG_INFO("URDF loader initialized with command: %s\n", cmd.c_str());
        return 0;
    }
};

class RaveURDFPlugin : public RavePlugin
{
public:
    /**
     * @brief creates an interface for the plugin, based on the requested interface type and name.
     *
     * @param type the type of interface to be created (e.g., Module).
     * @param interfacename the name of the interface to create.
     * @param sinput the input stream for configuration or initialization data.
     * @param penv the OpenRAVE environment where the interface will be used.
     * @return a pointer to the created InterfaceBase object, or null if the type/name does not match.
     */
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

    /**
     * @brief returns a map of supported interface types and their names.
     *
     * @return a constant reference to the InterfaceMap containing the plugin's interfaces.
     */
    const InterfaceMap& GetInterfaces() const override
    {
        static const RavePlugin::InterfaceMap interfaces = {
            {OpenRAVE::PT_Module, {"RaveURDF"}},
        };

        return interfaces;
    }

    /**
     * @brief retrieves the name of the plugin.
     *
     * @return the name of the plugin as a constant reference to a string.
     */
    const std::string& GetPluginName() const override
    {
        static const std::string pluginName = "RaveURDF";
        return pluginName;
    }
};

/**
 * @brief creates an instance of the RaveURDFPlugin for use by OpenRAVE.
 *
 * @return a pointer to the created RavePlugin instance.
 */
OPENRAVE_PLUGIN_API RavePlugin* CreatePlugin()
{
    return new RaveURDFPlugin();
}

/**
 * @brief destroys an instance of the RavePlugin.
 *
 * @param p the pointer to the plugin instance to be destroyed.
 */
OPENRAVE_PLUGIN_API void DestroyPlugin(RavePlugin* p)
{
    delete p;
}
}  // namespace raveurdf
