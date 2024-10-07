#include <raveurdf/raveurdf.h>

namespace raveurdf {

RaveURDF::RaveURDF(OpenRAVE::EnvironmentBasePtr penv)
    : OpenRAVE::ModuleBase(penv)
{
    __description = "URDFLoader: Loader that imports URDF files.";
    _env          = penv;

    RegisterCommand("loadURDF",
                    boost::bind(&RaveURDF::loadURDF, this, boost::placeholders::_1, boost::placeholders::_2),
                    "Load a URDF from a file");
    RAVELOG_INFO("URDFLoader: Loaded URDFLoader plugin\n");
}

bool RaveURDF::loadURDF(std::ostream& soutput, std::istream& sinput)
{
    std::string path_to_urdf;
    sinput >> path_to_urdf;

    // parse the URDF file
    urdf::Model urdf_model;
    if (!urdf_model.initFile(path_to_urdf)) {
        RAVELOG_ERROR("Failed to open URDF file: %s\n", path_to_urdf.c_str());
    }

    // parse the URDF again to find <geometry_group> elements
    TiXmlDocument xml_doc(path_to_urdf.c_str());
    if (!xml_doc.LoadFile()) {
        RAVELOG_ERROR("Failed to open URDF file for parsing: %s\n", path_to_urdf.c_str());
    }

    soutput << this->loadModel(urdf_model, xml_doc, path_to_urdf);
    return true;
}

std::string RaveURDF::loadModel(urdf::Model& urdf_model, TiXmlDocument& xml_doc, std::string path_to_urdf)
{
    return std::string();
}

}  // namespace raveurdf

// TODO: see if these are necessary
// // called to create a new plugin
// OpenRAVE::InterfaceBasePtr CreateInterfaceValidated(OpenRAVE::InterfaceType type,
//                                                     const std::string& interfacename,
//                                                     std::istream& sinput,
//                                                     OpenRAVE::EnvironmentBasePtr penv)
// {

//     return OpenRAVE::InterfaceBasePtr();
// }

// // called to query available plugins
// void GetPluginAttributesValidated(OpenRAVE::PLUGININFO& info) {}

// // called before plugin is terminated
// OPENRAVE_PLUGIN_API void DestroyPlugin() {}
