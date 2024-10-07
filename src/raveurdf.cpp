#include <raveurdf/raveurdf.h>

// called to create a new plugin
OpenRAVE::InterfaceBasePtr CreateInterfaceValidated(OpenRAVE::InterfaceType type,
                                                    const std::string& interfacename,
                                                    std::istream& sinput,
                                                    OpenRAVE::EnvironmentBasePtr penv)
{

    return OpenRAVE::InterfaceBasePtr();
}

// called to query available plugins
void GetPluginAttributesValidated(OpenRAVE::PLUGININFO& info) {}

// called before plugin is terminated
OPENRAVE_PLUGIN_API void DestroyPlugin() {}
