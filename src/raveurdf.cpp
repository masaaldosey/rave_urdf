#include <raveurdf/raveurdf.h>
#include <raveurdf/utils.h>

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

    soutput << this->loadRobotModel(urdf_model, xml_doc, path_to_urdf);
    return true;
}

std::string RaveURDF::loadRobotModel(urdf::Model& urdf_model, TiXmlDocument& xml_doc, std::string path_to_urdf)
{
    OpenRAVE::KinBodyPtr body;
    std::string name;

    std::vector<OpenRAVE::KinBody::LinkInfoPtr> link_infos;
    std::vector<OpenRAVE::KinBody::JointInfoPtr> joint_infos;
    this->parseURDF(urdf_model, link_infos, joint_infos);

    return std::string();
}

void RaveURDF::parseURDF(urdf::Model& model,
                         std::vector<OpenRAVE::KinBody::LinkInfoPtr>& link_infos,
                         std::vector<OpenRAVE::KinBody::JointInfoPtr>& joint_infos)
{
    // get all links from URDF model
    std::vector<std::shared_ptr<urdf::Link>> link_vector;
    model.getLinks(link_vector);

    // populate link_list with links in the order they appear in the URDF file
    std::list<std::shared_ptr<urdf::Link const>> link_list;
    link_list.insert(link_list.begin(), model.getRoot());
    for (const auto& link : link_vector) {
        if (link != model.getRoot()) {
            link_list.insert(link_list.end(), link);
        }
    }

    // traverse through the URDF model and populate openRAVE link information
    // prevent infinite loops by tracking processed links
    std::shared_ptr<urdf::Link const> link_ptr;
    std::set<std::string> processed_links;
    while (!link_list.empty()) {
        link_ptr = link_list.front();
        link_list.pop_front();

        // skip if link exists in `processed_links`
        if (processed_links.find(link_ptr->name) != processed_links.end()) {
            continue;
        }
        OpenRAVE::KinBody::LinkInfoPtr link_info = boost::make_shared<OpenRAVE::KinBody::LinkInfo>();
        link_info->_name                         = link_ptr->name;
        processed_links.insert(link_ptr->name);

        // set inertial parameters if available
        std::shared_ptr<urdf::Inertial> inertial = link_ptr->inertial;
        if (inertial) {
            link_info->_mass            = inertial->mass;
            link_info->_tMassFrame      = raveurdf_utils::urdfPoseToRaveTransform(inertial->origin);
            link_info->_vinertiamoments = OpenRAVE::Vector(inertial->ixx, inertial->iyy, inertial->izz);
        }

        // traverse parent joints to determine link transformation
        std::shared_ptr<urdf::Joint> parent_joint = link_ptr->parent_joint;
        while (parent_joint) {
            link_info->SetTransform(
                raveurdf_utils::urdfPoseToRaveTransform(parent_joint->parent_to_joint_origin_transform) *
                link_info->GetTransform());
            auto parent_link = model.getLink(parent_joint->parent_link_name);
            if (!parent_link || processed_links.count(parent_link->name)) {
                break;
            }
            parent_joint = parent_link->parent_joint;
        }

        // TODO: set collision geometry information
    }
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
