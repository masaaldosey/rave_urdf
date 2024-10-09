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

        // set collision geometry information
        for (const auto& collision : link_ptr->collision_array) {
            auto geom_info = boost::make_shared<OpenRAVE::KinBody::GeometryInfo>();
            geom_info->SetTransform(raveurdf_utils::urdfPoseToRaveTransform(collision->origin));
            geom_info->_bVisible    = false;
            geom_info->_bModifiable = false;

            // set geometry type
            switch (collision->geometry->type) {
                case urdf::Geometry::MESH: {
                    auto* mesh = dynamic_cast<const urdf::Mesh*>(collision->geometry.get());
                    if (mesh) {
                        geom_info->_filenamecollision = mesh->filename;
                        geom_info->_type              = OpenRAVE::GT_TriMesh;
                        geom_info->_vCollisionScale   = raveurdf_utils::urdfVectorToRaveVector(mesh->scale);

                        auto trimesh = boost::make_shared<OpenRAVE::TriMesh>();
                        trimesh      = GetEnv()->ReadTrimeshURI(trimesh, geom_info->_filenamecollision);
                        if (trimesh) {
                            for (auto& vertex : trimesh->vertices) {
                                vertex *= geom_info->_vCollisionScale;
                            }
                            geom_info->_meshcollision = *trimesh;
                        }
                        else {
                            RAVELOG_WARN("Failed loading mesh: %s\n", geom_info->_filenamecollision.c_str());
                        }
                    }
                    break;
                }
                case urdf::Geometry::SPHERE: {
                    auto* sphere = dynamic_cast<const urdf::Sphere*>(collision->geometry.get());
                    if (sphere) {
                        geom_info->_vGeomData = sphere->radius * OpenRAVE::Vector(1, 1, 1);
                        geom_info->_type      = OpenRAVE::GT_Sphere;
                    }
                    break;
                }
                case urdf::Geometry::BOX: {
                    auto* box = dynamic_cast<const urdf::Box*>(collision->geometry.get());
                    if (box) {
                        geom_info->_vGeomData = 0.5 * OpenRAVE::Vector(box->dim.x, box->dim.y, box->dim.z);
                        geom_info->_type      = OpenRAVE::GT_Box;
                    }
                    break;
                }
                case urdf::Geometry::CYLINDER: {
                    auto* cylinder = dynamic_cast<const urdf::Cylinder*>(collision->geometry.get());
                    if (cylinder) {
                        geom_info->_vGeomData = OpenRAVE::Vector(cylinder->radius, cylinder->length, 0);
                        geom_info->_type      = OpenRAVE::GT_Cylinder;
                    }
                    break;
                }
            }
            link_info->_vgeometryinfos.push_back(geom_info);
        }

        // add the render geometry. we can't create a link with no collision
        // geometry, so we'll instead create a zero-radius sphere with the
        // desired render mesh.
        std::shared_ptr<urdf::Visual> visual = link_ptr->visual;
        if (visual) {
            OpenRAVE::KinBody::GeometryInfoPtr geom_info = boost::make_shared<OpenRAVE::KinBody::GeometryInfo>();
            geom_info->SetTransform(raveurdf_utils::urdfPoseToRaveTransform(visual->origin));
            geom_info->_type        = OpenRAVE::GT_Sphere;
            geom_info->_vGeomData   = OpenRAVE::Vector(0.0, 0.0, 0.0);
            geom_info->_bModifiable = false;
            geom_info->_bVisible    = true;

            switch (visual->geometry->type) {
                case urdf::Geometry::MESH: {
                    const urdf::Mesh& mesh     = dynamic_cast<const urdf::Mesh&>(*visual->geometry);
                    geom_info->_filenamerender = mesh.filename;
                    geom_info->_vRenderScale   = raveurdf_utils::urdfVectorToRaveVector(mesh.scale);

                    // If a material color is specified, use it.
                    std::shared_ptr<urdf::Material> material = visual->material;
                    if (material) {
                        geom_info->_vDiffuseColor = raveurdf_utils::urdfColorToRaveVector(material->color);
                        geom_info->_vAmbientColor = raveurdf_utils::urdfColorToRaveVector(material->color);
                    }

                    // Add the render-only geometry to the standard geometry group for
                    // backwards compatability with QtCoin.
                    link_info->_vgeometryinfos.push_back(geom_info);

                    // Create a group dedicated to visual geometry for or_rviz.
                    OpenRAVE::KinBody::GeometryInfoPtr geom_info_clone =
                        boost::make_shared<OpenRAVE::KinBody::GeometryInfo>(*geom_info);
                    link_info->_mapExtraGeometries["visual"].push_back(geom_info_clone);
                    break;
                }

                default:
                    RAVELOG_WARN("Link[%s]: Only trimeshes are supported for visual geometry.\n",
                                 link_ptr->name.c_str());
            }
        }
        // verify that the "visual" and "spheres" groups always exist. Recall
        // that accessing an element with operator[] creates it using the default
        // no-arg constructor if it does not already exist.
        link_info->_mapExtraGeometries["visual"];
        link_info->_mapExtraGeometries["spheres"];

        link_infos.push_back(link_info);
    }

    // populate vector of joints
    std::string joint_name;
    std::shared_ptr<urdf::Joint> joint_ptr;

    // parse the joint properties
    std::vector<std::shared_ptr<urdf::Joint>> ordered_joints;
    for (const auto& joint : model.joints_) {
        ordered_joints.push_back(joint.second);
    }

    // iterate over the joints using a range-based for loop
    for (const auto& joint_ptr : ordered_joints) {
        OpenRAVE::KinBody::JointInfoPtr joint_info = boost::make_shared<OpenRAVE::KinBody::JointInfo>();
        joint_info->_name                          = joint_ptr->name;
        joint_info->_linkname0                     = joint_ptr->parent_link_name;
        joint_info->_linkname1                     = joint_ptr->child_link_name;
        joint_info->_vanchor =
            raveurdf_utils::urdfVectorToRaveVector(joint_ptr->parent_to_joint_origin_transform.position);

        int const urdf_joint_type = joint_ptr->type;

        // set the joint type. some URDF joints correspond to disabled OpenRAVE
        // joints, so we'll appropriately set the corresponding IsActive flag.
        OpenRAVE::KinBody::JointType joint_type;
        bool enabled;
        boost::tie(joint_type, enabled) = raveurdf_utils::urdfJointTypeToRaveJointType(urdf_joint_type);
        joint_info->_type               = joint_type;
        joint_info->_bIsActive          = enabled;
        joint_info->_bIsCircular[0]     = (urdf_joint_type == urdf::Joint::CONTINUOUS);

        // URDF only supports linear mimic joints with a constant offset. we map
        // that into the correct position (index 0) and velocity (index 1)
        // equations for OpenRAVE.
        std::shared_ptr<urdf::JointMimic> mimic = joint_ptr->mimic;
        if (mimic) {
            joint_info->_vmimic[0] = boost::make_shared<OpenRAVE::KinBody::MimicInfo>();
            joint_info->_vmimic[0]->_equations[0] =
                boost::str(boost::format("%s*%0.6f+%0.6f") % mimic->joint_name % mimic->multiplier % mimic->offset);
            joint_info->_vmimic[0]->_equations[1] =
                boost::str(boost::format("|%s %0.6f") % mimic->joint_name % mimic->multiplier);
        }

        // configure joint axis. add an arbitrary axis if the joint is disabled.
        urdf::Vector3 joint_axis;
        if (enabled) {
            joint_axis = joint_ptr->parent_to_joint_origin_transform.rotation * joint_ptr->axis;
        }
        else {
            joint_axis = urdf::Vector3(1, 0, 0);
        }
        joint_info->_vaxes[0] = raveurdf_utils::urdfVectorToRaveVector(joint_axis);

        // configure joint limits.
        std::shared_ptr<urdf::JointLimits> limits = joint_ptr->limits;
        if (limits) {
            joint_info->_vlowerlimit[0] = limits->lower;
            joint_info->_vupperlimit[0] = limits->upper;
            joint_info->_vmaxvel[0]     = limits->velocity;
            joint_info->_vmaxtorque[0]  = limits->effort;
        }
        // fixed joints are just revolute joints with zero limits.
        else if (!enabled) {
            joint_info->_vlowerlimit[0] = 0;
            joint_info->_vupperlimit[0] = 0;
        }
        // for revolute joints default to +/- 2*PI. this is
        // the same default used by OpenRAVE for revolute joints.
        else {
            joint_info->_vlowerlimit[0] = -M_PI;
            joint_info->_vupperlimit[0] = M_PI;
        }

        // force continuous joints to have +/- PI limits. otherwise, the internal
        // _vcircularlowerlimit and _vcircularupperlimit values will be set to
        // zero. this causes OpenRAVE::utils::NormalizeCircularAngle to crash.
        if (urdf_joint_type == urdf::Joint::CONTINUOUS) {
            joint_info->_vlowerlimit[0] = -M_PI;
            joint_info->_vupperlimit[0] = M_PI;
        }

        joint_infos.push_back(joint_info);
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
