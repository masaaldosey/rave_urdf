#include <raveurdf/raveurdf.h>
#include <raveurdf/utils.h>

namespace raveurdf {

RaveURDF::RaveURDF(OpenRAVE::EnvironmentBasePtr penv)
    : OpenRAVE::ModuleBase(penv)
{
    __description = "URDFLoader: Loader that imports URDF files.";

    RegisterCommand("load",
                    boost::bind(&RaveURDF::loadURDFFile, this, boost::placeholders::_1, boost::placeholders::_2),
                    "Load a URDF from a file");
    RAVELOG_INFO("URDFLoader: Loaded URDFLoader plugin\n");
}

bool RaveURDF::loadURDFFile(std::ostream& soutput, std::istream& sinput)
{
    std::string path_to_urdf;
    sinput >> path_to_urdf;

    RAVELOG_INFO("Loading URDF file: %s\n", path_to_urdf.c_str());

    // parse the URDF file
    urdf::Model urdf_model;
    if (!urdf_model.initFile(path_to_urdf)) {
        RAVELOG_ERROR("Failed to open URDF file: %s\n", path_to_urdf.c_str());
    }

    soutput << this->loadRobotModel(urdf_model, xml_doc, path_to_urdf);

    return true;
}

std::string RaveURDF::loadRobotModel(urdf::Model& urdf_model, TiXmlDocument& xml_doc, std::string path_to_urdf)
{
    OpenRAVE::KinBodyPtr body;
    std::string name;

    // parse links and joints information from the URDF model
    std::vector<OpenRAVE::KinBody::LinkInfoPtr> link_infos;
    std::vector<OpenRAVE::KinBody::JointInfoPtr> joint_infos;
    this->parseURDF(urdf_model, link_infos, joint_infos);

    // create the robot model
    std::vector<OpenRAVE::KinBody::LinkInfoConstPtr> link_infos_const =
        raveurdf_utils::convertToConstantSharedPointers(link_infos);
    std::vector<OpenRAVE::KinBody::JointInfoConstPtr> joint_infos_const =
        raveurdf_utils::convertToConstantSharedPointers(joint_infos);
    std::vector<OpenRAVE::RobotBase::ManipulatorInfoPtr> manip_infos;
    std::vector<OpenRAVE::RobotBase::AttachedSensorInfoPtr> sensor_infos;

    OpenRAVE::RobotBasePtr robot = OpenRAVE::RaveCreateRobot(GetEnv(), "");
    robot->Init(link_infos_const,
                joint_infos_const,
                raveurdf_utils::convertToConstantSharedPointers(manip_infos),
                raveurdf_utils::convertToConstantSharedPointers(sensor_infos),
                path_to_urdf);
    body = robot;

    body->SetName(urdf_model.getName());

    // interFaceAddMode details are available at the below link
    // https://github.com/rdiankov/openrave/blob/ec22ecfaf006688cbc5ee0fdd8fa05d2c5676d37/include/openrave/environment.h#L48
    GetEnv()->Add(body, static_cast<OpenRAVE::InterfaceAddMode>(0));
    return body->GetName();
}

void RaveURDF::parseURDF(urdf::Model& model,
                         std::vector<OpenRAVE::KinBody::LinkInfoPtr>& link_infos,
                         std::vector<OpenRAVE::KinBody::JointInfoPtr>& joint_infos)
{
    // get all links from URDF model
    std::vector<std::shared_ptr<urdf::Link>> link_vector;
    model.getLinks(link_vector);
    RAVELOG_INFO("Number of links in URDF model: %d\n", link_vector.size());
    RAVELOG_INFO("Number of joints in URDF model: %d\n", model.joints_.size());

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
            std::shared_ptr<urdf::Link const> parent_link = model.getLink(parent_joint->parent_link_name);
            parent_joint                                  = parent_link->parent_joint;
        }

        // set collision geometry information
        for (const auto& collision : link_ptr->collision_array) {
            auto geom_info = boost::make_shared<OpenRAVE::KinBody::GeometryInfo>();
            geom_info->SetTransform(raveurdf_utils::urdfPoseToRaveTransform(collision->origin));
            // keep collision geometry not visible
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
            link_info->_mapExtraGeometries["collision"].push_back(geom_info);
        }

        // add the render geometry. We can create a separate visual mesh now.
        std::shared_ptr<urdf::Visual> visual = link_ptr->visual;
        if (visual) {
            OpenRAVE::KinBody::GeometryInfoPtr geom_info = boost::make_shared<OpenRAVE::KinBody::GeometryInfo>();
            geom_info->SetTransform(raveurdf_utils::urdfPoseToRaveTransform(visual->origin));
            geom_info->_type        = OpenRAVE::GT_TriMesh;
            geom_info->_bModifiable = false;
            // make visual geometry visible
            geom_info->_bVisible = true;

            // set mesh properties
            if (visual->geometry->type == urdf::Geometry::MESH) {
                auto* mesh                 = static_cast<const urdf::Mesh*>(visual->geometry.get());
                geom_info->_filenamerender = mesh->filename;
                geom_info->_vRenderScale   = raveurdf_utils::urdfVectorToRaveVector(mesh->scale);

                // apply material colors if available
                std::shared_ptr<urdf::Material> material = visual->material;
                if (material) {
                    geom_info->_vDiffuseColor = raveurdf_utils::urdfColorToRaveVector(material->color);
                    geom_info->_vAmbientColor = raveurdf_utils::urdfColorToRaveVector(material->color);
                }

                // load the mesh
                auto trimesh = boost::make_shared<OpenRAVE::TriMesh>();
                trimesh      = GetEnv()->ReadTrimeshURI(trimesh, geom_info->_filenamerender);
                if (trimesh) {
                    for (auto& vertex : trimesh->vertices) {
                        vertex *= geom_info->_vRenderScale;
                    }
                    geom_info->_meshcollision = *trimesh;
                }
                else {
                    RAVELOG_WARN("Failed loading mesh: %s\n", geom_info->_filenamerender.c_str());
                }

                // add geometry to both main and "visual" groups to ensure export
                // main geometry list
                link_info->_vgeometryinfos.push_back(geom_info);
                // visual group
                link_info->_mapExtraGeometries["visual"].push_back(geom_info);
            }
            else if (visual->geometry->type == urdf::Geometry::BOX) {
                auto* mesh            = static_cast<const urdf::Box*>(visual->geometry.get());
                geom_info->_vGeomData = 0.5 * OpenRAVE::Vector(mesh->dim.x, mesh->dim.y, mesh->dim.z);
                geom_info->_type      = OpenRAVE::GT_Box;
                link_info->_vgeometryinfos.push_back(geom_info);
            }
            // TODO: add support for other geometry types
            else {
                RAVELOG_WARN("Link[%s]: Only mesh geometry is supported for visual elements.\n",
                             link_ptr->name.c_str());
            }
        }

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

        // set the joint type. some URDF joints correspond to disabled OpenRAVE joints
        OpenRAVE::KinBody::JointType joint_type;
        bool enabled;
        boost::tie(joint_type, enabled) = raveurdf_utils::urdfJointTypeToRaveJointType(urdf_joint_type);
        joint_info->_type               = joint_type;
        joint_info->_bIsActive          = enabled;
        joint_info->_bIsCircular[0]     = (urdf_joint_type == urdf::Joint::CONTINUOUS);

        // configure joint axis
        urdf::Vector3 joint_axis;
        if (enabled) {
            joint_axis = joint_ptr->parent_to_joint_origin_transform.rotation * joint_ptr->axis;
        }
        else {
            joint_axis = urdf::Vector3(1, 0, 0);
        }
        joint_info->_vaxes[0] = raveurdf_utils::urdfVectorToRaveVector(joint_axis);

        // configure joint limits
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
