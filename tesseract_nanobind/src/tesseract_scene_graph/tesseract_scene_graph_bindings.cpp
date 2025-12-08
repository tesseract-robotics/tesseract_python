/**
 * @file tesseract_scene_graph_bindings.cpp
 * @brief nanobind bindings for tesseract_scene_graph
 */

#include "tesseract_nb.h"

// tesseract_scene_graph
#include <tesseract_scene_graph/joint.h>
#include <tesseract_scene_graph/link.h>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/scene_state.h>

// tesseract_geometry for Visual/Collision geometry pointer
#include <tesseract_geometry/geometry.h>

// tesseract_common for AllowedCollisionMatrix
#include <tesseract_common/allowed_collision_matrix.h>

namespace tsg = tesseract_scene_graph;
namespace tg = tesseract_geometry;

NB_MODULE(_tesseract_scene_graph, m) {
    m.doc() = "tesseract_scene_graph Python bindings";

    // ==================== Joint-related classes ====================

    // JointType enum
    nb::enum_<tsg::JointType>(m, "JointType")
        .value("UNKNOWN", tsg::JointType::UNKNOWN)
        .value("REVOLUTE", tsg::JointType::REVOLUTE)
        .value("CONTINUOUS", tsg::JointType::CONTINUOUS)
        .value("PRISMATIC", tsg::JointType::PRISMATIC)
        .value("FLOATING", tsg::JointType::FLOATING)
        .value("PLANAR", tsg::JointType::PLANAR)
        .value("FIXED", tsg::JointType::FIXED);

    // Export enum values with SWIG-compatible naming (JointType_*)
    m.attr("JointType_UNKNOWN") = tsg::JointType::UNKNOWN;
    m.attr("JointType_REVOLUTE") = tsg::JointType::REVOLUTE;
    m.attr("JointType_CONTINUOUS") = tsg::JointType::CONTINUOUS;
    m.attr("JointType_PRISMATIC") = tsg::JointType::PRISMATIC;
    m.attr("JointType_FLOATING") = tsg::JointType::FLOATING;
    m.attr("JointType_PLANAR") = tsg::JointType::PLANAR;
    m.attr("JointType_FIXED") = tsg::JointType::FIXED;

    // JointDynamics
    nb::class_<tsg::JointDynamics>(m, "JointDynamics")
        .def(nb::init<>())
        .def(nb::init<double, double>(), "damping"_a, "friction"_a)
        .def_rw("damping", &tsg::JointDynamics::damping)
        .def_rw("friction", &tsg::JointDynamics::friction)
        .def("clear", &tsg::JointDynamics::clear)
        .def("__eq__", &tsg::JointDynamics::operator==)
        .def("__ne__", &tsg::JointDynamics::operator!=)
        .def("__repr__", [](const tsg::JointDynamics& self) {
            return "JointDynamics(damping=" + std::to_string(self.damping) +
                   ", friction=" + std::to_string(self.friction) + ")";
        });

    // JointLimits
    nb::class_<tsg::JointLimits>(m, "JointLimits")
        .def(nb::init<>())
        .def(nb::init<double, double, double, double, double, double>(),
             "lower"_a, "upper"_a, "effort"_a, "velocity"_a, "acceleration"_a, "jerk"_a)
        .def_rw("lower", &tsg::JointLimits::lower)
        .def_rw("upper", &tsg::JointLimits::upper)
        .def_rw("effort", &tsg::JointLimits::effort)
        .def_rw("velocity", &tsg::JointLimits::velocity)
        .def_rw("acceleration", &tsg::JointLimits::acceleration)
        .def_rw("jerk", &tsg::JointLimits::jerk)
        .def("clear", &tsg::JointLimits::clear)
        .def("__eq__", &tsg::JointLimits::operator==)
        .def("__ne__", &tsg::JointLimits::operator!=)
        .def("__repr__", [](const tsg::JointLimits& self) {
            return "JointLimits(lower=" + std::to_string(self.lower) +
                   ", upper=" + std::to_string(self.upper) + ")";
        });

    // JointSafety
    nb::class_<tsg::JointSafety>(m, "JointSafety")
        .def(nb::init<>())
        .def(nb::init<double, double, double, double>(),
             "soft_upper_limit"_a, "soft_lower_limit"_a, "k_position"_a, "k_velocity"_a)
        .def_rw("soft_upper_limit", &tsg::JointSafety::soft_upper_limit)
        .def_rw("soft_lower_limit", &tsg::JointSafety::soft_lower_limit)
        .def_rw("k_position", &tsg::JointSafety::k_position)
        .def_rw("k_velocity", &tsg::JointSafety::k_velocity)
        .def("clear", &tsg::JointSafety::clear)
        .def("__eq__", &tsg::JointSafety::operator==)
        .def("__ne__", &tsg::JointSafety::operator!=);

    // JointCalibration
    nb::class_<tsg::JointCalibration>(m, "JointCalibration")
        .def(nb::init<>())
        .def(nb::init<double, double, double>(),
             "reference_position"_a, "rising"_a, "falling"_a)
        .def_rw("reference_position", &tsg::JointCalibration::reference_position)
        .def_rw("rising", &tsg::JointCalibration::rising)
        .def_rw("falling", &tsg::JointCalibration::falling)
        .def("clear", &tsg::JointCalibration::clear)
        .def("__eq__", &tsg::JointCalibration::operator==)
        .def("__ne__", &tsg::JointCalibration::operator!=);

    // JointMimic
    nb::class_<tsg::JointMimic>(m, "JointMimic")
        .def(nb::init<>())
        .def(nb::init<double, double, std::string>(),
             "offset"_a, "multiplier"_a, "joint_name"_a)
        .def_rw("offset", &tsg::JointMimic::offset)
        .def_rw("multiplier", &tsg::JointMimic::multiplier)
        .def_rw("joint_name", &tsg::JointMimic::joint_name)
        .def("clear", &tsg::JointMimic::clear)
        .def("__eq__", &tsg::JointMimic::operator==)
        .def("__ne__", &tsg::JointMimic::operator!=);

    // Joint (non-copyable)
    nb::class_<tsg::Joint>(m, "Joint")
        .def(nb::init<std::string>(), "name"_a)
        .def(nb::init<>())
        .def("getName", &tsg::Joint::getName)
        .def_rw("type", &tsg::Joint::type)
        .def_rw("axis", &tsg::Joint::axis)
        .def_rw("child_link_name", &tsg::Joint::child_link_name)
        .def_rw("parent_link_name", &tsg::Joint::parent_link_name)
        .def_rw("parent_to_joint_origin_transform", &tsg::Joint::parent_to_joint_origin_transform)
        .def_rw("dynamics", &tsg::Joint::dynamics)
        .def_rw("limits", &tsg::Joint::limits)
        .def_rw("safety", &tsg::Joint::safety)
        .def_rw("calibration", &tsg::Joint::calibration)
        .def_rw("mimic", &tsg::Joint::mimic)
        .def("clear", &tsg::Joint::clear)
        .def("clone", nb::overload_cast<>(&tsg::Joint::clone, nb::const_))
        .def("clone", nb::overload_cast<const std::string&>(&tsg::Joint::clone, nb::const_), "name"_a)
        .def("__eq__", &tsg::Joint::operator==)
        .def("__ne__", &tsg::Joint::operator!=)
        .def("__repr__", [](const tsg::Joint& self) {
            return "Joint('" + self.getName() + "')";
        });

    // ==================== Link-related classes ====================

    // Material
    nb::class_<tsg::Material>(m, "Material")
        .def(nb::init<>())
        .def(nb::init<std::string>(), "name"_a)
        .def("getName", &tsg::Material::getName)
        .def_rw("texture_filename", &tsg::Material::texture_filename)
        .def_rw("color", &tsg::Material::color)
        .def_static("getDefaultMaterial", &tsg::Material::getDefaultMaterial)
        .def("clear", &tsg::Material::clear)
        .def("__eq__", &tsg::Material::operator==)
        .def("__ne__", &tsg::Material::operator!=);

    // Inertial
    nb::class_<tsg::Inertial>(m, "Inertial")
        .def(nb::init<>())
        .def_rw("origin", &tsg::Inertial::origin)
        .def_rw("mass", &tsg::Inertial::mass)
        .def_rw("ixx", &tsg::Inertial::ixx)
        .def_rw("ixy", &tsg::Inertial::ixy)
        .def_rw("ixz", &tsg::Inertial::ixz)
        .def_rw("iyy", &tsg::Inertial::iyy)
        .def_rw("iyz", &tsg::Inertial::iyz)
        .def_rw("izz", &tsg::Inertial::izz)
        .def("clear", &tsg::Inertial::clear)
        .def("__eq__", &tsg::Inertial::operator==)
        .def("__ne__", &tsg::Inertial::operator!=);

    // Visual
    nb::class_<tsg::Visual>(m, "Visual")
        .def(nb::init<>())
        .def_rw("origin", &tsg::Visual::origin)
        .def_prop_rw("geometry",
            [](const tsg::Visual& self) -> std::shared_ptr<tg::Geometry> {
                return std::const_pointer_cast<tg::Geometry>(self.geometry);
            },
            [](tsg::Visual& self, std::shared_ptr<const tg::Geometry> g) { self.geometry = g; })
        .def_rw("material", &tsg::Visual::material)
        .def_rw("name", &tsg::Visual::name)
        .def("clear", &tsg::Visual::clear)
        .def("__eq__", &tsg::Visual::operator==)
        .def("__ne__", &tsg::Visual::operator!=);

    // Collision
    nb::class_<tsg::Collision>(m, "Collision")
        .def(nb::init<>())
        .def_rw("origin", &tsg::Collision::origin)
        .def_prop_rw("geometry",
            [](const tsg::Collision& self) -> std::shared_ptr<tg::Geometry> {
                return std::const_pointer_cast<tg::Geometry>(self.geometry);
            },
            [](tsg::Collision& self, std::shared_ptr<const tg::Geometry> g) { self.geometry = g; })
        .def_rw("name", &tsg::Collision::name)
        .def("clear", &tsg::Collision::clear)
        .def("__eq__", &tsg::Collision::operator==)
        .def("__ne__", &tsg::Collision::operator!=);

    // Link (non-copyable)
    nb::class_<tsg::Link>(m, "Link")
        .def(nb::init<std::string>(), "name"_a)
        .def(nb::init<>())
        .def("getName", &tsg::Link::getName)
        .def_rw("inertial", &tsg::Link::inertial)
        .def_rw("visual", &tsg::Link::visual)
        .def_rw("collision", &tsg::Link::collision)
        .def("clear", &tsg::Link::clear)
        .def("clone", nb::overload_cast<>(&tsg::Link::clone, nb::const_))
        .def("clone", nb::overload_cast<const std::string&>(&tsg::Link::clone, nb::const_), "name"_a)
        .def("__eq__", &tsg::Link::operator==)
        .def("__ne__", &tsg::Link::operator!=)
        .def("__repr__", [](const tsg::Link& self) {
            return "Link('" + self.getName() + "')";
        });

    // ==================== Graph-related classes ====================

    // ShortestPath
    nb::class_<tsg::ShortestPath>(m, "ShortestPath")
        .def(nb::init<>())
        .def_rw("links", &tsg::ShortestPath::links)
        .def_rw("joints", &tsg::ShortestPath::joints)
        .def_rw("active_joints", &tsg::ShortestPath::active_joints);

    // SceneGraph (non-copyable)
    nb::class_<tsg::SceneGraph>(m, "SceneGraph")
        .def(nb::init<const std::string&>(), "name"_a = "")
        .def("clone", &tsg::SceneGraph::clone)
        .def("clear", &tsg::SceneGraph::clear)
        .def("setName", &tsg::SceneGraph::setName, "name"_a)
        .def("getName", &tsg::SceneGraph::getName)
        .def("setRoot", &tsg::SceneGraph::setRoot, "name"_a)
        .def("getRoot", &tsg::SceneGraph::getRoot)

        // Link operations
        .def("addLink", nb::overload_cast<const tsg::Link&, bool>(&tsg::SceneGraph::addLink),
             "link"_a, "replace_allowed"_a = false)
        .def("addLink", nb::overload_cast<const tsg::Link&, const tsg::Joint&>(&tsg::SceneGraph::addLink),
             "link"_a, "joint"_a)
        .def("getLink", &tsg::SceneGraph::getLink, "name"_a)
        .def("getLinks", &tsg::SceneGraph::getLinks)
        .def("getLeafLinks", &tsg::SceneGraph::getLeafLinks)
        .def("removeLink", &tsg::SceneGraph::removeLink, "name"_a, "recursive"_a = false)
        .def("moveLink", &tsg::SceneGraph::moveLink, "joint"_a)
        .def("setLinkVisibility", &tsg::SceneGraph::setLinkVisibility, "name"_a, "visibility"_a)
        .def("getLinkVisibility", &tsg::SceneGraph::getLinkVisibility, "name"_a)
        .def("setLinkCollisionEnabled", &tsg::SceneGraph::setLinkCollisionEnabled, "name"_a, "enabled"_a)
        .def("getLinkCollisionEnabled", &tsg::SceneGraph::getLinkCollisionEnabled, "name"_a)

        // Joint operations
        .def("addJoint", &tsg::SceneGraph::addJoint, "joint"_a)
        .def("getJoint", &tsg::SceneGraph::getJoint, "name"_a)
        .def("removeJoint", &tsg::SceneGraph::removeJoint, "name"_a, "recursive"_a = false)
        .def("moveJoint", &tsg::SceneGraph::moveJoint, "name"_a, "parent_link"_a)
        .def("getJoints", &tsg::SceneGraph::getJoints)
        .def("getActiveJoints", &tsg::SceneGraph::getActiveJoints)
        .def("changeJointOrigin", &tsg::SceneGraph::changeJointOrigin, "name"_a, "new_origin"_a)
        .def("changeJointLimits", &tsg::SceneGraph::changeJointLimits, "name"_a, "limits"_a)
        .def("changeJointPositionLimits", &tsg::SceneGraph::changeJointPositionLimits,
             "name"_a, "lower"_a, "upper"_a)
        .def("changeJointVelocityLimits", &tsg::SceneGraph::changeJointVelocityLimits, "name"_a, "limit"_a)
        .def("changeJointAccelerationLimits", &tsg::SceneGraph::changeJointAccelerationLimits, "name"_a, "limit"_a)
        .def("changeJointJerkLimits", &tsg::SceneGraph::changeJointJerkLimits, "name"_a, "limit"_a)
        .def("getJointLimits", &tsg::SceneGraph::getJointLimits, "name"_a)

        // Collision matrix
        .def("addAllowedCollision", &tsg::SceneGraph::addAllowedCollision,
             "link_name1"_a, "link_name2"_a, "reason"_a)
        .def("removeAllowedCollision",
             nb::overload_cast<const std::string&, const std::string&>(&tsg::SceneGraph::removeAllowedCollision),
             "link_name1"_a, "link_name2"_a)
        .def("removeAllowedCollision",
             nb::overload_cast<const std::string&>(&tsg::SceneGraph::removeAllowedCollision),
             "link_name"_a)
        .def("clearAllowedCollisions", &tsg::SceneGraph::clearAllowedCollisions)
        .def("isCollisionAllowed", &tsg::SceneGraph::isCollisionAllowed, "link_name1"_a, "link_name2"_a)
        .def("getAllowedCollisionMatrix",
             nb::overload_cast<>(&tsg::SceneGraph::getAllowedCollisionMatrix),
             "Get the allowed collision matrix")

        // Graph queries
        .def("getSourceLink", &tsg::SceneGraph::getSourceLink, "joint_name"_a)
        .def("getTargetLink", &tsg::SceneGraph::getTargetLink, "joint_name"_a)
        .def("getInboundJoints", &tsg::SceneGraph::getInboundJoints, "link_name"_a)
        .def("getOutboundJoints", &tsg::SceneGraph::getOutboundJoints, "link_name"_a)
        .def("isAcyclic", &tsg::SceneGraph::isAcyclic)
        .def("isTree", &tsg::SceneGraph::isTree)
        .def("isEmpty", &tsg::SceneGraph::isEmpty)
        .def("getAdjacentLinkNames", &tsg::SceneGraph::getAdjacentLinkNames, "name"_a)
        .def("getInvAdjacentLinkNames", &tsg::SceneGraph::getInvAdjacentLinkNames, "name"_a)
        .def("getLinkChildrenNames", &tsg::SceneGraph::getLinkChildrenNames, "name"_a)
        .def("getJointChildrenNames",
             nb::overload_cast<const std::string&>(&tsg::SceneGraph::getJointChildrenNames, nb::const_),
             "name"_a)
        .def("getShortestPath", &tsg::SceneGraph::getShortestPath, "root"_a, "tip"_a)
        .def("saveDOT", &tsg::SceneGraph::saveDOT, "path"_a)

        // Insert scene graph
        .def("insertSceneGraph",
             nb::overload_cast<const tsg::SceneGraph&, const std::string&>(&tsg::SceneGraph::insertSceneGraph),
             "scene_graph"_a, "prefix"_a = "")
        .def("insertSceneGraph",
             nb::overload_cast<const tsg::SceneGraph&, const tsg::Joint&, const std::string&>(&tsg::SceneGraph::insertSceneGraph),
             "scene_graph"_a, "joint"_a, "prefix"_a = "")

        .def("__eq__", &tsg::SceneGraph::operator==)
        .def("__ne__", &tsg::SceneGraph::operator!=)
        .def("__repr__", [](const tsg::SceneGraph& self) {
            return "SceneGraph('" + self.getName() + "')";
        });

    // Note: SceneState is bound in tesseract_state_solver module with proper
    // AlignedMap to std::map conversion for link_transforms and joint_transforms.
    // Import from tesseract_robotics.tesseract_state_solver.
}
