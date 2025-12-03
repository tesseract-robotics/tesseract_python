/**
 * @file tesseract_environment_bindings.cpp
 * @brief nanobind bindings for tesseract_environment
 */

#include "tesseract_nb.h"
#include <nanobind/stl/map.h>
#include <nanobind/stl/unique_ptr.h>
#include <nanobind/stl/set.h>

// tesseract_environment
#include <tesseract_environment/environment.h>

// tesseract_scene_graph
#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/scene_state.h>
#include <tesseract_scene_graph/link.h>
#include <tesseract_scene_graph/joint.h>

// tesseract_common
#include <tesseract_common/resource_locator.h>
#include <tesseract_common/manipulator_info.h>

// tesseract_srdf
#include <tesseract_srdf/srdf_model.h>

// tesseract_kinematics
#include <tesseract_kinematics/core/joint_group.h>
#include <tesseract_kinematics/core/kinematic_group.h>

// tesseract_collision
#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_collision/core/continuous_contact_manager.h>

namespace te = tesseract_environment;
namespace tsg = tesseract_scene_graph;
namespace tc = tesseract_common;
namespace tk = tesseract_kinematics;

NB_MODULE(_tesseract_environment, m) {
    m.doc() = "tesseract_environment Python bindings";

    // ========== Environment ==========
    nb::class_<te::Environment>(m, "Environment")
        .def(nb::init<>())
        // Init methods
        .def("init", [](te::Environment& self, const tsg::SceneGraph& scene_graph) {
            return self.init(scene_graph);
        }, "scene_graph"_a)
        .def("initFromUrdf", [](te::Environment& self, const std::string& urdf_string,
                                 const std::shared_ptr<const tc::ResourceLocator>& locator) {
            return self.init(urdf_string, locator);
        }, "urdf_string"_a, "locator"_a)
        .def("initFromUrdfSrdf", [](te::Environment& self, const std::string& urdf_string,
                                     const std::string& srdf_string,
                                     const std::shared_ptr<const tc::ResourceLocator>& locator) {
            return self.init(urdf_string, srdf_string, locator);
        }, "urdf_string"_a, "srdf_string"_a, "locator"_a)
        // State methods
        .def("isInitialized", &te::Environment::isInitialized)
        .def("reset", &te::Environment::reset)
        .def("clear", &te::Environment::clear)
        .def("getRevision", &te::Environment::getRevision)
        .def("getName", &te::Environment::getName)
        .def("setName", &te::Environment::setName, "name"_a)
        // Scene graph
        .def("getSceneGraph", &te::Environment::getSceneGraph)
        // State
        .def("getState", [](const te::Environment& self) {
            return self.getState();
        })
        .def("getStateByMap", [](const te::Environment& self,
                                  const std::unordered_map<std::string, double>& joints) {
            return self.getState(joints);
        }, "joints"_a)
        .def("getStateByNamesAndValues", [](const te::Environment& self,
                                             const std::vector<std::string>& joint_names,
                                             const Eigen::Ref<const Eigen::VectorXd>& joint_values) {
            return self.getState(joint_names, joint_values);
        }, "joint_names"_a, "joint_values"_a)
        .def("setState", [](te::Environment& self,
                            const std::unordered_map<std::string, double>& joints) {
            self.setState(joints);
        }, "joints"_a)
        .def("setStateByNamesAndValues", [](te::Environment& self,
                                             const std::vector<std::string>& joint_names,
                                             const Eigen::Ref<const Eigen::VectorXd>& joint_values) {
            self.setState(joint_names, joint_values);
        }, "joint_names"_a, "joint_values"_a)
        // Joint/Link info
        .def("getJointNames", &te::Environment::getJointNames)
        .def("getActiveJointNames", &te::Environment::getActiveJointNames)
        .def("getLinkNames", &te::Environment::getLinkNames)
        .def("getActiveLinkNames", [](const te::Environment& self) {
            return self.getActiveLinkNames();
        })
        .def("getStaticLinkNames", [](const te::Environment& self) {
            return self.getStaticLinkNames();
        })
        .def("getRootLinkName", &te::Environment::getRootLinkName)
        .def("getCurrentJointValues", [](const te::Environment& self) {
            return self.getCurrentJointValues();
        })
        .def("getCurrentJointValuesByNames", [](const te::Environment& self,
                                                 const std::vector<std::string>& joint_names) {
            return self.getCurrentJointValues(joint_names);
        }, "joint_names"_a)
        // Transforms
        .def("getLinkTransform", &te::Environment::getLinkTransform, "link_name"_a)
        .def("getRelativeLinkTransform", &te::Environment::getRelativeLinkTransform,
             "from_link_name"_a, "to_link_name"_a)
        // Link/Joint access
        .def("getLink", &te::Environment::getLink, "name"_a)
        .def("getJoint", &te::Environment::getJoint, "name"_a)
        // Groups
        .def("getGroupNames", &te::Environment::getGroupNames)
        .def("getGroupJointNames", &te::Environment::getGroupJointNames, "group_name"_a)
        .def("getJointGroup", [](const te::Environment& self, const std::string& group_name) {
            return self.getJointGroup(group_name);
        }, "group_name"_a)
        .def("getKinematicGroup", [](const te::Environment& self, const std::string& group_name,
                                      const std::string& ik_solver_name) {
            return self.getKinematicGroup(group_name, ik_solver_name);
        }, "group_name"_a, "ik_solver_name"_a = "")
        // TCP
        .def("findTCPOffset", &te::Environment::findTCPOffset, "manip_info"_a)
        // Contact managers
        .def("getDiscreteContactManager", [](const te::Environment& self) {
            return self.getDiscreteContactManager();
        })
        .def("getContinuousContactManager", [](const te::Environment& self) {
            return self.getContinuousContactManager();
        })
        .def("setActiveDiscreteContactManager", &te::Environment::setActiveDiscreteContactManager, "name"_a)
        .def("setActiveContinuousContactManager", &te::Environment::setActiveContinuousContactManager, "name"_a)
        // ACM and collision
        .def("getAllowedCollisionMatrix", &te::Environment::getAllowedCollisionMatrix)
        .def("getCollisionMarginData", &te::Environment::getCollisionMarginData)
        // Locator
        .def("setResourceLocator", &te::Environment::setResourceLocator, "locator"_a)
        .def("getResourceLocator", &te::Environment::getResourceLocator);
}
