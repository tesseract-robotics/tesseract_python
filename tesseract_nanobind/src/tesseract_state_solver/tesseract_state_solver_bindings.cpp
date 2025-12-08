/**
 * @file tesseract_state_solver_bindings.cpp
 * @brief nanobind bindings for tesseract_state_solver
 */

#include "tesseract_nb.h"
#include <nanobind/stl/map.h>

// tesseract_state_solver
#include <tesseract_state_solver/state_solver.h>
#include <tesseract_state_solver/mutable_state_solver.h>
#include <tesseract_state_solver/kdl/kdl_state_solver.h>
#include <tesseract_state_solver/ofkt/ofkt_state_solver.h>

// tesseract_scene_graph for SceneState and SceneGraph
#include <tesseract_scene_graph/scene_state.h>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/link.h>
#include <tesseract_scene_graph/joint.h>

// tesseract_common
#include <tesseract_common/kinematic_limits.h>
#include <tesseract_common/eigen_types.h>

namespace tsg = tesseract_scene_graph;
namespace tc = tesseract_common;

NB_MODULE(_tesseract_state_solver, m) {
    m.doc() = "tesseract_state_solver Python bindings";

    // ========== SceneState ==========
    nb::class_<tsg::SceneState>(m, "SceneState")
        .def(nb::init<>())
        .def_rw("joints", &tsg::SceneState::joints)
        .def_prop_rw("link_transforms",
            [](const tsg::SceneState& self) {
                // Convert AlignedMap to std::map for Python
                std::map<std::string, Eigen::Isometry3d> result;
                for (const auto& p : self.link_transforms) {
                    result[p.first] = p.second;
                }
                return result;
            },
            [](tsg::SceneState& self, const std::map<std::string, Eigen::Isometry3d>& m) {
                self.link_transforms.clear();
                for (const auto& p : m) {
                    self.link_transforms[p.first] = p.second;
                }
            })
        .def_prop_rw("joint_transforms",
            [](const tsg::SceneState& self) {
                std::map<std::string, Eigen::Isometry3d> result;
                for (const auto& p : self.joint_transforms) {
                    result[p.first] = p.second;
                }
                return result;
            },
            [](tsg::SceneState& self, const std::map<std::string, Eigen::Isometry3d>& m) {
                self.joint_transforms.clear();
                for (const auto& p : m) {
                    self.joint_transforms[p.first] = p.second;
                }
            })
        .def("getJointValues", &tsg::SceneState::getJointValues, "joint_names"_a);

    // ========== StateSolver (abstract base) ==========
    nb::class_<tsg::StateSolver>(m, "StateSolver")
        // getState methods - multiple overloads with same name for Python compatibility
        .def("getState", [](const tsg::StateSolver& self) {
            return self.getState();
        })
        .def("getState", [](const tsg::StateSolver& self,
                            const Eigen::Ref<const Eigen::VectorXd>& joint_values) {
            return self.getState(joint_values);
        }, "joint_values"_a)
        .def("getState", [](const tsg::StateSolver& self,
                            const std::unordered_map<std::string, double>& joint_values) {
            return self.getState(joint_values);
        }, "joint_values"_a)
        .def("getState", [](const tsg::StateSolver& self,
                            const std::vector<std::string>& joint_names,
                            const Eigen::Ref<const Eigen::VectorXd>& joint_values) {
            return self.getState(joint_names, joint_values);
        }, "joint_names"_a, "joint_values"_a)
        .def("getRandomState", &tsg::StateSolver::getRandomState)
        // setState methods
        .def("setState", [](tsg::StateSolver& self, const Eigen::Ref<const Eigen::VectorXd>& joint_values) {
            self.setState(joint_values);
        }, "joint_values"_a)
        .def("setStateByMap", [](tsg::StateSolver& self,
                                  const std::unordered_map<std::string, double>& joint_values) {
            self.setState(joint_values);
        }, "joint_values"_a)
        .def("setStateByNamesAndValues", [](tsg::StateSolver& self,
                                             const std::vector<std::string>& joint_names,
                                             const Eigen::Ref<const Eigen::VectorXd>& joint_values) {
            self.setState(joint_names, joint_values);
        }, "joint_names"_a, "joint_values"_a)
        // Jacobian methods
        .def("getJacobian", [](const tsg::StateSolver& self,
                               const Eigen::Ref<const Eigen::VectorXd>& joint_values,
                               const std::string& link_name) {
            return self.getJacobian(joint_values, link_name);
        }, "joint_values"_a, "link_name"_a)
        // Name getters
        .def("getJointNames", &tsg::StateSolver::getJointNames)
        .def("getFloatingJointNames", &tsg::StateSolver::getFloatingJointNames)
        .def("getActiveJointNames", &tsg::StateSolver::getActiveJointNames)
        .def("getBaseLinkName", &tsg::StateSolver::getBaseLinkName)
        .def("getLinkNames", &tsg::StateSolver::getLinkNames)
        .def("getActiveLinkNames", &tsg::StateSolver::getActiveLinkNames)
        .def("getStaticLinkNames", &tsg::StateSolver::getStaticLinkNames)
        // Link queries
        .def("isActiveLinkName", &tsg::StateSolver::isActiveLinkName, "link_name"_a)
        .def("hasLinkName", &tsg::StateSolver::hasLinkName, "link_name"_a)
        // Transform getters
        .def("getLinkTransform", &tsg::StateSolver::getLinkTransform, "link_name"_a)
        .def("getRelativeLinkTransform", &tsg::StateSolver::getRelativeLinkTransform,
             "from_link_name"_a, "to_link_name"_a)
        .def("getLimits", &tsg::StateSolver::getLimits);

    // ========== MutableStateSolver (abstract, inherits StateSolver) ==========
    nb::class_<tsg::MutableStateSolver, tsg::StateSolver>(m, "MutableStateSolver")
        .def("setRevision", &tsg::MutableStateSolver::setRevision, "revision"_a)
        .def("getRevision", &tsg::MutableStateSolver::getRevision)
        .def("addLink", &tsg::MutableStateSolver::addLink, "link"_a, "joint"_a)
        .def("moveLink", &tsg::MutableStateSolver::moveLink, "joint"_a)
        .def("removeLink", &tsg::MutableStateSolver::removeLink, "name"_a)
        .def("replaceJoint", &tsg::MutableStateSolver::replaceJoint, "joint"_a)
        .def("removeJoint", &tsg::MutableStateSolver::removeJoint, "name"_a)
        .def("moveJoint", &tsg::MutableStateSolver::moveJoint, "name"_a, "parent_link"_a)
        .def("changeJointOrigin", &tsg::MutableStateSolver::changeJointOrigin, "name"_a, "new_origin"_a)
        .def("changeJointPositionLimits", &tsg::MutableStateSolver::changeJointPositionLimits,
             "name"_a, "lower"_a, "upper"_a)
        .def("changeJointVelocityLimits", &tsg::MutableStateSolver::changeJointVelocityLimits,
             "name"_a, "limit"_a)
        .def("changeJointAccelerationLimits", &tsg::MutableStateSolver::changeJointAccelerationLimits,
             "name"_a, "limit"_a)
        .def("changeJointJerkLimits", &tsg::MutableStateSolver::changeJointJerkLimits,
             "name"_a, "limit"_a);

    // ========== KDLStateSolver (concrete) ==========
    nb::class_<tsg::KDLStateSolver, tsg::StateSolver>(m, "KDLStateSolver")
        .def(nb::init<const tsg::SceneGraph&>(), "scene_graph"_a)
        .def("clone", [](const tsg::KDLStateSolver& self) {
            return self.clone();
        });

    // ========== OFKTStateSolver (concrete, mutable) ==========
    nb::class_<tsg::OFKTStateSolver, tsg::MutableStateSolver>(m, "OFKTStateSolver")
        .def(nb::init<const tsg::SceneGraph&>(), "scene_graph"_a)
        .def("clone", [](const tsg::OFKTStateSolver& self) {
            return self.clone();
        });
}
