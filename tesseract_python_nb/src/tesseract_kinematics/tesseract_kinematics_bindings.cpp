/**
 * @file tesseract_kinematics_bindings.cpp
 * @brief nanobind bindings for tesseract_kinematics
 */

#include "tesseract_nb.h"
#include <nanobind/stl/map.h>
#include <nanobind/stl/unique_ptr.h>
#include <nanobind/stl/set.h>

// tesseract_state_solver - need full definition for JointGroup
#include <tesseract_state_solver/state_solver.h>

// tesseract_kinematics core
#include <tesseract_kinematics/core/forward_kinematics.h>
#include <tesseract_kinematics/core/inverse_kinematics.h>
#include <tesseract_kinematics/core/joint_group.h>
#include <tesseract_kinematics/core/kinematic_group.h>
#include <tesseract_kinematics/core/types.h>
#include <tesseract_kinematics/core/kinematics_plugin_factory.h>

// tesseract_scene_graph
#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/scene_state.h>

// tesseract_common
#include <tesseract_common/kinematic_limits.h>

namespace tk = tesseract_kinematics;
namespace tsg = tesseract_scene_graph;

NB_MODULE(_tesseract_kinematics, m) {
    m.doc() = "tesseract_kinematics Python bindings";

    // ========== URParameters ==========
    nb::class_<tk::URParameters>(m, "URParameters")
        .def(nb::init<>())
        .def(nb::init<double, double, double, double, double, double>(),
             "d1"_a, "a2"_a, "a3"_a, "d4"_a, "d5"_a, "d6"_a)
        .def_rw("d1", &tk::URParameters::d1)
        .def_rw("a2", &tk::URParameters::a2)
        .def_rw("a3", &tk::URParameters::a3)
        .def_rw("d4", &tk::URParameters::d4)
        .def_rw("d5", &tk::URParameters::d5)
        .def_rw("d6", &tk::URParameters::d6);

    // UR parameter constants
    m.attr("UR10Parameters") = tk::UR10Parameters;
    m.attr("UR5Parameters") = tk::UR5Parameters;
    m.attr("UR3Parameters") = tk::UR3Parameters;
    m.attr("UR10eParameters") = tk::UR10eParameters;
    m.attr("UR5eParameters") = tk::UR5eParameters;
    m.attr("UR3eParameters") = tk::UR3eParameters;

    // ========== KinGroupIKInput ==========
    nb::class_<tk::KinGroupIKInput>(m, "KinGroupIKInput")
        .def(nb::init<>())
        .def(nb::init<const Eigen::Isometry3d&, std::string, std::string>(),
             "pose"_a, "working_frame"_a, "tip_link_name"_a)
        .def_rw("pose", &tk::KinGroupIKInput::pose)
        .def_rw("working_frame", &tk::KinGroupIKInput::working_frame)
        .def_rw("tip_link_name", &tk::KinGroupIKInput::tip_link_name);

    // ========== ForwardKinematics (abstract) ==========
    nb::class_<tk::ForwardKinematics>(m, "ForwardKinematics")
        .def("calcFwdKin", [](const tk::ForwardKinematics& self,
                              const Eigen::Ref<const Eigen::VectorXd>& joint_angles) {
            auto result = self.calcFwdKin(joint_angles);
            // Convert TransformMap to std::map for Python
            std::map<std::string, Eigen::Isometry3d> py_result;
            for (const auto& p : result) {
                py_result[p.first] = p.second;
            }
            return py_result;
        }, "joint_angles"_a)
        .def("calcJacobian", &tk::ForwardKinematics::calcJacobian, "joint_angles"_a, "link_name"_a)
        .def("getBaseLinkName", &tk::ForwardKinematics::getBaseLinkName)
        .def("getJointNames", &tk::ForwardKinematics::getJointNames)
        .def("getTipLinkNames", &tk::ForwardKinematics::getTipLinkNames)
        .def("numJoints", &tk::ForwardKinematics::numJoints)
        .def("getSolverName", &tk::ForwardKinematics::getSolverName);

    // ========== InverseKinematics (abstract) ==========
    nb::class_<tk::InverseKinematics>(m, "InverseKinematics")
        .def("calcInvKin", [](const tk::InverseKinematics& self,
                              const std::map<std::string, Eigen::Isometry3d>& tip_link_poses,
                              const Eigen::Ref<const Eigen::VectorXd>& seed) {
            // Convert std::map to TransformMap
            tesseract_common::TransformMap poses;
            for (const auto& p : tip_link_poses) {
                poses[p.first] = p.second;
            }
            return self.calcInvKin(poses, seed);
        }, "tip_link_poses"_a, "seed"_a)
        .def("getJointNames", &tk::InverseKinematics::getJointNames)
        .def("numJoints", &tk::InverseKinematics::numJoints)
        .def("getBaseLinkName", &tk::InverseKinematics::getBaseLinkName)
        .def("getWorkingFrame", &tk::InverseKinematics::getWorkingFrame)
        .def("getTipLinkNames", &tk::InverseKinematics::getTipLinkNames)
        .def("getSolverName", &tk::InverseKinematics::getSolverName);

    // ========== JointGroup ==========
    nb::class_<tk::JointGroup>(m, "JointGroup")
        .def(nb::init<std::string, std::vector<std::string>, const tsg::SceneGraph&, const tsg::SceneState&>(),
             "name"_a, "joint_names"_a, "scene_graph"_a, "scene_state"_a)
        .def("calcFwdKin", [](const tk::JointGroup& self,
                              const Eigen::Ref<const Eigen::VectorXd>& joint_angles) {
            auto result = self.calcFwdKin(joint_angles);
            std::map<std::string, Eigen::Isometry3d> py_result;
            for (const auto& p : result) {
                py_result[p.first] = p.second;
            }
            return py_result;
        }, "joint_angles"_a)
        .def("calcJacobian", [](const tk::JointGroup& self,
                                const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                                const std::string& link_name) {
            return self.calcJacobian(joint_angles, link_name);
        }, "joint_angles"_a, "link_name"_a)
        .def("calcJacobianWithPoint", [](const tk::JointGroup& self,
                                          const Eigen::Ref<const Eigen::VectorXd>& joint_angles,
                                          const std::string& link_name,
                                          const Eigen::Vector3d& link_point) {
            return self.calcJacobian(joint_angles, link_name, link_point);
        }, "joint_angles"_a, "link_name"_a, "link_point"_a)
        .def("getJointNames", &tk::JointGroup::getJointNames)
        .def("getLinkNames", &tk::JointGroup::getLinkNames)
        .def("getActiveLinkNames", &tk::JointGroup::getActiveLinkNames)
        .def("getStaticLinkNames", &tk::JointGroup::getStaticLinkNames)
        .def("isActiveLinkName", &tk::JointGroup::isActiveLinkName, "link_name"_a)
        .def("hasLinkName", &tk::JointGroup::hasLinkName, "link_name"_a)
        .def("getLimits", &tk::JointGroup::getLimits)
        .def("setLimits", &tk::JointGroup::setLimits, "limits"_a)
        .def("getRedundancyCapableJointIndices", &tk::JointGroup::getRedundancyCapableJointIndices)
        .def("numJoints", &tk::JointGroup::numJoints)
        .def("getBaseLinkName", &tk::JointGroup::getBaseLinkName)
        .def("getName", &tk::JointGroup::getName)
        .def("checkJoints", &tk::JointGroup::checkJoints, "vec"_a);

    // ========== KinematicGroup (extends JointGroup) ==========
    nb::class_<tk::KinematicGroup, tk::JointGroup>(m, "KinematicGroup")
        .def("calcInvKin", [](const tk::KinematicGroup& self,
                              const tk::KinGroupIKInput& tip_link_pose,
                              const Eigen::Ref<const Eigen::VectorXd>& seed) {
            return self.calcInvKin(tip_link_pose, seed);
        }, "tip_link_pose"_a, "seed"_a)
        .def("calcInvKinMultiple", [](const tk::KinematicGroup& self,
                                       const std::vector<tk::KinGroupIKInput>& tip_link_poses,
                                       const Eigen::Ref<const Eigen::VectorXd>& seed) {
            tk::KinGroupIKInputs inputs;
            for (const auto& p : tip_link_poses) {
                inputs.push_back(p);
            }
            return self.calcInvKin(inputs, seed);
        }, "tip_link_poses"_a, "seed"_a)
        .def("getAllValidWorkingFrames", &tk::KinematicGroup::getAllValidWorkingFrames)
        .def("getAllPossibleTipLinkNames", &tk::KinematicGroup::getAllPossibleTipLinkNames)
        .def("getInverseKinematics", &tk::KinematicGroup::getInverseKinematics,
             nb::rv_policy::reference_internal);

    // ========== KinematicsPluginFactory ==========
    nb::class_<tk::KinematicsPluginFactory>(m, "KinematicsPluginFactory")
        .def(nb::init<>())
        .def("addSearchPath", &tk::KinematicsPluginFactory::addSearchPath, "path"_a)
        .def("getSearchPaths", &tk::KinematicsPluginFactory::getSearchPaths)
        .def("addSearchLibrary", &tk::KinematicsPluginFactory::addSearchLibrary, "library_name"_a)
        .def("getSearchLibraries", &tk::KinematicsPluginFactory::getSearchLibraries)
        .def("getDefaultFwdKinPlugin", &tk::KinematicsPluginFactory::getDefaultFwdKinPlugin, "group_name"_a)
        .def("getDefaultInvKinPlugin", &tk::KinematicsPluginFactory::getDefaultInvKinPlugin, "group_name"_a);
}
