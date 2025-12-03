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
#include <tesseract_environment/events.h>
#include <tesseract_environment/command.h>
#include <tesseract_environment/commands/remove_joint_command.h>
#include <tesseract_environment/commands/add_link_command.h>

// tesseract_scene_graph
#include <tesseract_scene_graph/graph.h>
#include <tesseract_scene_graph/scene_state.h>
#include <tesseract_scene_graph/link.h>
#include <tesseract_scene_graph/joint.h>

// tesseract_common
#include <tesseract_common/resource_locator.h>
#include <tesseract_common/manipulator_info.h>
#include <tesseract_common/filesystem.h>

// tesseract_srdf
#include <tesseract_srdf/srdf_model.h>

// tesseract_kinematics
#include <tesseract_kinematics/core/joint_group.h>
#include <tesseract_kinematics/core/kinematic_group.h>

// tesseract_collision
#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_collision/core/continuous_contact_manager.h>

// tesseract_state_solver - need full definition for getStateSolver return type
#include <tesseract_state_solver/state_solver.h>

namespace te = tesseract_environment;
namespace tsg = tesseract_scene_graph;
namespace tc = tesseract_common;
namespace tk = tesseract_kinematics;

// Wrapper for Python event callbacks
struct PyEventCallbackFn {
    nb::callable callback;
    void operator()(const te::Event& evt) const {
        callback(nb::cast(evt, nb::rv_policy::reference));
    }
};

NB_MODULE(_tesseract_environment, m) {
    m.doc() = "tesseract_environment Python bindings";

    // ========== Events enum ==========
    nb::enum_<te::Events>(m, "Events")
        .value("COMMAND_APPLIED", te::Events::COMMAND_APPLIED)
        .value("SCENE_STATE_CHANGED", te::Events::SCENE_STATE_CHANGED);

    // Export enum values with SWIG-compatible naming
    m.attr("Events_COMMAND_APPLIED") = te::Events::COMMAND_APPLIED;
    m.attr("Events_SCENE_STATE_CHANGED") = te::Events::SCENE_STATE_CHANGED;

    // ========== Event base class ==========
    nb::class_<te::Event>(m, "Event")
        .def_ro("type", &te::Event::type);

    // ========== CommandAppliedEvent ==========
    nb::class_<te::CommandAppliedEvent, te::Event>(m, "CommandAppliedEvent")
        .def_ro("revision", &te::CommandAppliedEvent::revision);

    // ========== SceneStateChangedEvent ==========
    nb::class_<te::SceneStateChangedEvent, te::Event>(m, "SceneStateChangedEvent")
        .def_prop_ro("state", [](const te::SceneStateChangedEvent& self) -> const tsg::SceneState& {
            return self.state;
        });

    // ========== Event cast functions (for Python to downcast Event to specific type) ==========
    m.def("cast_CommandAppliedEvent", [](const te::Event& evt) -> const te::CommandAppliedEvent& {
        return static_cast<const te::CommandAppliedEvent&>(evt);
    }, nb::rv_policy::reference, "Cast Event to CommandAppliedEvent");

    m.def("cast_SceneStateChangedEvent", [](const te::Event& evt) -> const te::SceneStateChangedEvent& {
        return static_cast<const te::SceneStateChangedEvent&>(evt);
    }, nb::rv_policy::reference, "Cast Event to SceneStateChangedEvent");

    // EventCallbackFn wrapper for Python callbacks
    nb::class_<PyEventCallbackFn>(m, "EventCallbackFn")
        .def(nb::init<nb::callable>(), "callback"_a);

    // ========== Command base class ==========
    nb::class_<te::Command>(m, "Command")
        .def("getType", &te::Command::getType);

    // ========== RemoveJointCommand ==========
    nb::class_<te::RemoveJointCommand, te::Command>(m, "RemoveJointCommand")
        .def(nb::init<std::string>(), "joint_name"_a)
        .def("getJointName", &te::RemoveJointCommand::getJointName);

    // ========== AddLinkCommand ==========
    nb::class_<te::AddLinkCommand, te::Command>(m, "AddLinkCommand")
        .def(nb::init<const tsg::Link&, bool>(), "link"_a, "replace_allowed"_a = false)
        .def(nb::init<const tsg::Link&, const tsg::Joint&, bool>(),
             "link"_a, "joint"_a, "replace_allowed"_a = false)
        .def("getLink", &te::AddLinkCommand::getLink)
        .def("getJoint", &te::AddLinkCommand::getJoint)
        .def("replaceAllowed", &te::AddLinkCommand::replaceAllowed);

    // ========== Environment ==========
    nb::class_<te::Environment>(m, "Environment")
        .def(nb::init<>())
        // Init methods
        .def("init", [](te::Environment& self, const tsg::SceneGraph& scene_graph) {
            return self.init(scene_graph);
        }, "scene_graph"_a)
        // Init with scene_graph + srdf (makes a copy of srdf into shared_ptr)
        .def("init", [](te::Environment& self, const tsg::SceneGraph& scene_graph,
                        const tesseract_srdf::SRDFModel& srdf) {
            auto srdf_ptr = std::make_shared<const tesseract_srdf::SRDFModel>(srdf);
            return self.init(scene_graph, srdf_ptr);
        }, "scene_graph"_a, "srdf"_a)
        .def("initFromUrdf", [](te::Environment& self, const std::string& urdf_string,
                                 const std::shared_ptr<const tc::ResourceLocator>& locator) {
            return self.init(urdf_string, locator);
        }, "urdf_string"_a, "locator"_a)
        .def("initFromUrdfSrdf", [](te::Environment& self, const std::string& urdf_string,
                                     const std::string& srdf_string,
                                     const std::shared_ptr<const tc::ResourceLocator>& locator) {
            return self.init(urdf_string, srdf_string, locator);
        }, "urdf_string"_a, "srdf_string"_a, "locator"_a)
        // Init from paths (SWIG compatibility - accepts FilesystemPath or string)
        .def("init", [](te::Environment& self, const std::string& urdf_path,
                        const std::string& srdf_path,
                        const std::shared_ptr<const tc::ResourceLocator>& locator) {
            return self.init(tc::fs::path(urdf_path), tc::fs::path(srdf_path), locator);
        }, "urdf_path"_a, "srdf_path"_a, "locator"_a)
        .def("init", [](te::Environment& self, const std::string& urdf_path,
                        const std::shared_ptr<const tc::ResourceLocator>& locator) {
            return self.init(tc::fs::path(urdf_path), locator);
        }, "urdf_path"_a, "locator"_a)
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
        // setState with (names, values) - SWIG compatibility
        .def("setState", [](te::Environment& self,
                            const std::vector<std::string>& joint_names,
                            const Eigen::Ref<const Eigen::VectorXd>& joint_values) {
            self.setState(joint_names, joint_values);
        }, "joint_names"_a, "joint_values"_a)
        // Event callbacks
        .def("addEventCallback", [](te::Environment& self, std::size_t hash, const PyEventCallbackFn& fn) {
            self.addEventCallback(hash, fn);
        }, "hash"_a, "fn"_a)
        .def("removeEventCallback", &te::Environment::removeEventCallback, "hash"_a)
        .def("clearEventCallbacks", &te::Environment::clearEventCallbacks)
        // Commands - RemoveJointCommand
        .def("applyCommand", [](te::Environment& self, const te::RemoveJointCommand& cmd) {
            auto cmd_ptr = std::make_shared<te::RemoveJointCommand>(cmd.getJointName());
            return self.applyCommand(cmd_ptr);
        }, "command"_a)
        // Commands - AddLinkCommand
        .def("applyCommand", [](te::Environment& self, const te::AddLinkCommand& cmd) {
            std::shared_ptr<te::Command> cmd_ptr;
            if (cmd.getJoint() != nullptr) {
                cmd_ptr = std::make_shared<te::AddLinkCommand>(*cmd.getLink(), *cmd.getJoint(), cmd.replaceAllowed());
            } else {
                cmd_ptr = std::make_shared<te::AddLinkCommand>(*cmd.getLink(), cmd.replaceAllowed());
            }
            return self.applyCommand(cmd_ptr);
        }, "command"_a)
        // State solver
        .def("getStateSolver", [](const te::Environment& self) {
            return self.getStateSolver();
        })
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
        .def("getJointGroup", [](const te::Environment& self, const std::string& group_name) -> const tk::JointGroup& {
            auto ptr = self.getJointGroup(group_name);
            if (!ptr) throw std::runtime_error("Failed to get joint group: " + group_name);
            return *ptr;
        }, "group_name"_a, nb::rv_policy::reference_internal)
        .def("getKinematicGroup", [](const te::Environment& self, const std::string& group_name,
                                      const std::string& ik_solver_name) -> const tk::KinematicGroup& {
            auto ptr = self.getKinematicGroup(group_name, ik_solver_name);
            if (!ptr) throw std::runtime_error("Failed to get kinematic group: " + group_name);
            return *ptr;
        }, "group_name"_a, "ik_solver_name"_a = "", nb::rv_policy::reference_internal)
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
