/**
 * @file tesseract_task_composer_bindings.cpp
 * @brief nanobind bindings for tesseract_task_composer
 */

#include "tesseract_nb.h"
#include <nanobind/stl/unordered_map.h>
#include <nanobind/stl/chrono.h>
#include <nanobind/stl/optional.h>

// tesseract_task_composer core
#include <tesseract_task_composer/core/task_composer_context.h>
#include <tesseract_task_composer/core/task_composer_data_storage.h>
#include <tesseract_task_composer/core/task_composer_executor.h>
#include <tesseract_task_composer/core/task_composer_future.h>
#include <tesseract_task_composer/core/task_composer_node.h>
#include <tesseract_task_composer/core/task_composer_node_info.h>
#include <tesseract_task_composer/core/task_composer_keys.h>
#include <tesseract_task_composer/core/task_composer_plugin_factory.h>
#include <tesseract_task_composer/core/task_composer_server.h>

// tesseract_task_composer taskflow
#include <tesseract_task_composer/taskflow/taskflow_task_composer_executor.h>

// tesseract_common
#include <tesseract_common/any_poly.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_common/filesystem.h>

// tesseract_environment for Environment wrapper
#include <tesseract_environment/environment.h>

// tesseract_command_language for CompositeInstruction and ProfileDictionary wrappers
#include <tesseract_command_language/composite_instruction.h>
#include <tesseract_command_language/profile_dictionary.h>

namespace tp = tesseract_planning;
namespace te = tesseract_environment;
namespace tc = tesseract_common;

NB_MODULE(_tesseract_task_composer, m) {
    m.doc() = "tesseract_task_composer Python bindings";

    // Import tesseract_common module to ensure type hierarchy is available
    nb::module_::import_("tesseract_robotics.tesseract_common._tesseract_common");

    // ========== TaskComposerKeys ==========
    nb::class_<tp::TaskComposerKeys>(m, "TaskComposerKeys")
        .def(nb::init<>())
        .def("add", nb::overload_cast<const std::string&, std::string>(&tp::TaskComposerKeys::add), "port"_a, "key"_a)
        .def("addKeys", nb::overload_cast<const std::string&, std::vector<std::string>>(&tp::TaskComposerKeys::add), "port"_a, "keys"_a)
        .def("get", [](const tp::TaskComposerKeys& self, const std::string& port) -> std::string {
            return self.get<std::string>(port);
        }, "port"_a)
        .def("has", &tp::TaskComposerKeys::has, "port"_a)
        .def("empty", &tp::TaskComposerKeys::empty)
        .def("size", &tp::TaskComposerKeys::size)
        .def("data", &tp::TaskComposerKeys::data);

    // ========== TaskComposerDataStorage ==========
    // Use shared_ptr for proper lifetime management with executor.run()
    nb::class_<tp::TaskComposerDataStorage>(m, "TaskComposerDataStorage")
        .def(nb::init<>())
        .def("getName", &tp::TaskComposerDataStorage::getName)
        .def("setName", &tp::TaskComposerDataStorage::setName, "name"_a)
        .def("hasKey", &tp::TaskComposerDataStorage::hasKey, "key"_a)
        .def("setData", &tp::TaskComposerDataStorage::setData, "key"_a, "data"_a)
        .def("getData", nb::overload_cast<const std::string&>(&tp::TaskComposerDataStorage::getData, nb::const_), "key"_a)
        .def("removeData", &tp::TaskComposerDataStorage::removeData, "key"_a);

    // Factory function that returns shared_ptr (useful when passing to executor.run)
    m.def("createTaskComposerDataStorage", []() {
        return std::make_shared<tp::TaskComposerDataStorage>();
    }, "Create a TaskComposerDataStorage as shared_ptr");

    // ========== TaskComposerNodeInfo ==========
    nb::class_<tp::TaskComposerNodeInfo>(m, "TaskComposerNodeInfo")
        .def_prop_ro("name", [](const tp::TaskComposerNodeInfo& self) { return self.name; })
        .def_prop_ro("return_value", [](const tp::TaskComposerNodeInfo& self) { return self.return_value; })
        .def_prop_ro("status_code", [](const tp::TaskComposerNodeInfo& self) { return self.status_code; })
        .def_prop_ro("status_message", [](const tp::TaskComposerNodeInfo& self) { return self.status_message; })
        .def_prop_ro("elapsed_time", [](const tp::TaskComposerNodeInfo& self) { return self.elapsed_time; });

    // ========== TaskComposerNodeInfoContainer ==========
    nb::class_<tp::TaskComposerNodeInfoContainer>(m, "TaskComposerNodeInfoContainer")
        .def(nb::init<>())
        .def("getAbortingNodeInfo", [](const tp::TaskComposerNodeInfoContainer& self) -> nb::object {
            auto uuid = self.getAbortingNode();
            if (uuid.is_nil())
                return nb::none();
            auto info = self.getInfo(uuid);
            if (!info)
                return nb::none();
            // Return a dict with the info
            nb::dict result;
            result["name"] = info->name;
            result["return_value"] = info->return_value;
            result["status_code"] = info->status_code;
            result["status_message"] = info->status_message;
            result["elapsed_time"] = info->elapsed_time;
            return result;
        })
        .def("getAllInfos", [](const tp::TaskComposerNodeInfoContainer& self) -> nb::list {
            nb::list result;
            auto info_map = self.getInfoMap();
            for (const auto& [uuid, info] : info_map) {
                if (info) {
                    nb::dict d;
                    d["name"] = info->name;
                    d["return_value"] = info->return_value;
                    d["status_code"] = info->status_code;
                    d["status_message"] = info->status_message;
                    d["elapsed_time"] = info->elapsed_time;
                    result.append(d);
                }
            }
            return result;
        });

    // ========== TaskComposerContext ==========
    nb::class_<tp::TaskComposerContext>(m, "TaskComposerContext")
        .def_prop_rw("name",
            [](const tp::TaskComposerContext& self) { return self.name; },
            [](tp::TaskComposerContext& self, const std::string& v) { self.name = v; })
        .def_prop_rw("dotgraph",
            [](const tp::TaskComposerContext& self) { return self.dotgraph; },
            [](tp::TaskComposerContext& self, bool v) { self.dotgraph = v; })
        .def_prop_rw("data_storage",
            [](const tp::TaskComposerContext& self) { return self.data_storage; },
            [](tp::TaskComposerContext& self, std::shared_ptr<tp::TaskComposerDataStorage> v) { self.data_storage = v; })
        .def("isAborted", &tp::TaskComposerContext::isAborted)
        .def("isSuccessful", &tp::TaskComposerContext::isSuccessful)
        .def_prop_ro("task_infos",
            [](const tp::TaskComposerContext& self) { return self.task_infos; });

    // ========== TaskComposerNode (abstract base) ==========
    nb::class_<tp::TaskComposerNode>(m, "TaskComposerNode")
        .def("getName", &tp::TaskComposerNode::getName)
        .def("getNamespace", &tp::TaskComposerNode::getNamespace)
        .def("getUUIDString", &tp::TaskComposerNode::getUUIDString)
        .def("isConditional", &tp::TaskComposerNode::isConditional)
        .def("getInputKeys", &tp::TaskComposerNode::getInputKeys, nb::rv_policy::reference)
        .def("getOutputKeys", &tp::TaskComposerNode::getOutputKeys, nb::rv_policy::reference)
        .def("setInputKeys", &tp::TaskComposerNode::setInputKeys, "input_keys"_a)
        .def("setOutputKeys", &tp::TaskComposerNode::setOutputKeys, "output_keys"_a);

    // ========== TaskComposerFuture ==========
    nb::class_<tp::TaskComposerFuture>(m, "TaskComposerFuture")
        .def_prop_rw("context",
            [](const tp::TaskComposerFuture& self) { return self.context; },
            [](tp::TaskComposerFuture& self, std::shared_ptr<tp::TaskComposerContext> v) { self.context = v; })
        .def("valid", &tp::TaskComposerFuture::valid)
        .def("ready", &tp::TaskComposerFuture::ready)
        .def("wait", &tp::TaskComposerFuture::wait)
        .def("waitFor", &tp::TaskComposerFuture::waitFor, "duration"_a);

    // ========== TaskComposerExecutor (abstract base) ==========
    nb::class_<tp::TaskComposerExecutor>(m, "TaskComposerExecutor")
        .def("getName", &tp::TaskComposerExecutor::getName)
        .def("run", [](tp::TaskComposerExecutor& self, const tp::TaskComposerNode& node,
                       std::shared_ptr<tp::TaskComposerDataStorage> data_storage, bool dotgraph) {
            return self.run(node, data_storage, dotgraph);
        }, "node"_a, "data_storage"_a, "dotgraph"_a = false)
        .def("getWorkerCount", &tp::TaskComposerExecutor::getWorkerCount)
        .def("getTaskCount", &tp::TaskComposerExecutor::getTaskCount);

    // ========== TaskflowTaskComposerExecutor ==========
    // Non-copyable (deleted copy/move). Don't specify inheritance (cross-module issue).
    // The public run() is inherited from TaskComposerExecutor base class.
    nb::class_<tp::TaskflowTaskComposerExecutor>(m, "TaskflowTaskComposerExecutor")
        .def(nb::init<std::string, size_t>(), "name"_a = "TaskflowExecutor",
             "num_threads"_a = std::thread::hardware_concurrency())
        .def(nb::init<size_t>(), "num_threads"_a)
        // Re-expose base class methods (public run is from TaskComposerExecutor)
        .def("getName", &tp::TaskflowTaskComposerExecutor::getName)
        .def("run", [](tp::TaskflowTaskComposerExecutor& self, const tp::TaskComposerNode& node,
                       std::shared_ptr<tp::TaskComposerDataStorage> data_storage, bool dotgraph) {
            // Cast to base class to call public run method
            return static_cast<tp::TaskComposerExecutor&>(self).run(node, data_storage, dotgraph);
        }, "node"_a, "data_storage"_a, "dotgraph"_a = false)
        .def("getWorkerCount", &tp::TaskflowTaskComposerExecutor::getWorkerCount)
        .def("getTaskCount", &tp::TaskflowTaskComposerExecutor::getTaskCount);

    // ========== TaskComposerPluginFactory ==========
    // Move-only class (deleted copy, has move). Use unique_ptr internally to handle moves.
    // Note: Cross-module inheritance with ResourceLocator - use nb::handle with manual type check
    nb::class_<tp::TaskComposerPluginFactory>(m, "TaskComposerPluginFactory")
        .def("__init__", [](tp::TaskComposerPluginFactory* self, const std::string& config_str, nb::handle locator_handle) {
            tc::fs::path config(config_str);
            auto common_module = nb::module_::import_("tesseract_robotics.tesseract_common._tesseract_common");
            auto grl_type = common_module.attr("GeneralResourceLocator");
            if (!nb::isinstance(locator_handle, grl_type)) {
                throw nb::type_error("locator must be a GeneralResourceLocator");
            }
            auto* locator = nb::cast<tc::GeneralResourceLocator*>(locator_handle);
            new (self) tp::TaskComposerPluginFactory(config, *locator);
        }, "config"_a, "locator"_a,
             "Create from config file path (string) and GeneralResourceLocator")
        .def("createTaskComposerExecutor", [](tp::TaskComposerPluginFactory& self, const std::string& name) {
            return self.createTaskComposerExecutor(name);
        }, "name"_a, nb::rv_policy::move,
           "Create a task composer executor by name")
        .def("createTaskComposerNode", [](tp::TaskComposerPluginFactory& self, const std::string& name) {
            return self.createTaskComposerNode(name);
        }, "name"_a, nb::rv_policy::move,
           "Create a task composer node by name")
        .def("hasTaskComposerExecutorPlugins", &tp::TaskComposerPluginFactory::hasTaskComposerExecutorPlugins)
        .def("hasTaskComposerNodePlugins", &tp::TaskComposerPluginFactory::hasTaskComposerNodePlugins)
        .def("getDefaultTaskComposerExecutorPlugin", &tp::TaskComposerPluginFactory::getDefaultTaskComposerExecutorPlugin)
        .def("getDefaultTaskComposerNodePlugin", &tp::TaskComposerPluginFactory::getDefaultTaskComposerNodePlugin);

    // Keep factory functions for backwards compatibility
    // Note: Cross-module inheritance with ResourceLocator - use nb::handle with manual type check
    m.def("createTaskComposerPluginFactory", [](const std::string& config_str, nb::handle locator_handle) {
        // Convert config string to path
        tc::fs::path config(config_str);

        // Get the GeneralResourceLocator type from the tesseract_common module
        auto common_module = nb::module_::import_("tesseract_robotics.tesseract_common._tesseract_common");
        auto grl_type = common_module.attr("GeneralResourceLocator");

        // Check if locator is a GeneralResourceLocator
        if (!nb::isinstance(locator_handle, grl_type)) {
            throw nb::type_error("locator must be a GeneralResourceLocator");
        }

        // Cast using the imported type
        auto* locator = nb::cast<tc::GeneralResourceLocator*>(locator_handle);
        return std::make_unique<tp::TaskComposerPluginFactory>(config, *locator);
    }, "config"_a, "locator"_a,
    "Create a TaskComposerPluginFactory from a config file path (string) and GeneralResourceLocator");

    // ========== AnyPoly ==========
    // Bind AnyPoly class for type-erased data storage
    nb::class_<tc::AnyPoly>(m, "AnyPoly")
        .def(nb::init<>())
        .def("isNull", &tc::AnyPoly::isNull)
        .def("getTypeName", [](const tc::AnyPoly& self) {
            if (self.isNull())
                return std::string("null");
            return std::string(self.getType().name());
        });

    // ========== AnyPoly wrapper functions for common types ==========
    // These wrap specific types into AnyPoly for use with TaskComposerDataStorage.setData()

    m.def("AnyPoly_wrap_CompositeInstruction", [](const tp::CompositeInstruction& ci) {
        return tc::AnyPoly(ci);
    }, "instruction"_a, "Wrap a CompositeInstruction into an AnyPoly");

    m.def("AnyPoly_wrap_ProfileDictionary", [](std::shared_ptr<tp::ProfileDictionary> pd) {
        return tc::AnyPoly(pd);
    }, "profiles"_a, "Wrap a ProfileDictionary shared_ptr into an AnyPoly");

    m.def("AnyPoly_wrap_EnvironmentConst", [](std::shared_ptr<const te::Environment> env) {
        return tc::AnyPoly(env);
    }, "environment"_a, "Wrap a const Environment shared_ptr into an AnyPoly");

    // ========== AnyPoly unwrap functions ==========
    m.def("AnyPoly_as_CompositeInstruction", [](const tc::AnyPoly& ap) -> tp::CompositeInstruction {
        return ap.as<tp::CompositeInstruction>();
    }, "any_poly"_a, "Extract a CompositeInstruction from an AnyPoly");
}
