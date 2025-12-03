/**
 * @file tesseract_collision_bindings.cpp
 * @brief nanobind bindings for tesseract_collision
 */

#include "tesseract_nb.h"

// tesseract_collision core
#include <tesseract_collision/core/types.h>
#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_collision/core/continuous_contact_manager.h>
#include <tesseract_collision/core/contact_managers_plugin_factory.h>

// tesseract_common for types
#include <tesseract_common/allowed_collision_matrix.h>
#include <tesseract_common/collision_margin_data.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_common/filesystem.h>
#include <tesseract_common/types.h>

// tesseract_geometry for collision objects
#include <tesseract_geometry/geometry.h>

namespace tc = tesseract_collision;
namespace tcommon = tesseract_common;
namespace tg = tesseract_geometry;

// Disable type caster for ContactResultVector so we can bind it as a class
NB_MAKE_OPAQUE(tc::ContactResultVector);

NB_MODULE(_tesseract_collision, m) {
    m.doc() = "tesseract_collision Python bindings";

    // ========== Enums ==========
    nb::enum_<tc::ContinuousCollisionType>(m, "ContinuousCollisionType")
        .value("CCType_None", tc::ContinuousCollisionType::CCType_None)
        .value("CCType_Time0", tc::ContinuousCollisionType::CCType_Time0)
        .value("CCType_Time1", tc::ContinuousCollisionType::CCType_Time1)
        .value("CCType_Between", tc::ContinuousCollisionType::CCType_Between);

    nb::enum_<tc::ContactTestType>(m, "ContactTestType")
        .value("FIRST", tc::ContactTestType::FIRST)
        .value("CLOSEST", tc::ContactTestType::CLOSEST)
        .value("ALL", tc::ContactTestType::ALL)
        .value("LIMITED", tc::ContactTestType::LIMITED);

    // SWIG-compatible module-level constants
    m.attr("ContactTestType_FIRST") = tc::ContactTestType::FIRST;
    m.attr("ContactTestType_CLOSEST") = tc::ContactTestType::CLOSEST;
    m.attr("ContactTestType_ALL") = tc::ContactTestType::ALL;
    m.attr("ContactTestType_LIMITED") = tc::ContactTestType::LIMITED;

    nb::enum_<tc::CollisionEvaluatorType>(m, "CollisionEvaluatorType")
        .value("NONE", tc::CollisionEvaluatorType::NONE)
        .value("DISCRETE", tc::CollisionEvaluatorType::DISCRETE)
        .value("LVS_DISCRETE", tc::CollisionEvaluatorType::LVS_DISCRETE)
        .value("CONTINUOUS", tc::CollisionEvaluatorType::CONTINUOUS)
        .value("LVS_CONTINUOUS", tc::CollisionEvaluatorType::LVS_CONTINUOUS);

    nb::enum_<tc::CollisionCheckProgramType>(m, "CollisionCheckProgramType")
        .value("ALL", tc::CollisionCheckProgramType::ALL)
        .value("ALL_EXCEPT_START", tc::CollisionCheckProgramType::ALL_EXCEPT_START)
        .value("ALL_EXCEPT_END", tc::CollisionCheckProgramType::ALL_EXCEPT_END)
        .value("START_ONLY", tc::CollisionCheckProgramType::START_ONLY)
        .value("END_ONLY", tc::CollisionCheckProgramType::END_ONLY)
        .value("INTERMEDIATE_ONLY", tc::CollisionCheckProgramType::INTERMEDIATE_ONLY);

    nb::enum_<tc::ACMOverrideType>(m, "ACMOverrideType")
        .value("NONE", tc::ACMOverrideType::NONE)
        .value("ASSIGN", tc::ACMOverrideType::ASSIGN)
        .value("AND", tc::ACMOverrideType::AND)
        .value("OR", tc::ACMOverrideType::OR);

    // ========== ContactResult ==========
    nb::class_<tc::ContactResult>(m, "ContactResult")
        .def(nb::init<>())
        .def_rw("distance", &tc::ContactResult::distance)
        .def_prop_rw("type_id",
            [](const tc::ContactResult& self) {
                return std::vector<int>{self.type_id[0], self.type_id[1]};
            },
            [](tc::ContactResult& self, const std::vector<int>& v) {
                if (v.size() >= 2) { self.type_id[0] = v[0]; self.type_id[1] = v[1]; }
            })
        .def_prop_rw("link_names",
            [](const tc::ContactResult& self) {
                return std::vector<std::string>{self.link_names[0], self.link_names[1]};
            },
            [](tc::ContactResult& self, const std::vector<std::string>& v) {
                if (v.size() >= 2) { self.link_names[0] = v[0]; self.link_names[1] = v[1]; }
            })
        .def_prop_rw("shape_id",
            [](const tc::ContactResult& self) {
                return std::vector<int>{self.shape_id[0], self.shape_id[1]};
            },
            [](tc::ContactResult& self, const std::vector<int>& v) {
                if (v.size() >= 2) { self.shape_id[0] = v[0]; self.shape_id[1] = v[1]; }
            })
        .def_prop_rw("nearest_points",
            [](const tc::ContactResult& self) {
                std::vector<Eigen::Vector3d> pts;
                pts.push_back(self.nearest_points[0]);
                pts.push_back(self.nearest_points[1]);
                return pts;
            },
            [](tc::ContactResult& self, const std::vector<Eigen::Vector3d>& v) {
                if (v.size() >= 2) { self.nearest_points[0] = v[0]; self.nearest_points[1] = v[1]; }
            })
        .def_prop_rw("transform",
            [](const tc::ContactResult& self) {
                std::vector<Eigen::Isometry3d> tfs;
                tfs.push_back(self.transform[0]);
                tfs.push_back(self.transform[1]);
                return tfs;
            },
            [](tc::ContactResult& self, const std::vector<Eigen::Isometry3d>& v) {
                if (v.size() >= 2) { self.transform[0] = v[0]; self.transform[1] = v[1]; }
            })
        .def_rw("normal", &tc::ContactResult::normal)
        .def_rw("single_contact_point", &tc::ContactResult::single_contact_point)
        .def("clear", &tc::ContactResult::clear);

    // ========== ContactResultVector ==========
    // ContactResultVector uses Eigen aligned_allocator, so we need to bind manually
    nb::class_<tc::ContactResultVector>(m, "ContactResultVector")
        .def(nb::init<>())
        .def("__len__", [](const tc::ContactResultVector& v) { return v.size(); })
        .def("__getitem__", [](const tc::ContactResultVector& v, size_t i) -> const tc::ContactResult& {
            if (i >= v.size()) throw nb::index_error();
            return v[i];
        }, nb::rv_policy::reference_internal)
        .def("append", [](tc::ContactResultVector& v, const tc::ContactResult& item) {
            v.push_back(item);
        })
        .def("clear", [](tc::ContactResultVector& v) { v.clear(); });

    // ========== ContactResultMap ==========
    nb::class_<tc::ContactResultMap>(m, "ContactResultMap")
        .def(nb::init<>())
        .def("count", &tc::ContactResultMap::count)
        .def("size", &tc::ContactResultMap::size)
        .def("empty", &tc::ContactResultMap::empty)
        .def("clear", &tc::ContactResultMap::clear)
        .def("release", &tc::ContactResultMap::release)
        .def("getSummary", &tc::ContactResultMap::getSummary)
        .def("flattenCopyResults", [](const tc::ContactResultMap& self) {
            tc::ContactResultVector v;
            self.flattenCopyResults(v);
            return v;
        })
        .def("flattenMoveResults", [](tc::ContactResultMap& self, tc::ContactResultVector& v) {
            self.flattenMoveResults(v);
        }, "results"_a)
        .def("__len__", &tc::ContactResultMap::size);

    // ========== ContactRequest ==========
    nb::class_<tc::ContactRequest>(m, "ContactRequest")
        .def(nb::init<>())
        .def(nb::init<tc::ContactTestType>(), "type"_a)
        .def_rw("type", &tc::ContactRequest::type)
        .def_rw("calculate_penetration", &tc::ContactRequest::calculate_penetration)
        .def_rw("calculate_distance", &tc::ContactRequest::calculate_distance)
        .def_rw("contact_limit", &tc::ContactRequest::contact_limit);

    // ========== ContactManagerConfig ==========
    // Note: margin_data and acm are complex types from tesseract_common
    // Simplified binding without direct access to those members
    nb::class_<tc::ContactManagerConfig>(m, "ContactManagerConfig")
        .def(nb::init<>())
        .def(nb::init<double>(), "default_margin"_a)
        .def_rw("margin_data_override_type", &tc::ContactManagerConfig::margin_data_override_type)
        .def_rw("acm_override_type", &tc::ContactManagerConfig::acm_override_type);

    // ========== CollisionCheckConfig ==========
    nb::class_<tc::CollisionCheckConfig>(m, "CollisionCheckConfig")
        .def(nb::init<>())
        .def_rw("contact_request", &tc::CollisionCheckConfig::contact_request)
        .def_rw("type", &tc::CollisionCheckConfig::type)
        .def_rw("longest_valid_segment_length", &tc::CollisionCheckConfig::longest_valid_segment_length)
        .def_rw("check_program_mode", &tc::CollisionCheckConfig::check_program_mode);

    // ========== DiscreteContactManager (abstract, expose key methods) ==========
    nb::class_<tc::DiscreteContactManager>(m, "DiscreteContactManager")
        .def("getName", &tc::DiscreteContactManager::getName)
        .def("hasCollisionObject", &tc::DiscreteContactManager::hasCollisionObject, "name"_a)
        .def("removeCollisionObject", &tc::DiscreteContactManager::removeCollisionObject, "name"_a)
        .def("enableCollisionObject", &tc::DiscreteContactManager::enableCollisionObject, "name"_a)
        .def("disableCollisionObject", &tc::DiscreteContactManager::disableCollisionObject, "name"_a)
        .def("isCollisionObjectEnabled", &tc::DiscreteContactManager::isCollisionObjectEnabled, "name"_a)
        .def("setCollisionObjectsTransform",
             [](tc::DiscreteContactManager& self, const std::string& name, const Eigen::Isometry3d& pose) {
                 self.setCollisionObjectsTransform(name, pose);
             }, "name"_a, "pose"_a)
        .def("setCollisionObjectsTransform",
             [](tc::DiscreteContactManager& self, const tcommon::TransformMap& transforms) {
                 self.setCollisionObjectsTransform(transforms);
             }, "transforms"_a)
        .def("getCollisionObjects", &tc::DiscreteContactManager::getCollisionObjects)
        .def("setActiveCollisionObjects", &tc::DiscreteContactManager::setActiveCollisionObjects, "names"_a)
        .def("getActiveCollisionObjects", &tc::DiscreteContactManager::getActiveCollisionObjects)
        .def("setDefaultCollisionMarginData", &tc::DiscreteContactManager::setDefaultCollisionMarginData,
             "default_collision_margin"_a)
        .def("setPairCollisionMarginData", &tc::DiscreteContactManager::setPairCollisionMarginData,
             "name1"_a, "name2"_a, "collision_margin"_a)
        .def("setCollisionMarginData", [](tc::DiscreteContactManager& self,
                                          const tcommon::CollisionMarginData& margin_data) {
            self.setCollisionMarginData(margin_data, tcommon::CollisionMarginOverrideType::REPLACE);
        }, "collision_margin_data"_a)
        .def("getCollisionMarginData", &tc::DiscreteContactManager::getCollisionMarginData)
        .def("addCollisionObject",
             [](tc::DiscreteContactManager& self, const std::string& name, int mask_id,
                const std::vector<std::shared_ptr<const tg::Geometry>>& shapes,
                const tcommon::VectorIsometry3d& shape_poses, bool enabled) {
                 return self.addCollisionObject(name, mask_id, shapes, shape_poses, enabled);
             }, "name"_a, "mask_id"_a, "shapes"_a, "shape_poses"_a, "enabled"_a = true)
        .def("getCollisionObjectGeometries", &tc::DiscreteContactManager::getCollisionObjectGeometries, "name"_a)
        .def("getCollisionObjectGeometriesTransforms", &tc::DiscreteContactManager::getCollisionObjectGeometriesTransforms, "name"_a)
        .def("contactTest", &tc::DiscreteContactManager::contactTest, "collisions"_a, "request"_a);

    // ========== ContinuousContactManager (abstract, expose key methods) ==========
    nb::class_<tc::ContinuousContactManager>(m, "ContinuousContactManager")
        .def("getName", &tc::ContinuousContactManager::getName)
        .def("hasCollisionObject", &tc::ContinuousContactManager::hasCollisionObject, "name"_a)
        .def("removeCollisionObject", &tc::ContinuousContactManager::removeCollisionObject, "name"_a)
        .def("enableCollisionObject", &tc::ContinuousContactManager::enableCollisionObject, "name"_a)
        .def("disableCollisionObject", &tc::ContinuousContactManager::disableCollisionObject, "name"_a)
        .def("isCollisionObjectEnabled", &tc::ContinuousContactManager::isCollisionObjectEnabled, "name"_a)
        .def("setCollisionObjectsTransform",
             [](tc::ContinuousContactManager& self, const std::string& name, const Eigen::Isometry3d& pose) {
                 self.setCollisionObjectsTransform(name, pose);
             }, "name"_a, "pose"_a)
        .def("setCollisionObjectsTransformCast",
             [](tc::ContinuousContactManager& self, const std::string& name,
                const Eigen::Isometry3d& pose1, const Eigen::Isometry3d& pose2) {
                 self.setCollisionObjectsTransform(name, pose1, pose2);
             }, "name"_a, "pose1"_a, "pose2"_a)
        .def("getCollisionObjects", &tc::ContinuousContactManager::getCollisionObjects)
        .def("setActiveCollisionObjects", &tc::ContinuousContactManager::setActiveCollisionObjects, "names"_a)
        .def("getActiveCollisionObjects", &tc::ContinuousContactManager::getActiveCollisionObjects)
        .def("setDefaultCollisionMarginData", &tc::ContinuousContactManager::setDefaultCollisionMarginData,
             "default_collision_margin"_a)
        .def("setPairCollisionMarginData", &tc::ContinuousContactManager::setPairCollisionMarginData,
             "name1"_a, "name2"_a, "collision_margin"_a)
        .def("contactTest", &tc::ContinuousContactManager::contactTest, "collisions"_a, "request"_a);

    // ========== ContactManagersPluginFactory ==========
    nb::class_<tc::ContactManagersPluginFactory>(m, "ContactManagersPluginFactory")
        .def(nb::init<>())
        .def("__init__", [](tc::ContactManagersPluginFactory* self,
                            const tcommon::fs::path& config_path,
                            const tcommon::ResourceLocator& locator) {
            new (self) tc::ContactManagersPluginFactory(config_path, locator);
        }, "config_path"_a, "locator"_a)
        .def("__init__", [](tc::ContactManagersPluginFactory* self,
                            const std::string& config,
                            const tcommon::ResourceLocator& locator) {
            new (self) tc::ContactManagersPluginFactory(config, locator);
        }, "config"_a, "locator"_a)
        .def("addSearchPath", &tc::ContactManagersPluginFactory::addSearchPath, "path"_a)
        .def("getSearchPaths", &tc::ContactManagersPluginFactory::getSearchPaths)
        .def("addSearchLibrary", &tc::ContactManagersPluginFactory::addSearchLibrary, "library_name"_a)
        .def("getSearchLibraries", &tc::ContactManagersPluginFactory::getSearchLibraries)
        .def("hasDiscreteContactManagerPlugins", &tc::ContactManagersPluginFactory::hasDiscreteContactManagerPlugins)
        .def("getDefaultDiscreteContactManagerPlugin", &tc::ContactManagersPluginFactory::getDefaultDiscreteContactManagerPlugin)
        .def("hasContinuousContactManagerPlugins", &tc::ContactManagersPluginFactory::hasContinuousContactManagerPlugins)
        .def("getDefaultContinuousContactManagerPlugin", &tc::ContactManagersPluginFactory::getDefaultContinuousContactManagerPlugin)
        .def("createDiscreteContactManager",
             [](const tc::ContactManagersPluginFactory& self, const std::string& name) {
                 return self.createDiscreteContactManager(name);
             }, "name"_a)
        .def("createContinuousContactManager",
             [](const tc::ContactManagersPluginFactory& self, const std::string& name) {
                 return self.createContinuousContactManager(name);
             }, "name"_a);
}
