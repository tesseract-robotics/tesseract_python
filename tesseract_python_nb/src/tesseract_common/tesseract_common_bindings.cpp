#include "tesseract_nb.h"

// tesseract_common headers
#include <tesseract_common/types.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_common/manipulator_info.h>
#include <tesseract_common/joint_state.h>
#include <tesseract_common/collision_margin_data.h>
#include <tesseract_common/allowed_collision_matrix.h>
#include <tesseract_common/kinematic_limits.h>
#include <tesseract_common/plugin_info.h>

// console_bridge
#include <console_bridge/console.h>

// Trampoline class for ResourceLocator
class PyResourceLocator : public tesseract_common::ResourceLocator {
public:
    NB_TRAMPOLINE(tesseract_common::ResourceLocator, 1);

    std::shared_ptr<tesseract_common::Resource> locateResource(const std::string& url) const override {
        NB_OVERRIDE_PURE(locateResource, url);
    }
};

// Trampoline class for OutputHandler
class PyOutputHandler : public console_bridge::OutputHandler {
public:
    NB_TRAMPOLINE(console_bridge::OutputHandler, 1);

    void log(const std::string& text, console_bridge::LogLevel level, const char* filename, int line) override {
        NB_OVERRIDE_PURE(log, text, level, filename, line);
    }
};

NB_MODULE(_tesseract_common, m) {
    m.doc() = "tesseract_common Python bindings (nanobind)";

    // ========== Eigen Type Aliases ==========
    // Note: Vector3d, VectorXd, MatrixXd are handled automatically by nanobind/eigen/dense.h
    // Isometry3d has a custom type caster in tesseract_nb.h (converts to/from 4x4 numpy arrays)
    // Only bind geometric types that need explicit bindings

    nb::class_<Eigen::Translation3d>(m, "Translation3d")
        .def(nb::init<double, double, double>())
        .def("__mul__", [](const Eigen::Translation3d& self, const Eigen::Isometry3d& other) {
            return self * other;
        });

    nb::class_<Eigen::Quaterniond>(m, "Quaterniond")
        .def(nb::init<double, double, double, double>())
        .def("w", [](const Eigen::Quaterniond& q) { return q.w(); })
        .def("x", [](const Eigen::Quaterniond& q) { return q.x(); })
        .def("y", [](const Eigen::Quaterniond& q) { return q.y(); })
        .def("z", [](const Eigen::Quaterniond& q) { return q.z(); });

    nb::class_<Eigen::AngleAxisd>(m, "AngleAxisd")
        .def(nb::init<double, const Eigen::Vector3d&>());

    // ========== Resource Types ==========
    // Note: In nanobind 2.x, shared_ptr holder is automatic - don't specify it
    nb::class_<tesseract_common::Resource>(m, "Resource")
        .def("isFile", &tesseract_common::Resource::isFile)
        .def("getUrl", &tesseract_common::Resource::getUrl)
        .def("getFilePath", &tesseract_common::Resource::getFilePath)
        .def("getResourceContents", [](tesseract_common::Resource& self) {
            std::vector<uint8_t> data = self.getResourceContents();
            return nb::bytes(reinterpret_cast<const char*>(data.data()), data.size());
        })
        .def("getResourceContentStream", &tesseract_common::Resource::getResourceContentStream);

    nb::class_<tesseract_common::BytesResource, tesseract_common::Resource>(m, "BytesResource")
        .def(nb::init<const std::string&, const std::vector<uint8_t>&>())
        .def("__init__", [](tesseract_common::BytesResource* self, const std::string& url, nb::bytes data) {
            std::vector<uint8_t> vec(data.size());
            std::memcpy(vec.data(), data.c_str(), data.size());
            new (self) tesseract_common::BytesResource(url, vec);
        });

    nb::class_<tesseract_common::SimpleLocatedResource, tesseract_common::Resource>(m, "SimpleLocatedResource");

    // ========== ResourceLocator Hierarchy ==========
    nb::class_<tesseract_common::ResourceLocator, PyResourceLocator>(m, "ResourceLocator")
        .def(nb::init<>())
        .def("locateResource", &tesseract_common::ResourceLocator::locateResource);

    nb::class_<tesseract_common::GeneralResourceLocator, tesseract_common::ResourceLocator>(m, "GeneralResourceLocator")
        .def(nb::init<>());

    // ========== ManipulatorInfo ==========
    nb::class_<tesseract_common::ManipulatorInfo>(m, "ManipulatorInfo")
        .def(nb::init<>())
        .def_rw("manipulator", &tesseract_common::ManipulatorInfo::manipulator)
        .def_rw("manipulator_ik_solver", &tesseract_common::ManipulatorInfo::manipulator_ik_solver)
        .def_rw("working_frame", &tesseract_common::ManipulatorInfo::working_frame)
        .def_rw("tcp_frame", &tesseract_common::ManipulatorInfo::tcp_frame)
        .def_prop_rw("tcp_offset",
            [](const tesseract_common::ManipulatorInfo& self) -> nb::object {
                if (self.tcp_offset.index() == 0) {
                    return nb::cast(std::get<std::string>(self.tcp_offset));
                } else {
                    return nb::cast(std::get<Eigen::Isometry3d>(self.tcp_offset));
                }
            },
            [](tesseract_common::ManipulatorInfo& self, nb::object value) {
                if (nb::isinstance<nb::str>(value)) {
                    self.tcp_offset = nb::cast<std::string>(value);
                } else {
                    self.tcp_offset = nb::cast<Eigen::Isometry3d>(value);
                }
            })
        .def("__repr__", [](const tesseract_common::ManipulatorInfo& self) {
            return "<ManipulatorInfo manipulator='" + self.manipulator + "'>";
        });

    // ========== JointState ==========
    nb::class_<tesseract_common::JointState>(m, "JointState")
        .def(nb::init<>())
        .def(nb::init<const std::vector<std::string>&, const Eigen::VectorXd&>())
        .def_rw("joint_names", &tesseract_common::JointState::joint_names)
        .def_rw("position", &tesseract_common::JointState::position)
        .def_rw("velocity", &tesseract_common::JointState::velocity)
        .def_rw("acceleration", &tesseract_common::JointState::acceleration)
        .def_rw("effort", &tesseract_common::JointState::effort)
        .def_rw("time", &tesseract_common::JointState::time);

    // ========== AllowedCollisionMatrix ==========
    nb::class_<tesseract_common::AllowedCollisionMatrix>(m, "AllowedCollisionMatrix")
        .def(nb::init<>())
        .def("addAllowedCollision",
             nb::overload_cast<const std::string&, const std::string&, const std::string&>(
                 &tesseract_common::AllowedCollisionMatrix::addAllowedCollision))
        .def("removeAllowedCollision",
             nb::overload_cast<const std::string&, const std::string&>(
                 &tesseract_common::AllowedCollisionMatrix::removeAllowedCollision))
        .def("isCollisionAllowed", &tesseract_common::AllowedCollisionMatrix::isCollisionAllowed)
        .def("clearAllowedCollisions", &tesseract_common::AllowedCollisionMatrix::clearAllowedCollisions)
        .def("getAllAllowedCollisions", &tesseract_common::AllowedCollisionMatrix::getAllAllowedCollisions)
        .def("insertAllowedCollisionMatrix", &tesseract_common::AllowedCollisionMatrix::insertAllowedCollisionMatrix);

    // ========== CollisionMarginData ==========
    nb::enum_<tesseract_common::CollisionMarginOverrideType>(m, "CollisionMarginOverrideType")
        .value("NONE", tesseract_common::CollisionMarginOverrideType::NONE)
        .value("REPLACE", tesseract_common::CollisionMarginOverrideType::REPLACE)
        .value("MODIFY", tesseract_common::CollisionMarginOverrideType::MODIFY)
        .value("OVERRIDE_DEFAULT_MARGIN", tesseract_common::CollisionMarginOverrideType::OVERRIDE_DEFAULT_MARGIN)
        .value("OVERRIDE_PAIR_MARGIN", tesseract_common::CollisionMarginOverrideType::OVERRIDE_PAIR_MARGIN)
        .value("MODIFY_PAIR_MARGIN", tesseract_common::CollisionMarginOverrideType::MODIFY_PAIR_MARGIN);

    nb::class_<tesseract_common::CollisionMarginData>(m, "CollisionMarginData")
        .def(nb::init<>())
        .def(nb::init<double>())
        .def("getDefaultCollisionMargin", &tesseract_common::CollisionMarginData::getDefaultCollisionMargin)
        .def("setDefaultCollisionMargin", &tesseract_common::CollisionMarginData::setDefaultCollisionMargin)
        .def("getPairCollisionMargin", &tesseract_common::CollisionMarginData::getPairCollisionMargin)
        .def("setPairCollisionMargin", &tesseract_common::CollisionMarginData::setPairCollisionMargin)
        .def("getMaxCollisionMargin", &tesseract_common::CollisionMarginData::getMaxCollisionMargin);

    // ========== KinematicLimits ==========
    nb::class_<tesseract_common::KinematicLimits>(m, "KinematicLimits")
        .def(nb::init<>())
        .def_rw("joint_limits", &tesseract_common::KinematicLimits::joint_limits)
        .def_rw("velocity_limits", &tesseract_common::KinematicLimits::velocity_limits)
        .def_rw("acceleration_limits", &tesseract_common::KinematicLimits::acceleration_limits);

    // ========== PluginInfo ==========
    nb::class_<tesseract_common::PluginInfo>(m, "PluginInfo")
        .def(nb::init<>())
        .def_rw("class_name", &tesseract_common::PluginInfo::class_name)
        .def_rw("config", &tesseract_common::PluginInfo::config);

    // ========== Console Bridge ==========
    nb::enum_<console_bridge::LogLevel>(m, "LogLevel")
        .value("CONSOLE_BRIDGE_LOG_DEBUG", console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG)
        .value("CONSOLE_BRIDGE_LOG_INFO", console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO)
        .value("CONSOLE_BRIDGE_LOG_WARN", console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_WARN)
        .value("CONSOLE_BRIDGE_LOG_ERROR", console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_ERROR)
        .value("CONSOLE_BRIDGE_LOG_NONE", console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_NONE);

    // Export log level constants at module level
    m.attr("CONSOLE_BRIDGE_LOG_DEBUG") = console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_DEBUG;
    m.attr("CONSOLE_BRIDGE_LOG_INFO") = console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_INFO;
    m.attr("CONSOLE_BRIDGE_LOG_WARN") = console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_WARN;
    m.attr("CONSOLE_BRIDGE_LOG_ERROR") = console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_ERROR;
    m.attr("CONSOLE_BRIDGE_LOG_NONE") = console_bridge::LogLevel::CONSOLE_BRIDGE_LOG_NONE;

    nb::class_<console_bridge::OutputHandler, PyOutputHandler>(m, "OutputHandler")
        .def(nb::init<>())
        .def("log", &console_bridge::OutputHandler::log);

    m.def("setLogLevel", &console_bridge::setLogLevel, "level"_a);
    m.def("getLogLevel", &console_bridge::getLogLevel);
    // Note: console_bridge::log is variadic and can't be bound directly
    // Use OutputHandler subclass for custom logging
    m.def("useOutputHandler", &console_bridge::useOutputHandler, "handler"_a);
    m.def("restorePreviousOutputHandler", &console_bridge::restorePreviousOutputHandler);

    // ========== STL Container Bindings ==========
    // Note: std::vector and std::map are automatically handled by nanobind/stl includes
    // No explicit bind_vector/bind_map needed when type casters are enabled
}
