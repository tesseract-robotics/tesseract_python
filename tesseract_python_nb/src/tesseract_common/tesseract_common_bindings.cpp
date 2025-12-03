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

    std::shared_ptr<tesseract_common::Resource> locateResource(const std::string& url) override {
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
    nb::class_<Eigen::Isometry3d>(m, "Isometry3d")
        .def(nb::init<>())
        .def("matrix", [](const Eigen::Isometry3d& self) { return self.matrix(); })
        .def("translation", [](const Eigen::Isometry3d& self) { return self.translation(); })
        .def("linear", [](const Eigen::Isometry3d& self) { return self.linear(); })
        .def("__mul__", [](const Eigen::Isometry3d& self, const Eigen::Isometry3d& other) {
            return self * other;
        })
        .def("__repr__", [](const Eigen::Isometry3d& self) {
            std::ostringstream oss;
            oss << "Isometry3d(\n" << self.matrix() << "\n)";
            return oss.str();
        });

    nb::class_<Eigen::Translation3d>(m, "Translation3d")
        .def(nb::init<double, double, double>())
        .def("__mul__", [](const Eigen::Translation3d& self, const Eigen::Isometry3d& other) {
            return self * other;
        });

    nb::class_<Eigen::Quaterniond>(m, "Quaterniond")
        .def(nb::init<double, double, double, double>())
        .def("w", &Eigen::Quaterniond::w)
        .def("x", &Eigen::Quaterniond::x)
        .def("y", &Eigen::Quaterniond::y)
        .def("z", &Eigen::Quaterniond::z);

    nb::class_<Eigen::AngleAxisd>(m, "AngleAxisd")
        .def(nb::init<double, const Eigen::Vector3d&>());

    // Vector and Matrix types are automatically handled by nanobind/eigen
    // But we can add explicit bindings for documentation
    auto vector3d = nb::class_<Eigen::Vector3d>(m, "Vector3d");
    auto vectorxd = nb::class_<Eigen::VectorXd>(m, "VectorXd");
    auto matrixxd = nb::class_<Eigen::MatrixXd>(m, "MatrixXd");

    // ========== Resource Types ==========
    nb::class_<tesseract_common::Resource, std::shared_ptr<tesseract_common::Resource>>(m, "Resource")
        .def("isFile", &tesseract_common::Resource::isFile)
        .def("getUrl", &tesseract_common::Resource::getUrl)
        .def("getFilePath", &tesseract_common::Resource::getFilePath)
        .def("getResourceContents", [](tesseract_common::Resource& self) {
            std::vector<uint8_t> data = self.getResourceContents();
            return nb::bytes(reinterpret_cast<const char*>(data.data()), data.size());
        })
        .def("getResourceContentStream", &tesseract_common::Resource::getResourceContentStream);

    nb::class_<tesseract_common::BytesResource,
               tesseract_common::Resource,
               std::shared_ptr<tesseract_common::BytesResource>>(m, "BytesResource")
        .def(nb::init<const std::string&, const std::vector<uint8_t>&>())
        .def(nb::init([](const std::string& url, nb::bytes data) {
            std::vector<uint8_t> vec(data.size());
            std::memcpy(vec.data(), data.c_str(), data.size());
            return std::make_shared<tesseract_common::BytesResource>(url, vec);
        }));

    nb::class_<tesseract_common::SimpleLocatedResource,
               tesseract_common::Resource,
               std::shared_ptr<tesseract_common::SimpleLocatedResource>>(m, "SimpleLocatedResource");

    // ========== ResourceLocator Hierarchy ==========
    nb::class_<tesseract_common::ResourceLocator,
               PyResourceLocator,
               std::shared_ptr<tesseract_common::ResourceLocator>>(m, "ResourceLocator")
        .def(nb::init<>())
        .def("locateResource", &tesseract_common::ResourceLocator::locateResource);

    nb::class_<tesseract_common::SimpleResourceLocator,
               tesseract_common::ResourceLocator,
               std::shared_ptr<tesseract_common::SimpleResourceLocator>>(m, "SimpleResourceLocator")
        .def(nb::init<std::function<std::string(const std::string&)>>());

    nb::class_<tesseract_common::GeneralResourceLocator,
               tesseract_common::ResourceLocator,
               std::shared_ptr<tesseract_common::GeneralResourceLocator>>(m, "GeneralResourceLocator")
        .def(nb::init<>());

    // ========== ManipulatorInfo ==========
    nb::class_<tesseract_common::ManipulatorInfo, std::shared_ptr<tesseract_common::ManipulatorInfo>>(m, "ManipulatorInfo")
        .def(nb::init<>())
        .def_rw("manipulator", &tesseract_common::ManipulatorInfo::manipulator)
        .def_rw("manipulator_ik_solver", &tesseract_common::ManipulatorInfo::manipulator_ik_solver)
        .def_rw("working_frame", &tesseract_common::ManipulatorInfo::working_frame)
        .def_rw("tcp_frame", &tesseract_common::ManipulatorInfo::tcp_frame)
        .def_property("tcp_offset",
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
    nb::class_<tesseract_common::AllowedCollisionMatrix, std::shared_ptr<tesseract_common::AllowedCollisionMatrix>>(m, "AllowedCollisionMatrix")
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
        .value("OVERRIDE_DEFAULT_MARGIN", tesseract_common::CollisionMarginOverrideType::OVERRIDE_DEFAULT_MARGIN)
        .value("OVERRIDE_PAIR_MARGIN", tesseract_common::CollisionMarginOverrideType::OVERRIDE_PAIR_MARGIN)
        .value("MODIFY_PAIR_MARGIN", tesseract_common::CollisionMarginOverrideType::MODIFY_PAIR_MARGIN);

    nb::class_<tesseract_common::CollisionMarginData>(m, "CollisionMarginData")
        .def(nb::init<>())
        .def(nb::init<double>())
        .def_rw("default_collision_margin", &tesseract_common::CollisionMarginData::default_collision_margin)
        .def_rw("max_collision_margin", &tesseract_common::CollisionMarginData::max_collision_margin)
        .def_rw("pair_collision_margins", &tesseract_common::CollisionMarginData::pair_collision_margins)
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

    nb::class_<console_bridge::OutputHandler,
               PyOutputHandler,
               std::shared_ptr<console_bridge::OutputHandler>>(m, "OutputHandler")
        .def(nb::init<>())
        .def("log", &console_bridge::OutputHandler::log);

    m.def("setLogLevel", &console_bridge::setLogLevel, "level"_a);
    m.def("getLogLevel", &console_bridge::getLogLevel);
    m.def("log", &console_bridge::log, "filename"_a, "line"_a, "level"_a, "message"_a);
    m.def("useOutputHandler", &console_bridge::useOutputHandler, "handler"_a);
    m.def("restorePreviousOutputHandler", &console_bridge::restorePreviousOutputHandler);

    // ========== STL Container Bindings ==========
    nb::bind_vector<tesseract_common::AlignedVector<Eigen::Isometry3d>>(m, "VectorIsometry3d");
    nb::bind_map<tesseract_common::TransformMap>(m, "TransformMap");
}
