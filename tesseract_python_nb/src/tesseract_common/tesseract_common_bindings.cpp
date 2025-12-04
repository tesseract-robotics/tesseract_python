#include "tesseract_nb.h"

// tesseract_common headers (need eigen_types.h before opaque declarations)
#include <tesseract_common/eigen_types.h>
#include <tesseract_common/types.h>

// Opaque declarations for vector types we want to bind as classes
using VectorVector3d = tesseract_common::VectorVector3d;  // std::vector<Eigen::Vector3d>
using VectorIsometry3d = tesseract_common::VectorIsometry3d;  // std::vector<Eigen::Isometry3d>
NB_MAKE_OPAQUE(VectorVector3d)
NB_MAKE_OPAQUE(VectorIsometry3d)
#include <tesseract_common/resource_locator.h>
#include <tesseract_common/manipulator_info.h>
#include <tesseract_common/joint_state.h>
#include <tesseract_common/collision_margin_data.h>
#include <tesseract_common/allowed_collision_matrix.h>
#include <tesseract_common/kinematic_limits.h>
#include <tesseract_common/plugin_info.h>
#include <tesseract_common/filesystem.h>

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
    // Isometry3d needs explicit binding for SWIG compatibility (tests expect .matrix() method)

    // Isometry3d class binding for SWIG API compatibility
    nb::class_<Eigen::Isometry3d>(m, "Isometry3d")
        .def(nb::init<>())  // Identity transform
        .def("__init__", [](Eigen::Isometry3d* self, const Eigen::Matrix4d& mat) {
            new (self) Eigen::Isometry3d();
            self->matrix() = mat;
        }, "matrix"_a)
        .def_static("Identity", []() { return Eigen::Isometry3d::Identity(); })
        .def("matrix", [](const Eigen::Isometry3d& self) -> Eigen::Matrix4d {
            return self.matrix();
        })
        .def("translation", [](const Eigen::Isometry3d& self) -> Eigen::Vector3d {
            return self.translation();
        })
        .def("rotation", [](const Eigen::Isometry3d& self) -> Eigen::Matrix3d {
            return self.rotation();
        })
        .def("linear", [](const Eigen::Isometry3d& self) -> Eigen::Matrix3d {
            return self.linear();
        })
        .def("inverse", [](const Eigen::Isometry3d& self) {
            return self.inverse();
        })
        .def("__mul__", [](const Eigen::Isometry3d& self, const Eigen::Isometry3d& other) {
            return self * other;
        })
        .def("__mul__", [](const Eigen::Isometry3d& self, const Eigen::Translation3d& t) {
            return Eigen::Isometry3d(self * t);
        })
        .def("__mul__", [](const Eigen::Isometry3d& self, const Eigen::Quaterniond& q) {
            Eigen::Isometry3d result = self;
            result.rotate(q);
            return result;
        })
        .def("__mul__", [](const Eigen::Isometry3d& self, const Eigen::AngleAxisd& aa) {
            Eigen::Isometry3d result = self;
            result.rotate(aa);
            return result;
        })
        .def("__mul__", [](const Eigen::Isometry3d& self, const Eigen::Vector3d& v) {
            return self * v;
        });

    nb::class_<Eigen::Translation3d>(m, "Translation3d")
        .def(nb::init<double, double, double>())
        .def("__mul__", [](const Eigen::Translation3d& self, const Eigen::Isometry3d& other) {
            return Eigen::Isometry3d(self * other);
        })
        .def("__mul__", [](const Eigen::Translation3d& self, const Eigen::Translation3d& other) {
            return Eigen::Translation3d(self.x() + other.x(), self.y() + other.y(), self.z() + other.z());
        });

    nb::class_<Eigen::Quaterniond>(m, "Quaterniond")
        .def(nb::init<double, double, double, double>())  // w, x, y, z
        // Constructor from rotation matrix
        .def("__init__", [](Eigen::Quaterniond* self, const Eigen::Matrix3d& rot) {
            new (self) Eigen::Quaterniond(rot);
        }, "rotation_matrix"_a)
        .def("w", [](const Eigen::Quaterniond& q) { return q.w(); })
        .def("x", [](const Eigen::Quaterniond& q) { return q.x(); })
        .def("y", [](const Eigen::Quaterniond& q) { return q.y(); })
        .def("z", [](const Eigen::Quaterniond& q) { return q.z(); })
        .def("toRotationMatrix", [](const Eigen::Quaterniond& q) -> Eigen::Matrix3d {
            return q.toRotationMatrix();
        });

    nb::class_<Eigen::AngleAxisd>(m, "AngleAxisd")
        .def(nb::init<double, const Eigen::Vector3d&>())
        .def("toRotationMatrix", [](const Eigen::AngleAxisd& self) -> Eigen::Matrix3d {
            return self.toRotationMatrix();
        });

    // ========== FilesystemPath ==========
    // Wrapper for std::filesystem::path (SWIG compatibility)
    nb::class_<tesseract_common::fs::path>(m, "FilesystemPath")
        .def(nb::init<>())
        .def(nb::init<const std::string&>(), "path"_a)
        .def("string", [](const tesseract_common::fs::path& p) { return p.string(); })
        .def("__str__", [](const tesseract_common::fs::path& p) { return p.string(); })
        .def("__repr__", [](const tesseract_common::fs::path& p) {
            return "FilesystemPath('" + p.string() + "')";
        });

    // Note: TransformMap (std::map<string, Isometry3d>) is handled automatically by nanobind's
    // stl/map type caster - Python dict with Isometry3d values will convert automatically

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

    nb::class_<tesseract_common::SimpleLocatedResource, tesseract_common::Resource>(m, "SimpleLocatedResource")
        .def(nb::init<const std::string&, const std::string&>(), "url"_a, "filename"_a)
        .def(nb::init<const std::string&, const std::string&, std::shared_ptr<tesseract_common::ResourceLocator>>(),
             "url"_a, "filename"_a, "parent"_a);

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
    // Wrapper for console_bridge::log (variadic function)
    m.def("log", [](const std::string& filename, int line, console_bridge::LogLevel level, const std::string& msg) {
        console_bridge::log(filename.c_str(), line, level, "%s", msg.c_str());
    }, "filename"_a, "line"_a, "level"_a, "msg"_a);
    m.def("useOutputHandler", &console_bridge::useOutputHandler, "handler"_a);
    m.def("restorePreviousOutputHandler", &console_bridge::restorePreviousOutputHandler);

    // ========== STL Container Bindings ==========
    // VectorVector3d - explicit binding for aligned Eigen vectors (NB_MAKE_OPAQUE at top)
    nb::class_<VectorVector3d>(m, "VectorVector3d")
        .def(nb::init<>())
        .def("__len__", [](const VectorVector3d& self) { return self.size(); })
        .def("__getitem__", [](const VectorVector3d& self, size_t i) -> Eigen::Vector3d {
            if (i >= self.size()) throw std::out_of_range("index out of range");
            return self[i];
        })
        .def("__setitem__", [](VectorVector3d& self, size_t i, const Eigen::Vector3d& v) {
            if (i >= self.size()) throw std::out_of_range("index out of range");
            self[i] = v;
        })
        .def("append", [](VectorVector3d& self, const Eigen::Vector3d& v) { self.push_back(v); })
        .def("clear", [](VectorVector3d& self) { self.clear(); });

    // VectorIsometry3d - for transform arrays (NB_MAKE_OPAQUE at top)
    nb::class_<VectorIsometry3d>(m, "VectorIsometry3d")
        .def(nb::init<>())
        .def("__len__", [](const VectorIsometry3d& self) { return self.size(); })
        .def("__getitem__", [](const VectorIsometry3d& self, size_t i) {
            if (i >= self.size()) throw std::out_of_range("index out of range");
            return self[i];
        })
        .def("append", [](VectorIsometry3d& self, const Eigen::Isometry3d& v) { self.push_back(v); })
        .def("clear", [](VectorIsometry3d& self) { self.clear(); });

    // Note: VectorLong and Eigen::VectorXi use numpy arrays (automatic conversion)
}
