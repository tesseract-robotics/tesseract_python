/**
 * @file tesseract_srdf_bindings.cpp
 * @brief nanobind bindings for tesseract_srdf
 */

#include "tesseract_nb.h"

#include <tesseract_srdf/srdf_model.h>
#include <tesseract_srdf/kinematics_information.h>
#include <tesseract_srdf/utils.h>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_common/resource_locator.h>
#include <tesseract_common/allowed_collision_matrix.h>
#include <tesseract_common/calibration_info.h>
#include <tesseract_common/collision_margin_data.h>

namespace ts = tesseract_srdf;

NB_MODULE(_tesseract_srdf, m) {
    m.doc() = "tesseract_srdf Python bindings";

    // KinematicsInformation
    nb::class_<ts::KinematicsInformation>(m, "KinematicsInformation")
        .def(nb::init<>())
        .def_rw("group_names", &ts::KinematicsInformation::group_names)
        .def_rw("chain_groups", &ts::KinematicsInformation::chain_groups)
        .def_rw("joint_groups", &ts::KinematicsInformation::joint_groups)
        .def_rw("link_groups", &ts::KinematicsInformation::link_groups)
        .def_rw("group_states", &ts::KinematicsInformation::group_states)
        .def_rw("group_tcps", &ts::KinematicsInformation::group_tcps)
        .def("insert", &ts::KinematicsInformation::insert, "other"_a)
        .def("clear", &ts::KinematicsInformation::clear)
        .def("hasGroup", &ts::KinematicsInformation::hasGroup, "group_name"_a)
        .def("addChainGroup", &ts::KinematicsInformation::addChainGroup, "group_name"_a, "chain_group"_a)
        .def("removeChainGroup", &ts::KinematicsInformation::removeChainGroup, "group_name"_a)
        .def("hasChainGroup", &ts::KinematicsInformation::hasChainGroup, "group_name"_a)
        .def("addJointGroup", &ts::KinematicsInformation::addJointGroup, "group_name"_a, "joint_group"_a)
        .def("removeJointGroup", &ts::KinematicsInformation::removeJointGroup, "group_name"_a)
        .def("hasJointGroup", &ts::KinematicsInformation::hasJointGroup, "group_name"_a)
        .def("addLinkGroup", &ts::KinematicsInformation::addLinkGroup, "group_name"_a, "link_group"_a)
        .def("removeLinkGroup", &ts::KinematicsInformation::removeLinkGroup, "group_name"_a)
        .def("hasLinkGroup", &ts::KinematicsInformation::hasLinkGroup, "group_name"_a)
        .def("addGroupJointState", &ts::KinematicsInformation::addGroupJointState,
             "group_name"_a, "state_name"_a, "joint_state"_a)
        .def("removeGroupJointState", &ts::KinematicsInformation::removeGroupJointState,
             "group_name"_a, "state_name"_a)
        .def("hasGroupJointState", &ts::KinematicsInformation::hasGroupJointState,
             "group_name"_a, "state_name"_a)
        .def("addGroupTCP", &ts::KinematicsInformation::addGroupTCP,
             "group_name"_a, "tcp_name"_a, "tcp"_a)
        .def("removeGroupTCP", &ts::KinematicsInformation::removeGroupTCP,
             "group_name"_a, "tcp_name"_a)
        .def("hasGroupTCP", &ts::KinematicsInformation::hasGroupTCP,
             "group_name"_a, "tcp_name"_a)
        .def("__eq__", &ts::KinematicsInformation::operator==)
        .def("__ne__", &ts::KinematicsInformation::operator!=);

    // SRDFModel
    nb::class_<ts::SRDFModel>(m, "SRDFModel")
        .def(nb::init<>())
        .def("initFile", &ts::SRDFModel::initFile,
             "scene_graph"_a, "filename"_a, "locator"_a)
        .def("initString", &ts::SRDFModel::initString,
             "scene_graph"_a, "xmlstring"_a, "locator"_a)
        .def("saveToFile", &ts::SRDFModel::saveToFile, "file_path"_a)
        .def("clear", &ts::SRDFModel::clear)
        .def_rw("name", &ts::SRDFModel::name)
        .def_rw("version", &ts::SRDFModel::version)
        .def_rw("kinematics_information", &ts::SRDFModel::kinematics_information)
        .def_rw("acm", &ts::SRDFModel::acm)
        .def_rw("collision_margin_data", &ts::SRDFModel::collision_margin_data)
        .def_rw("calibration_info", &ts::SRDFModel::calibration_info)
        .def("__eq__", &ts::SRDFModel::operator==)
        .def("__ne__", &ts::SRDFModel::operator!=)
        .def("__repr__", [](const ts::SRDFModel& self) {
            return "SRDFModel('" + self.name + "')";
        });

    // Utility functions
    m.def("processSRDFAllowedCollisions", &ts::processSRDFAllowedCollisions,
          "scene_graph"_a, "srdf_model"_a,
          "Process SRDF allowed collisions and add to scene graph ACM");
}
