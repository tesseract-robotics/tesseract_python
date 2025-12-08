/**
 * @file tesseract_urdf_bindings.cpp
 * @brief nanobind bindings for tesseract_urdf
 */

#include "tesseract_nb.h"

#include <tesseract_urdf/urdf_parser.h>
#include <tesseract_scene_graph/graph.h>
#include <tesseract_common/resource_locator.h>

namespace tu = tesseract_urdf;

NB_MODULE(_tesseract_urdf, m) {
    m.doc() = "tesseract_urdf Python bindings";

    // parseURDFString - returns unique_ptr, nanobind handles conversion
    m.def("parseURDFString", &tu::parseURDFString,
          "urdf_xml_string"_a, "locator"_a,
          "Parse a URDF string into a SceneGraph");

    // parseURDFFile
    m.def("parseURDFFile", &tu::parseURDFFile,
          "path"_a, "locator"_a,
          "Parse a URDF file into a SceneGraph");

    // writeURDFFile
    m.def("writeURDFFile", &tu::writeURDFFile,
          "scene_graph"_a, "package_path"_a, "urdf_name"_a = "",
          "Write a SceneGraph to a URDF file");
}
