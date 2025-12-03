/**
 * @file tesseract_geometry_bindings.cpp
 * @brief nanobind bindings for tesseract_geometry
 */

#include "tesseract_nb.h"

// tesseract_geometry
#include <tesseract_geometry/geometry.h>
#include <tesseract_geometry/geometries.h>
#include <tesseract_geometry/impl/box.h>
#include <tesseract_geometry/impl/sphere.h>
#include <tesseract_geometry/impl/cylinder.h>
#include <tesseract_geometry/impl/capsule.h>
#include <tesseract_geometry/impl/cone.h>
#include <tesseract_geometry/impl/plane.h>
#include <tesseract_geometry/impl/polygon_mesh.h>

namespace tg = tesseract_geometry;

NB_MODULE(_tesseract_geometry, m) {
    m.doc() = "tesseract_geometry Python bindings";

    // GeometryType enum
    nb::enum_<tg::GeometryType>(m, "GeometryType")
        .value("UNINITIALIZED", tg::GeometryType::UNINITIALIZED)
        .value("SPHERE", tg::GeometryType::SPHERE)
        .value("CYLINDER", tg::GeometryType::CYLINDER)
        .value("CAPSULE", tg::GeometryType::CAPSULE)
        .value("CONE", tg::GeometryType::CONE)
        .value("BOX", tg::GeometryType::BOX)
        .value("PLANE", tg::GeometryType::PLANE)
        .value("MESH", tg::GeometryType::MESH)
        .value("CONVEX_MESH", tg::GeometryType::CONVEX_MESH)
        .value("SDF_MESH", tg::GeometryType::SDF_MESH)
        .value("OCTREE", tg::GeometryType::OCTREE)
        .value("POLYGON_MESH", tg::GeometryType::POLYGON_MESH)
        .value("COMPOUND_MESH", tg::GeometryType::COMPOUND_MESH);

    // Geometry base class (abstract)
    nb::class_<tg::Geometry>(m, "Geometry")
        .def("getType", &tg::Geometry::getType, "Get the geometry type")
        .def("clone", &tg::Geometry::clone, "Create a copy of this geometry")
        .def("__eq__", &tg::Geometry::operator==)
        .def("__ne__", &tg::Geometry::operator!=);

    // Box
    nb::class_<tg::Box, tg::Geometry>(m, "Box")
        .def(nb::init<double, double, double>(), "x"_a, "y"_a, "z"_a,
             "Create a box with dimensions x, y, z")
        .def(nb::init<>(), "Create a default box")
        .def("getX", &tg::Box::getX, "Get X dimension")
        .def("getY", &tg::Box::getY, "Get Y dimension")
        .def("getZ", &tg::Box::getZ, "Get Z dimension")
        .def("__eq__", &tg::Box::operator==)
        .def("__ne__", &tg::Box::operator!=)
        .def("__repr__", [](const tg::Box& self) {
            return "Box(" + std::to_string(self.getX()) + ", " +
                   std::to_string(self.getY()) + ", " +
                   std::to_string(self.getZ()) + ")";
        });

    // Sphere
    nb::class_<tg::Sphere, tg::Geometry>(m, "Sphere")
        .def(nb::init<double>(), "r"_a, "Create a sphere with radius r")
        .def(nb::init<>(), "Create a default sphere")
        .def("getRadius", &tg::Sphere::getRadius, "Get the radius")
        .def("__eq__", &tg::Sphere::operator==)
        .def("__ne__", &tg::Sphere::operator!=)
        .def("__repr__", [](const tg::Sphere& self) {
            return "Sphere(" + std::to_string(self.getRadius()) + ")";
        });

    // Cylinder
    nb::class_<tg::Cylinder, tg::Geometry>(m, "Cylinder")
        .def(nb::init<double, double>(), "r"_a, "l"_a,
             "Create a cylinder with radius r and length l")
        .def(nb::init<>(), "Create a default cylinder")
        .def("getRadius", &tg::Cylinder::getRadius, "Get the radius")
        .def("getLength", &tg::Cylinder::getLength, "Get the length")
        .def("__eq__", &tg::Cylinder::operator==)
        .def("__ne__", &tg::Cylinder::operator!=)
        .def("__repr__", [](const tg::Cylinder& self) {
            return "Cylinder(r=" + std::to_string(self.getRadius()) +
                   ", l=" + std::to_string(self.getLength()) + ")";
        });

    // Capsule
    nb::class_<tg::Capsule, tg::Geometry>(m, "Capsule")
        .def(nb::init<double, double>(), "r"_a, "l"_a,
             "Create a capsule with radius r and length l")
        .def(nb::init<>(), "Create a default capsule")
        .def("getRadius", &tg::Capsule::getRadius, "Get the radius")
        .def("getLength", &tg::Capsule::getLength, "Get the length")
        .def("__eq__", &tg::Capsule::operator==)
        .def("__ne__", &tg::Capsule::operator!=)
        .def("__repr__", [](const tg::Capsule& self) {
            return "Capsule(r=" + std::to_string(self.getRadius()) +
                   ", l=" + std::to_string(self.getLength()) + ")";
        });

    // Cone
    nb::class_<tg::Cone, tg::Geometry>(m, "Cone")
        .def(nb::init<double, double>(), "r"_a, "l"_a,
             "Create a cone with radius r and length l")
        .def(nb::init<>(), "Create a default cone")
        .def("getRadius", &tg::Cone::getRadius, "Get the radius")
        .def("getLength", &tg::Cone::getLength, "Get the length")
        .def("__eq__", &tg::Cone::operator==)
        .def("__ne__", &tg::Cone::operator!=)
        .def("__repr__", [](const tg::Cone& self) {
            return "Cone(r=" + std::to_string(self.getRadius()) +
                   ", l=" + std::to_string(self.getLength()) + ")";
        });

    // Plane
    nb::class_<tg::Plane, tg::Geometry>(m, "Plane")
        .def(nb::init<double, double, double, double>(), "a"_a, "b"_a, "c"_a, "d"_a,
             "Create a plane with equation ax + by + cz + d = 0")
        .def(nb::init<>(), "Create a default plane")
        .def("getA", &tg::Plane::getA, "Get coefficient a")
        .def("getB", &tg::Plane::getB, "Get coefficient b")
        .def("getC", &tg::Plane::getC, "Get coefficient c")
        .def("getD", &tg::Plane::getD, "Get coefficient d")
        .def("__eq__", &tg::Plane::operator==)
        .def("__ne__", &tg::Plane::operator!=)
        .def("__repr__", [](const tg::Plane& self) {
            return "Plane(" + std::to_string(self.getA()) + ", " +
                   std::to_string(self.getB()) + ", " +
                   std::to_string(self.getC()) + ", " +
                   std::to_string(self.getD()) + ")";
        });

    // PolygonMesh (base for Mesh, ConvexMesh, SDFMesh)
    // Note: Mesh types with vertices/faces require special handling due to
    // shared_ptr<vector<Eigen>> type conversion issues. Keeping simple for now.
    nb::class_<tg::PolygonMesh, tg::Geometry>(m, "PolygonMesh")
        .def("getVertexCount", &tg::PolygonMesh::getVertexCount, "Get number of vertices")
        .def("getFaceCount", &tg::PolygonMesh::getFaceCount, "Get number of faces")
        .def("getScale", &tg::PolygonMesh::getScale, "Get mesh scale");
}
