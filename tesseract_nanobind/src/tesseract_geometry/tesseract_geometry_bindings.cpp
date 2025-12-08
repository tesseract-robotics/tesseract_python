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
#include <tesseract_geometry/impl/mesh.h>
#include <tesseract_geometry/impl/convex_mesh.h>
#include <tesseract_geometry/impl/sdf_mesh.h>
#include <tesseract_geometry/impl/compound_mesh.h>
#include <tesseract_geometry/impl/mesh_material.h>
#include <tesseract_geometry/mesh_parser.h>

// tesseract_common
#include <tesseract_common/types.h>
#include <tesseract_common/eigen_types.h>
#include <tesseract_common/resource_locator.h>

namespace tg = tesseract_geometry;
namespace tc = tesseract_common;

// Disable type caster for this specific vector type so we can bind it as a class
NB_MAKE_OPAQUE(std::vector<std::shared_ptr<const tg::Geometry>>);

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

    // GeometriesConst - vector of const geometry shared_ptr
    using GeometriesConst = std::vector<std::shared_ptr<const tg::Geometry>>;
    nb::class_<GeometriesConst>(m, "GeometriesConst")
        .def(nb::init<>())
        .def("__len__", [](const GeometriesConst& v) { return v.size(); })
        .def("__getitem__", [](const GeometriesConst& v, size_t i) {
            if (i >= v.size()) throw nb::index_error();
            return v[i];
        }, nb::rv_policy::reference_internal)
        .def("append", [](GeometriesConst& v, std::shared_ptr<const tg::Geometry> item) {
            v.push_back(item);
        })
        .def("clear", [](GeometriesConst& v) { v.clear(); });

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

    // MeshMaterial - PBR material properties
    nb::class_<tg::MeshMaterial>(m, "MeshMaterial")
        .def(nb::init<>())
        .def(nb::init<const Eigen::Vector4d&, double, double, const Eigen::Vector4d&>(),
             "base_color_factor"_a, "metallic_factor"_a, "roughness_factor"_a, "emissive_factor"_a)
        .def("getBaseColorFactor", &tg::MeshMaterial::getBaseColorFactor, "Get base color (RGBA)")
        .def("getMetallicFactor", &tg::MeshMaterial::getMetallicFactor, "Get metallic factor (0-1)")
        .def("getRoughnessFactor", &tg::MeshMaterial::getRoughnessFactor, "Get roughness factor (0-1)")
        .def("getEmissiveFactor", &tg::MeshMaterial::getEmissiveFactor, "Get emissive factor (RGBA)");

    // MeshTexture - texture with UV coordinates
    nb::class_<tg::MeshTexture>(m, "MeshTexture")
        .def("getTextureImage", &tg::MeshTexture::getTextureImage, "Get the texture image resource")
        .def("getUVs", [](tg::MeshTexture& self) {
            auto uvs = self.getUVs();
            if (!uvs) return tc::VectorVector2d();
            return *uvs;
        }, "Get UV coordinates");

    // PolygonMesh (base for Mesh, ConvexMesh, SDFMesh) - inherits shared_ptr holder from Geometry
    nb::class_<tg::PolygonMesh, tg::Geometry>(m, "PolygonMesh")
        .def("getVertexCount", &tg::PolygonMesh::getVertexCount, "Get number of vertices")
        .def("getFaceCount", &tg::PolygonMesh::getFaceCount, "Get number of faces")
        .def("getScale", &tg::PolygonMesh::getScale, "Get mesh scale")
        .def("getVertices", [](const tg::PolygonMesh& self) {
            auto verts = self.getVertices();
            if (!verts) return tc::VectorVector3d();
            return *verts;
        })
        .def("getFaces", [](const tg::PolygonMesh& self) -> Eigen::VectorXi {
            auto faces = self.getFaces();
            if (!faces) return Eigen::VectorXi();
            return *faces;
        })
        .def("getNormals", [](const tg::PolygonMesh& self) -> std::optional<tc::VectorVector3d> {
            auto normals = self.getNormals();
            if (!normals) return std::nullopt;
            return *normals;
        }, "Get vertex normals (optional)")
        .def("getVertexColors", [](const tg::PolygonMesh& self) -> std::optional<tc::VectorVector4d> {
            auto colors = self.getVertexColors();
            if (!colors) return std::nullopt;
            return *colors;
        }, "Get vertex colors (optional)")
        .def("getMaterial", &tg::PolygonMesh::getMaterial, "Get mesh material (optional)")
        .def("getTextures", [](const tg::PolygonMesh& self) -> std::optional<std::vector<std::shared_ptr<tg::MeshTexture>>> {
            auto textures = self.getTextures();
            if (!textures) return std::nullopt;
            return *textures;
        }, "Get mesh textures (optional)")
        .def("getResource", &tg::PolygonMesh::getResource, "Get mesh resource");

    // Mesh
    nb::class_<tg::Mesh, tg::PolygonMesh>(m, "Mesh")
        .def("__init__", [](tg::Mesh* self, const tc::VectorVector3d& vertices, const Eigen::VectorXi& faces) {
            auto verts = std::make_shared<const tc::VectorVector3d>(vertices);
            auto face_data = std::make_shared<const Eigen::VectorXi>(faces);
            new (self) tg::Mesh(verts, face_data);
        }, "vertices"_a, "faces"_a);

    // ConvexMesh
    nb::class_<tg::ConvexMesh, tg::PolygonMesh>(m, "ConvexMesh")
        .def("__init__", [](tg::ConvexMesh* self, const tc::VectorVector3d& vertices, const Eigen::VectorXi& faces) {
            auto verts = std::make_shared<const tc::VectorVector3d>(vertices);
            auto face_data = std::make_shared<const Eigen::VectorXi>(faces);
            new (self) tg::ConvexMesh(verts, face_data);
        }, "vertices"_a, "faces"_a);

    // SDFMesh
    nb::class_<tg::SDFMesh, tg::PolygonMesh>(m, "SDFMesh")
        .def("__init__", [](tg::SDFMesh* self, const tc::VectorVector3d& vertices, const Eigen::VectorXi& faces) {
            auto verts = std::make_shared<const tc::VectorVector3d>(vertices);
            auto face_data = std::make_shared<const Eigen::VectorXi>(faces);
            new (self) tg::SDFMesh(verts, face_data);
        }, "vertices"_a, "faces"_a);

    // CompoundMesh - container for multiple meshes from a single resource (e.g., .dae file)
    nb::class_<tg::CompoundMesh, tg::Geometry>(m, "CompoundMesh")
        .def(nb::init<std::vector<std::shared_ptr<tg::PolygonMesh>>>(), "meshes"_a)
        .def("getMeshes", &tg::CompoundMesh::getMeshes, nb::rv_policy::reference_internal,
             "Get the vector of meshes")
        .def("getResource", &tg::CompoundMesh::getResource, "Get the resource used to create this mesh")
        .def("getScale", &tg::CompoundMesh::getScale, "Get the scale applied to the mesh");

    // Mesh loading functions
    m.def("createMeshFromPath", [](const std::string& path,
                                   const Eigen::Vector3d& scale,
                                   bool triangulate,
                                   bool flatten) {
        return tg::createMeshFromPath<tg::Mesh>(path, scale, triangulate, flatten);
    }, "path"_a, "scale"_a = Eigen::Vector3d::Ones(), "triangulate"_a = true, "flatten"_a = false,
    "Load mesh from file and return vector of Mesh geometries");

    m.def("createConvexMeshFromPath", [](const std::string& path,
                                         const Eigen::Vector3d& scale,
                                         bool triangulate,
                                         bool flatten) {
        return tg::createMeshFromPath<tg::ConvexMesh>(path, scale, triangulate, flatten);
    }, "path"_a, "scale"_a = Eigen::Vector3d::Ones(), "triangulate"_a = true, "flatten"_a = false,
    "Load mesh from file and return vector of ConvexMesh geometries");

    m.def("createSDFMeshFromPath", [](const std::string& path,
                                      const Eigen::Vector3d& scale,
                                      bool triangulate,
                                      bool flatten) {
        return tg::createMeshFromPath<tg::SDFMesh>(path, scale, triangulate, flatten);
    }, "path"_a, "scale"_a = Eigen::Vector3d::Ones(), "triangulate"_a = true, "flatten"_a = false,
    "Load mesh from file and return vector of SDFMesh geometries");

    // Mesh loading from Resource (for package:// URLs)
    m.def("createMeshFromResource", [](tc::Resource::Ptr resource,
                                       const Eigen::Vector3d& scale,
                                       bool triangulate,
                                       bool flatten) {
        return tg::createMeshFromResource<tg::Mesh>(resource, scale, triangulate, flatten);
    }, "resource"_a, "scale"_a = Eigen::Vector3d::Ones(), "triangulate"_a = true, "flatten"_a = false,
    "Load Mesh from resource (e.g., package:// URL)");

    m.def("createConvexMeshFromResource", [](tc::Resource::Ptr resource,
                                              const Eigen::Vector3d& scale,
                                              bool triangulate,
                                              bool flatten) {
        return tg::createMeshFromResource<tg::ConvexMesh>(resource, scale, triangulate, flatten);
    }, "resource"_a, "scale"_a = Eigen::Vector3d::Ones(), "triangulate"_a = true, "flatten"_a = false,
    "Load ConvexMesh from resource (e.g., package:// URL)");

    m.def("createSDFMeshFromResource", [](tc::Resource::Ptr resource,
                                          const Eigen::Vector3d& scale,
                                          bool triangulate,
                                          bool flatten) {
        return tg::createMeshFromResource<tg::SDFMesh>(resource, scale, triangulate, flatten);
    }, "resource"_a, "scale"_a = Eigen::Vector3d::Ones(), "triangulate"_a = true, "flatten"_a = false,
    "Load SDFMesh from resource (e.g., package:// URL)");
}
