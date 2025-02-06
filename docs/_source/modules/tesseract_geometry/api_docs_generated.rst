
Classes
*******


Box Class
===================================================================================================
.. autoclass:: tesseract_robotics.tesseract_geometry.Box
   :noindex:
   :exclude-members: thisown
   :members:

Capsule Class
===================================================================================================
.. autoclass:: tesseract_robotics.tesseract_geometry.Capsule
   :noindex:
   :exclude-members: thisown
   :members:

CompoundMesh Class
===================================================================================================
.. autoclass:: tesseract_robotics.tesseract_geometry.CompoundMesh
   :noindex:
   :exclude-members: thisown
   :members:

Cone Class
===================================================================================================
.. autoclass:: tesseract_robotics.tesseract_geometry.Cone
   :noindex:
   :exclude-members: thisown
   :members:

ConvexMesh Class
===================================================================================================
.. autoclass:: tesseract_robotics.tesseract_geometry.ConvexMesh
   :noindex:
   :exclude-members: thisown
   :members:

Cylinder Class
===================================================================================================
.. autoclass:: tesseract_robotics.tesseract_geometry.Cylinder
   :noindex:
   :exclude-members: thisown
   :members:

Geometry Class
===================================================================================================
.. autoclass:: tesseract_robotics.tesseract_geometry.Geometry
   :noindex:
   :exclude-members: thisown
   :members:

Mesh Class
===================================================================================================
.. autoclass:: tesseract_robotics.tesseract_geometry.Mesh
   :noindex:
   :exclude-members: thisown
   :members:

MeshMaterial Class
===================================================================================================
.. autoclass:: tesseract_robotics.tesseract_geometry.MeshMaterial
   :noindex:
   :exclude-members: thisown
   :members:

MeshTexture Class
===================================================================================================
.. autoclass:: tesseract_robotics.tesseract_geometry.MeshTexture
   :noindex:
   :exclude-members: thisown
   :members:

OcTree Class
===================================================================================================
.. autoclass:: tesseract_robotics.tesseract_geometry.OcTree
   :noindex:
   :exclude-members: thisown
   :members:

Octree Class
===================================================================================================
.. autoclass:: tesseract_robotics.tesseract_geometry.Octree
   :noindex:
   :exclude-members: thisown
   :members:

Plane Class
===================================================================================================
.. autoclass:: tesseract_robotics.tesseract_geometry.Plane
   :noindex:
   :exclude-members: thisown
   :members:

PolygonMesh Class
===================================================================================================
.. autoclass:: tesseract_robotics.tesseract_geometry.PolygonMesh
   :noindex:
   :exclude-members: thisown
   :members:

SDFMesh Class
===================================================================================================
.. autoclass:: tesseract_robotics.tesseract_geometry.SDFMesh
   :noindex:
   :exclude-members: thisown
   :members:

Sphere Class
===================================================================================================
.. autoclass:: tesseract_robotics.tesseract_geometry.Sphere
   :noindex:
   :exclude-members: thisown
   :members:


Functions
*********

createConvexMeshFromBytes Function
===================================================================================================
.. autofunction:: tesseract_robotics.tesseract_geometry.createConvexMeshFromBytes
   :noindex:

createConvexMeshFromPath Function
===================================================================================================
.. autofunction:: tesseract_robotics.tesseract_geometry.createConvexMeshFromPath
   :noindex:

createConvexMeshFromResource Function
===================================================================================================
.. autofunction:: tesseract_robotics.tesseract_geometry.createConvexMeshFromResource
   :noindex:

createMeshFromBytes Function
===================================================================================================
.. autofunction:: tesseract_robotics.tesseract_geometry.createMeshFromBytes
   :noindex:

createMeshFromPath Function
===================================================================================================
.. autofunction:: tesseract_robotics.tesseract_geometry.createMeshFromPath
   :noindex:

createMeshFromResource Function
===================================================================================================
.. autofunction:: tesseract_robotics.tesseract_geometry.createMeshFromResource
   :noindex:

createSDFMeshFromBytes Function
===================================================================================================
.. autofunction:: tesseract_robotics.tesseract_geometry.createSDFMeshFromBytes
   :noindex:

createSDFMeshFromPath Function
===================================================================================================
.. autofunction:: tesseract_robotics.tesseract_geometry.createSDFMeshFromPath
   :noindex:

createSDFMeshFromResource Function
===================================================================================================
.. autofunction:: tesseract_robotics.tesseract_geometry.createSDFMeshFromResource
   :noindex:

isIdentical Function
===================================================================================================
.. autofunction:: tesseract_robotics.tesseract_geometry.isIdentical
   :noindex:

Constants
*********

* ``GeometryType_BOX``
* ``GeometryType_CAPSULE``
* ``GeometryType_COMPOUND_MESH``
* ``GeometryType_CONE``
* ``GeometryType_CONVEX_MESH``
* ``GeometryType_CYLINDER``
* ``GeometryType_MESH``
* ``GeometryType_OCTREE``
* ``GeometryType_PLANE``
* ``GeometryType_POLYGON_MESH``
* ``GeometryType_SDF_MESH``
* ``GeometryType_SPHERE``
* ``GeometryType_UNINITIALIZED``
* ``OctreeSubType_BOX``
* ``OctreeSubType_SPHERE_INSIDE``
* ``OctreeSubType_SPHERE_OUTSIDE``

Container Templates
*******************

* ``ConvexMeshVector`` -> ``std::vector<std::shared_ptr<tesseract_geometry::ConvexMesh> >``
* ``Geometries`` -> ``std::vector<std::shared_ptr<tesseract_geometry::Geometry> >``
* ``GeometriesConst`` -> ``std::vector<std::shared_ptr<const tesseract_geometry::Geometry> >``
* ``MeshVector`` -> ``std::vector<std::shared_ptr<tesseract_geometry::Mesh> >``
* ``PolygonMeshVector`` -> ``std::vector<std::shared_ptr<tesseract_geometry::PolygonMesh> >``
* ``SDFMeshVector`` -> ``std::vector<std::shared_ptr<tesseract_geometry::SDFMesh> >``
* ``VectorMeshTexture`` -> ``std::vector<std::shared_ptr<tesseract_geometry::MeshTexture>>``

