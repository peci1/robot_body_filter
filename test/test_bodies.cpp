#include "gtest/gtest.h"
#include <set>
#include <robot_body_filter/utils/bodies.h>
#include <geometric_shapes/mesh_operations.h>
#include "utils.cpp"

using namespace bodies;

#define EXPECT_VECTORS_EQUAL(v1, v2, error)                                                                            \
  EXPECT_NEAR((v1)[0], (v2)[0], (error));                                                                              \
  EXPECT_NEAR((v1)[1], (v2)[1], (error));                                                                              \
  EXPECT_NEAR((v1)[2], (v2)[2], (error));

class WrongBody : public ::bodies::Body
{
  public: WrongBody(const ::shapes::ShapeType type) : Body()
  {
    this->type_ = type;
  }

  std::vector<double> getDimensions() const override
  {
    throw std::runtime_error("Should not be called");
  }
  bool containsPoint(const Eigen::Vector3d& p, bool verbose) const override
  {
    throw std::runtime_error("Should not be called");
  }
  bool intersectsRay(const Eigen::Vector3d& origin, const Eigen::Vector3d& dir,
    EigenSTL::vector_Vector3d* intersections, unsigned int count) const override
  {
    throw std::runtime_error("Should not be called");
  }
  double computeVolume() const override
  {
    throw std::runtime_error("Should not be called");
  }
  void computeBoundingSphere(BoundingSphere& sphere) const override
  {
    throw std::runtime_error("Should not be called");
  }
  void computeBoundingCylinder(BoundingCylinder& cylinder) const override
  {
    throw std::runtime_error("Should not be called");
  }
  void computeBoundingBox(AABB& box) const override
  {
    throw std::runtime_error("Should not be called");
  }
  BodyPtr cloneAt(const Eigen::Isometry3d& pose, double padding, double scaling) const override
  {
    throw std::runtime_error("Should not be called");
  }
  // noetic has this function as pure virtual, melodic should not have it at all,
  // so defining it here like this (using virtual and not override) should work
  // in both cases
  virtual std::vector<double> getScaledDimensions() const
  {
    throw std::runtime_error("Should not be called");
  }

protected:
  void updateInternalData() override
  {
    throw std::runtime_error("Should not be called");
  }
  void useDimensions(const shapes::Shape* shape) override
  {
    throw std::runtime_error("Should not be called");
  }
};

TEST(Bodies, ComputeBoundingBoxWrongBody)
{
  AxisAlignedBoundingBox bbox;
  OrientedBoundingBox obb;

  const auto unknown = WrongBody(::shapes::ShapeType::UNKNOWN_SHAPE);
  EXPECT_THROW(unknown.computeBoundingBox(bbox), std::runtime_error);
  EXPECT_THROW(computeBoundingBox(&unknown, obb), std::runtime_error);

  const auto cone = WrongBody(::shapes::ShapeType::CONE);
  EXPECT_THROW(cone.computeBoundingBox(bbox), std::runtime_error);
  EXPECT_THROW(computeBoundingBox(&cone, obb), std::runtime_error);

  const auto octree = WrongBody(::shapes::ShapeType::OCTREE);
  EXPECT_THROW(octree.computeBoundingBox(bbox), std::runtime_error);
  EXPECT_THROW(computeBoundingBox(&octree, obb), std::runtime_error);

  const auto plane = WrongBody(::shapes::ShapeType::PLANE);
  EXPECT_THROW(plane.computeBoundingBox(bbox), std::runtime_error);
  EXPECT_THROW(computeBoundingBox(&plane, obb), std::runtime_error);
}

TEST(Bodies, ComputeBoundingBoxSphere)
{
  AxisAlignedBoundingBox bbox1, bbox2, bbox3;
  OrientedBoundingBox obb1, obb2;

  const Eigen::Isometry3d pose = Eigen::Translation3d(1.0, 2.0, 3.0) *
      Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d(0.0, 1.0, 0.0));

  const shapes::Shape* shape = new shapes::Sphere(2.0);
  const auto sphere = new Sphere(shape);
  const auto body = dynamic_cast<Body*>(sphere);

  computeBoundingBoxAt(sphere, bbox3, pose);

  body->setPose(pose);

  sphere->computeBoundingBox(bbox1);
  body->computeBoundingBox(bbox2);
  computeBoundingBox(sphere, obb1);
  computeBoundingBox(body, obb2);

  // check that bboxes are equal regardless we ask through the exact type or Body*
  EXPECT_EQ(bbox1.min(), bbox2.min());
  EXPECT_EQ(bbox1.max(), bbox2.max());
  EXPECT_EQ(bbox1.min(), bbox3.min());
  EXPECT_EQ(bbox1.max(), bbox3.max());
  EXPECT_EQ(obb1.getExtents(), obb2.getExtents());
  expectTransformsDoubleEq(obb1.getPose(), obb2.getPose());

  EXPECT_DOUBLE_EQ(4.0, obb1.getExtents().x());
  EXPECT_DOUBLE_EQ(4.0, obb1.getExtents().y());
  EXPECT_DOUBLE_EQ(4.0, obb1.getExtents().z());
  EXPECT_DOUBLE_EQ(1.0, obb1.getPose().translation().x());
  EXPECT_DOUBLE_EQ(2.0, obb1.getPose().translation().y());
  EXPECT_DOUBLE_EQ(3.0, obb1.getPose().translation().z());

  // we intentionally reuse the non-empty bboxes to check they are zeroed-out
  computeBoundingBoxAt(static_cast<const bodies::Sphere*>(nullptr), bbox3, pose);
  computeBoundingBox(static_cast<const bodies::Sphere*>(nullptr), obb1);
  computeBoundingBox(static_cast<const bodies::Body*>(nullptr), obb2);

  EXPECT_TRUE(bbox3.isEmpty());
  expectTransformsDoubleEq(obb1.getPose(), Eigen::Isometry3d::Identity());
  EXPECT_VECTORS_EQUAL(obb1.getExtents(), Eigen::Vector3d::Zero(), 1e-12)
  expectTransformsDoubleEq(obb2.getPose(), Eigen::Isometry3d::Identity());
  EXPECT_VECTORS_EQUAL(obb2.getExtents(), Eigen::Vector3d::Zero(), 1e-12)

  delete sphere;
  delete shape;
}

TEST(Bodies, ComputeBoundingBoxBox)
{
  AxisAlignedBoundingBox bbox1, bbox2, bbox3;
  OrientedBoundingBox obb1, obb2;

  const Eigen::Isometry3d pose = Eigen::Translation3d(1.0, 2.0, 3.0) *
      Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d(0.0, 1.0, 0.0));

  const shapes::Shape* shape = new shapes::Box(1.0, 2.0, 3.0);
  const auto box = new Box(shape);
  const auto body = dynamic_cast<Body*>(box);

  computeBoundingBoxAt(box, bbox3, pose);

  body->setPose(pose);

  box->computeBoundingBox(bbox1);
  body->computeBoundingBox(bbox2);
  computeBoundingBox(box, obb1);
  computeBoundingBox(body, obb2);

  // check that bboxes are equal regardless we ask through the exact type or Body*
  EXPECT_EQ(bbox1.min(), bbox2.min());
  EXPECT_EQ(bbox1.max(), bbox2.max());
  EXPECT_EQ(bbox1.min(), bbox3.min());
  EXPECT_EQ(bbox1.max(), bbox3.max());
  EXPECT_EQ(obb1.getExtents(), obb2.getExtents());
  expectTransformsDoubleEq(obb1.getPose(), obb2.getPose());

  // for box primitives, we require that the OBB is equal to the box itself
  EXPECT_DOUBLE_EQ(1.0, obb1.getExtents().x());
  EXPECT_DOUBLE_EQ(2.0, obb1.getExtents().y());
  EXPECT_DOUBLE_EQ(3.0, obb1.getExtents().z());
  EXPECT_DOUBLE_EQ(1.0, obb1.getPose().translation().x());
  EXPECT_DOUBLE_EQ(2.0, obb1.getPose().translation().y());
  EXPECT_DOUBLE_EQ(3.0, obb1.getPose().translation().z());

  // we intentionally reuse the non-empty bboxes to check they are zeroed-out
  computeBoundingBoxAt(static_cast<const bodies::Box*>(nullptr), bbox3, pose);
  computeBoundingBox(static_cast<const bodies::Box*>(nullptr), obb1);
  computeBoundingBox(static_cast<const bodies::Body*>(nullptr), obb2);

  EXPECT_TRUE(bbox3.isEmpty());
  expectTransformsDoubleEq(obb1.getPose(), Eigen::Isometry3d::Identity());
  EXPECT_VECTORS_EQUAL(obb1.getExtents(), Eigen::Vector3d::Zero(), 1e-12)
  expectTransformsDoubleEq(obb2.getPose(), Eigen::Isometry3d::Identity());
  EXPECT_VECTORS_EQUAL(obb2.getExtents(), Eigen::Vector3d::Zero(), 1e-12)

  delete box;
  delete shape;
}

TEST(Bodies, ComputeBoundingBoxCylinder)
{
  AxisAlignedBoundingBox bbox1, bbox2, bbox3;
  OrientedBoundingBox obb1, obb2;

  const Eigen::Isometry3d pose = Eigen::Translation3d(1.0, 2.0, 3.0) *
      Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d(0.0, 1.0, 0.0));

  const shapes::Shape* shape = new shapes::Cylinder(2.0, 3.0);
  const auto cylinder = new Cylinder(shape);
  const auto body = dynamic_cast<Body*>(cylinder);

  computeBoundingBoxAt(cylinder, bbox3, pose);

  body->setPose(pose);

  cylinder->computeBoundingBox(bbox1);
  body->computeBoundingBox(bbox2);
  computeBoundingBox(cylinder, obb1);
  computeBoundingBox(body, obb2);

  // check that bboxes are equal regardless we ask through the exact type or Body*
  EXPECT_EQ(bbox1.min(), bbox2.min());
  EXPECT_EQ(bbox1.max(), bbox2.max());
  EXPECT_EQ(bbox1.min(), bbox3.min());
  EXPECT_EQ(bbox1.max(), bbox3.max());
  EXPECT_EQ(obb1.getExtents(), obb2.getExtents());
  expectTransformsDoubleEq(obb1.getPose(), obb2.getPose());

  // for cylinder primitives, we require that the OBB's z-axis is aligned with the cylinder's
  // rotational axis
  EXPECT_DOUBLE_EQ(4.0, obb1.getExtents().x());
  EXPECT_DOUBLE_EQ(4.0, obb1.getExtents().y());
  EXPECT_DOUBLE_EQ(3.0, obb1.getExtents().z());
  EXPECT_DOUBLE_EQ(1.0, obb1.getPose().translation().x());
  EXPECT_DOUBLE_EQ(2.0, obb1.getPose().translation().y());
  EXPECT_DOUBLE_EQ(3.0, obb1.getPose().translation().z());

  // we intentionally reuse the non-empty bboxes to check they are zeroed-out
  computeBoundingBoxAt(static_cast<const bodies::Cylinder*>(nullptr), bbox3, pose);
  computeBoundingBox(static_cast<const bodies::Cylinder*>(nullptr), obb1);
  computeBoundingBox(static_cast<const bodies::Body*>(nullptr), obb2);

  EXPECT_TRUE(bbox3.isEmpty());
  expectTransformsDoubleEq(obb1.getPose(), Eigen::Isometry3d::Identity());
  EXPECT_VECTORS_EQUAL(obb1.getExtents(), Eigen::Vector3d::Zero(), 1e-12)
  expectTransformsDoubleEq(obb2.getPose(), Eigen::Isometry3d::Identity());
  EXPECT_VECTORS_EQUAL(obb2.getExtents(), Eigen::Vector3d::Zero(), 1e-12)

  delete cylinder;
  delete shape;
}

TEST(Bodies, ComputeBoundingBoxConvexMesh)
{
  AxisAlignedBoundingBox bbox1, bbox2, bbox3;
  OrientedBoundingBox obb1, obb2;

  const Eigen::Isometry3d pose = Eigen::Translation3d(1.0, 2.0, 3.0) *
      Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d(0.0, 1.0, 0.0));

  // box (1.0, 2.0, 3.0)
  const shapes::Shape* shape = shapes::createMeshFromResource(
      "package://robot_body_filter/test/box.dae");
  const auto mesh = new ConvexMesh(shape);
  const auto body = dynamic_cast<Body*>(mesh);

  computeBoundingBoxAt(mesh, bbox3, pose);

  body->setPose(pose);

  mesh->computeBoundingBox(bbox1);
  body->computeBoundingBox(bbox2);
  computeBoundingBox(mesh, obb1);
  computeBoundingBox(body, obb2);

  // check that bboxes are equal regardless we ask through the exact type or Body*
  EXPECT_EQ(bbox1.min(), bbox2.min());
  EXPECT_EQ(bbox1.max(), bbox2.max());
  EXPECT_EQ(bbox1.min(), bbox3.min());
  EXPECT_EQ(bbox1.max(), bbox3.max());
  EXPECT_EQ(obb1.getExtents(), obb2.getExtents());
  expectTransformsDoubleEq(obb1.getPose(), obb2.getPose());

  // there's not a single unique box that would be considered the correct OBB for a generic mesh
  // so we need to treat any rotation of the box correct
  const std::set<std::tuple<double, double, double>> permutations = {
      {1.0, 2.0, 3.0},
      {2.0, 3.0, 1.0},
      {3.0, 1.0, 2.0},
      {2.0, 1.0, 3.0},
      {1.0, 3.0, 2.0},
      {3.0, 2.0, 1.0},
  };
  bool matches = false;
  double maxErr = 1e-6;
  for (const auto ext : permutations)
  {
    if (fabs(obb1.getExtents().x() - std::get<0>(ext)) < maxErr &&
        fabs(obb1.getExtents().y() - std::get<1>(ext)) < maxErr &&
        fabs(obb1.getExtents().z() - std::get<2>(ext)) < maxErr)
    {
      matches = true;
      break;
    }
  }
  EXPECT_TRUE(matches);

  EXPECT_DOUBLE_EQ(1.0, obb1.getPose().translation().x());
  EXPECT_DOUBLE_EQ(2.0, obb1.getPose().translation().y());
  EXPECT_DOUBLE_EQ(3.0, obb1.getPose().translation().z());

  // we intentionally reuse the non-empty bboxes to check they are zeroed-out
  computeBoundingBoxAt(static_cast<const bodies::ConvexMesh*>(nullptr), bbox3, pose);
  computeBoundingBox(static_cast<const bodies::ConvexMesh*>(nullptr), obb1);
  computeBoundingBox(static_cast<const bodies::Body*>(nullptr), obb2);

  EXPECT_TRUE(bbox3.isEmpty());
  expectTransformsDoubleEq(obb1.getPose(), Eigen::Isometry3d::Identity());
  EXPECT_VECTORS_EQUAL(obb1.getExtents(), Eigen::Vector3d::Zero(), 1e-12)
  expectTransformsDoubleEq(obb2.getPose(), Eigen::Isometry3d::Identity());
  EXPECT_VECTORS_EQUAL(obb2.getExtents(), Eigen::Vector3d::Zero(), 1e-12)

  delete mesh;
  delete shape;
}

TEST(Bodies, MergeOBBs)
{
  OrientedBoundingBox bbox1, bbox2, mergedBbox;

  bbox1.setPoseAndExtents(
      Eigen::Translation3d(1.0, 1.5, 2.0) * Eigen::Quaterniond::Identity(),
      {2.0, 3.0, 4.0});

  bbox2.setPoseAndExtents(
      Eigen::Translation3d(-1.0, -1.5, -2.0) * Eigen::Quaterniond::Identity(),
      {2.0, 3.0, 4.0});

  mergeOrientedBoundingBoxesApprox({bbox1, bbox2}, mergedBbox);

  EXPECT_DOUBLE_EQ(4.0, mergedBbox.getExtents().x());
  EXPECT_DOUBLE_EQ(6.0, mergedBbox.getExtents().y());
  EXPECT_DOUBLE_EQ(8.0, mergedBbox.getExtents().z());
  EXPECT_DOUBLE_EQ(0.0, mergedBbox.getPose().translation().x());
  EXPECT_DOUBLE_EQ(0.0, mergedBbox.getPose().translation().y());
  EXPECT_DOUBLE_EQ(0.0, mergedBbox.getPose().translation().z());
}

TEST(Bodies, ConstructShapeFromBodyWrong)
{
  EXPECT_EQ(shapes::ShapePtr(), constructShapeFromBody(nullptr));

  const auto unknown = WrongBody(::shapes::ShapeType::UNKNOWN_SHAPE);
  EXPECT_EQ(shapes::ShapePtr(), constructShapeFromBody(&unknown));

  const auto cone = WrongBody(::shapes::ShapeType::CONE);
  EXPECT_EQ(shapes::ShapePtr(), constructShapeFromBody(&cone));

  const auto octree = WrongBody(::shapes::ShapeType::OCTREE);
  EXPECT_EQ(shapes::ShapePtr(), constructShapeFromBody(&octree));

  const auto plane = WrongBody(::shapes::ShapeType::PLANE);
  EXPECT_EQ(shapes::ShapePtr(), constructShapeFromBody(&plane));
}

TEST(Bodies, ConstructShapeFromBodySphere)
{
  const shapes::Shape* shape = new shapes::Sphere(2.0);
  const auto body = new Sphere(shape);

  const auto constructedShape = constructShapeFromBody(body);
  const auto constructedSphere = std::dynamic_pointer_cast<const shapes::Sphere>(constructedShape);

  EXPECT_EQ(shape->type, constructedShape->type);
  ASSERT_NE(nullptr, constructedSphere);
  EXPECT_EQ(2.0, constructedSphere->radius);
}

TEST(Bodies, ConstructShapeFromBodyBox)
{
  const shapes::Shape* shape = new shapes::Box(1.0, 2.0, 3.0);
  const auto body = new Box(shape);

  const auto constructedShape = constructShapeFromBody(body);
  const auto constructedBox = std::dynamic_pointer_cast<const shapes::Box>(constructedShape);

  EXPECT_EQ(shape->type, constructedShape->type);
  ASSERT_NE(nullptr, constructedBox);
  EXPECT_EQ(1.0, constructedBox->size[0]);
  EXPECT_EQ(2.0, constructedBox->size[1]);
  EXPECT_EQ(3.0, constructedBox->size[2]);
}

TEST(Bodies, ConstructShapeFromBodyCylinder)
{
  const shapes::Shape* shape = new shapes::Cylinder(1.0, 2.0);
  const auto body = new Cylinder(shape);

  const auto constructedShape = constructShapeFromBody(body);
  const auto constructedCylinder =
      std::dynamic_pointer_cast<const shapes::Cylinder>(constructedShape);

  EXPECT_EQ(shape->type, constructedShape->type);
  ASSERT_NE(nullptr, constructedCylinder);
  EXPECT_EQ(1.0, constructedCylinder->radius);
  EXPECT_EQ(2.0, constructedCylinder->length);
}

TEST(Bodies, ConstructShapeFromBodyMesh)
{
  shapes::Mesh* shape = shapes::createMeshFromResource(
      "package://robot_body_filter/test/box.dae");
  const auto body = new ConvexMesh(shape);

  const auto constructedShape = constructShapeFromBody(body);
  const auto constructedMesh =
      std::dynamic_pointer_cast<const shapes::Mesh>(constructedShape);

  EXPECT_EQ(shape->type, constructedShape->type);
  ASSERT_NE(nullptr, constructedMesh);
  ASSERT_EQ(shape->vertex_count, constructedMesh->vertex_count);
  ASSERT_EQ(shape->triangle_count, constructedMesh->triangle_count);

  // Compare the vertices and triangle normals of the constructed mesh
  // Triangle indices cannot be checked because the vertex IDs could change in the constructed mesh

  EigenSTL::vector_Vector3d verticesOrig, verticesConstructed;
  for (size_t i = 0; i < shape->vertex_count * 3; i += 3)
    verticesOrig.push_back({shape->vertices[i], shape->vertices[i+1], shape->vertices[i+2]});
  for (size_t i = 0; i < constructedMesh->vertex_count * 3; i += 3)
    verticesConstructed.push_back({constructedMesh->vertices[i], constructedMesh->vertices[i+1],
                                constructedMesh->vertices[i+2]});

  expectVector3dSetsEqual(verticesOrig, verticesConstructed);

  EigenSTL::vector_Vector3d normalsOrig, normalsConstructed;
  shape->computeTriangleNormals();
  // constructedMesh->computeTriangleNormals();  // is done during construction
  for (size_t i = 0; i < shape->triangle_count * 3; i += 3)
    normalsOrig.push_back({shape->triangle_normals[i], shape->triangle_normals[i+1],
                           shape->triangle_normals[i+2]});
  for (size_t i = 0; i < constructedMesh->triangle_count * 3; i += 3)
    normalsConstructed.push_back({constructedMesh->triangle_normals[i],
                                  constructedMesh->triangle_normals[i+1],
                                  constructedMesh->triangle_normals[i+2]});

  expectVector3dSetsEqual(normalsOrig, normalsConstructed);
}

TEST(Bodies, ConstructMarkerFromBodySphere)
{
  const Eigen::Isometry3d pose = Eigen::Translation3d(1.0, 2.0, 3.0) *
      Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d(0.0, 1.0, 0.0));

  const shapes::Shape* shape = new shapes::Sphere(2.0);
  const auto body = new Sphere(shape);
  body->setPose(pose);

  visualization_msgs::Marker marker;
  constructMarkerFromBody(body, marker);

  EXPECT_EQ(visualization_msgs::Marker::SPHERE, marker.type);
  EXPECT_DOUBLE_EQ(4.0, marker.scale.x);
  EXPECT_DOUBLE_EQ(4.0, marker.scale.y);
  EXPECT_DOUBLE_EQ(4.0, marker.scale.z);
  EXPECT_DOUBLE_EQ(1.0, marker.pose.position.x);
  EXPECT_DOUBLE_EQ(2.0, marker.pose.position.y);
  EXPECT_DOUBLE_EQ(3.0, marker.pose.position.z);
  EXPECT_DOUBLE_EQ(0.0, marker.pose.orientation.x);
  EXPECT_DOUBLE_EQ(M_SQRT1_2, marker.pose.orientation.y);
  EXPECT_DOUBLE_EQ(0.0, marker.pose.orientation.z);
  EXPECT_DOUBLE_EQ(M_SQRT1_2, marker.pose.orientation.w);
}

TEST(Bodies, ConstructMarkerFromBodyBox)
{
  const Eigen::Isometry3d pose = Eigen::Translation3d(1.0, 2.0, 3.0) *
      Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d(0.0, 1.0, 0.0));

  const shapes::Shape* shape = new shapes::Box(1.0, 2.0, 3.0);
  const auto body = new Box(shape);
  body->setPose(pose);

  visualization_msgs::Marker marker;
  constructMarkerFromBody(body, marker);

  EXPECT_EQ(visualization_msgs::Marker::CUBE, marker.type);
  EXPECT_DOUBLE_EQ(1.0, marker.scale.x);
  EXPECT_DOUBLE_EQ(2.0, marker.scale.y);
  EXPECT_DOUBLE_EQ(3.0, marker.scale.z);
  EXPECT_DOUBLE_EQ(1.0, marker.pose.position.x);
  EXPECT_DOUBLE_EQ(2.0, marker.pose.position.y);
  EXPECT_DOUBLE_EQ(3.0, marker.pose.position.z);
  EXPECT_DOUBLE_EQ(0.0, marker.pose.orientation.x);
  EXPECT_DOUBLE_EQ(M_SQRT1_2, marker.pose.orientation.y);
  EXPECT_DOUBLE_EQ(0.0, marker.pose.orientation.z);
  EXPECT_DOUBLE_EQ(M_SQRT1_2, marker.pose.orientation.w);
}

TEST(Bodies, ConstructMarkerFromBodyCylinder)
{
  const Eigen::Isometry3d pose = Eigen::Translation3d(1.0, 2.0, 3.0) *
      Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d(0.0, 1.0, 0.0));

  const shapes::Shape* shape = new shapes::Cylinder(3.0, 2.0);
  const auto body = new Cylinder(shape);
  body->setPose(pose);

  visualization_msgs::Marker marker;
  constructMarkerFromBody(body, marker);

  EXPECT_EQ(visualization_msgs::Marker::CYLINDER, marker.type);
  EXPECT_DOUBLE_EQ(6.0, marker.scale.x);
  EXPECT_DOUBLE_EQ(6.0, marker.scale.y);
  EXPECT_DOUBLE_EQ(2.0, marker.scale.z);
  EXPECT_DOUBLE_EQ(1.0, marker.pose.position.x);
  EXPECT_DOUBLE_EQ(2.0, marker.pose.position.y);
  EXPECT_DOUBLE_EQ(3.0, marker.pose.position.z);
  EXPECT_DOUBLE_EQ(0.0, marker.pose.orientation.x);
  EXPECT_DOUBLE_EQ(M_SQRT1_2, marker.pose.orientation.y);
  EXPECT_DOUBLE_EQ(0.0, marker.pose.orientation.z);
  EXPECT_DOUBLE_EQ(M_SQRT1_2, marker.pose.orientation.w);
}

TEST(Bodies, ConstructMarkerFromBodyMesh)
{
  const Eigen::Isometry3d pose = Eigen::Translation3d(1.0, 2.0, 3.0) *
      Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d(0.0, 1.0, 0.0));

  shapes::Mesh* shape = shapes::createMeshFromResource(
      "package://robot_body_filter/test/box.dae");
  const auto body = new ConvexMesh(shape);
  body->setPose(pose);

  visualization_msgs::Marker marker;
  constructMarkerFromBody(body, marker);

  EXPECT_EQ(visualization_msgs::Marker::TRIANGLE_LIST, marker.type);
  EXPECT_DOUBLE_EQ(1.0, marker.scale.x);
  EXPECT_DOUBLE_EQ(1.0, marker.scale.y);
  EXPECT_DOUBLE_EQ(1.0, marker.scale.z);
  EXPECT_DOUBLE_EQ(1.0, marker.pose.position.x);
  EXPECT_DOUBLE_EQ(2.0, marker.pose.position.y);
  EXPECT_DOUBLE_EQ(3.0, marker.pose.position.z);
  EXPECT_DOUBLE_EQ(0.0, marker.pose.orientation.x);
  EXPECT_DOUBLE_EQ(M_SQRT1_2, marker.pose.orientation.y);
  EXPECT_DOUBLE_EQ(0.0, marker.pose.orientation.z);
  EXPECT_DOUBLE_EQ(M_SQRT1_2, marker.pose.orientation.w);

  // We can't directly use shape->triangles because after passing it to the body constructor,
  // it can "optimize" the vertices (i.e. swap normals etc.) or reorder them
  const auto shapeFromBody =
      std::dynamic_pointer_cast<const shapes::Mesh>(constructShapeFromBody(body));

  EigenSTL::vector_Vector3d shapeVertices, markerVertices;
  for (size_t t = 0; t < shapeFromBody->triangle_count * 3; ++t)
  {
    const auto vertexId = shapeFromBody->triangles[t];
    shapeVertices.push_back({
      shapeFromBody->vertices[3 * vertexId + 0],
      shapeFromBody->vertices[3 * vertexId + 1],
      shapeFromBody->vertices[3 * vertexId + 2]
    });
  }
  for (size_t i = 0; i < marker.points.size(); ++i)
    markerVertices.push_back({marker.points[i].x, marker.points[i].y, marker.points[i].z});

  expectVector3dSetsEqual(shapeVertices, markerVertices);
}

#define EXPECT_VECTORS_EQUAL(v1, v2, error)                                                                            \
  EXPECT_NEAR((v1)[0], (v2)[0], (error));                                                                              \
  EXPECT_NEAR((v1)[1], (v2)[1], (error));                                                                              \
  EXPECT_NEAR((v1)[2], (v2)[2], (error));

#define CHECK_INTERSECTS_TWICE(body, origin, direction, intersc1, intersc2, error)                                     \
  {                                                                                                                    \
    EigenSTL::vector_Vector3d intersections;                                                                           \
    Eigen::Vector3d o origin;                                                                                          \
    Eigen::Vector3d d direction;                                                                                       \
    Eigen::Vector3d i1 intersc1;                                                                                       \
    Eigen::Vector3d i2 intersc2;                                                                                       \
    const auto result = (body).intersectsRay(o, d, &intersections, 2);                                                \
    EXPECT_TRUE(result);                                                                                               \
    ASSERT_EQ(2, intersections.size());                                                                                \
    if (fabs(static_cast<double>((intersections.at(0) - i1).norm())) < (error))                                        \
    {                                                                                                                  \
      EXPECT_VECTORS_EQUAL(intersections.at(0), i1, (error));                                                          \
      EXPECT_VECTORS_EQUAL(intersections.at(1), i2, (error));                                                          \
    }                                                                                                                  \
    else                                                                                                               \
    {                                                                                                                  \
      EXPECT_VECTORS_EQUAL(intersections.at(0), i2, (error));                                                          \
      EXPECT_VECTORS_EQUAL(intersections.at(1), i1, (error));                                                          \
    }                                                                                                                  \
  }

// This tests if https://github.com/ros-planning/geometric_shapes/pull/109 is fixed
TEST(BoxRayIntersection, FailedInUpstream)
{
  shapes::Box shape(1.0, 1.0, 1.0);
  bodies::Box box(&shape);

  // rotates the box so that the original (0.5, 0.5, 0.5) corner gets elsewhere and is no longer the
  // maximal corner
  const auto rot = Eigen::AngleAxisd(M_PI * 2 / 3, Eigen::Vector3d(1.0, -1.0, 1.0).normalized());
  box.setPose(Eigen::Isometry3d(rot));

  CHECK_INTERSECTS_TWICE(box, (-2, 0, 0), (1, 0, 0), (0.5, 0, 0), (-0.5, 0, 0), 1e-6)
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}