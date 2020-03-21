#include "gtest/gtest.h"
#include <set>
#include <robot_body_filter/utils/bodies.h>
#include <geometric_shapes/mesh_operations.h>
#include "utils.cpp"

using namespace bodies;

TEST(Bodies, ComputeBoundingBoxSphere)
{
  AxisAlignedBoundingBox bbox1, bbox2;
  OrientedBoundingBox obb1, obb2;

  const Eigen::Isometry3d pose = Eigen::Translation3d(1.0, 2.0, 3.0) *
      Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d(0.0, 1.0, 0.0));

  const shapes::Shape* shape = new shapes::Sphere(2.0);
  const auto sphere = new Sphere(shape);
  const auto body = dynamic_cast<Body*>(sphere);
  body->setPose(pose);

  computeBoundingBox(sphere, bbox1);
  computeBoundingBox(body, bbox2);
  computeBoundingBox(sphere, obb1);
  computeBoundingBox(body, obb2);

  // check that bboxes are equal regardless we ask through the exact type or Body*
  EXPECT_EQ(bbox1.min(), bbox2.min());
  EXPECT_EQ(bbox1.max(), bbox2.max());
  EXPECT_EQ(obb1.getExtents(), obb2.getExtents());
  EXPECT_EQ(obb1.getPose().translation(), obb2.getPose().translation());
  EXPECT_EQ(obb1.getPose().rotation(), obb2.getPose().rotation());

  EXPECT_DOUBLE_EQ(-1.0, bbox1.min().x());
  EXPECT_DOUBLE_EQ(0.0, bbox1.min().y());
  EXPECT_DOUBLE_EQ(1.0, bbox1.min().z());
  EXPECT_DOUBLE_EQ(3.0, bbox1.max().x());
  EXPECT_DOUBLE_EQ(4.0, bbox1.max().y());
  EXPECT_DOUBLE_EQ(5.0, bbox1.max().z());
  EXPECT_DOUBLE_EQ(4.0, obb1.getExtents().x());
  EXPECT_DOUBLE_EQ(4.0, obb1.getExtents().y());
  EXPECT_DOUBLE_EQ(4.0, obb1.getExtents().z());
  EXPECT_DOUBLE_EQ(1.0, obb1.getPose().translation().x());
  EXPECT_DOUBLE_EQ(2.0, obb1.getPose().translation().y());
  EXPECT_DOUBLE_EQ(3.0, obb1.getPose().translation().z());
}

TEST(Bodies, ComputeBoundingBoxBox)
{
  AxisAlignedBoundingBox bbox1, bbox2;
  OrientedBoundingBox obb1, obb2;

  const Eigen::Isometry3d pose = Eigen::Translation3d(1.0, 2.0, 3.0) *
      Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d(0.0, 1.0, 0.0));

  const shapes::Shape* shape = new shapes::Box(1.0, 2.0, 3.0);
  const auto box = new Box(shape);
  const auto body = dynamic_cast<Body*>(box);
  body->setPose(pose);

  computeBoundingBox(box, bbox1);
  computeBoundingBox(body, bbox2);
  computeBoundingBox(box, obb1);
  computeBoundingBox(body, obb2);

  // check that bboxes are equal regardless we ask through the exact type or Body*
  EXPECT_EQ(bbox1.min(), bbox2.min());
  EXPECT_EQ(bbox1.max(), bbox2.max());
  EXPECT_EQ(obb1.getExtents(), obb2.getExtents());
  EXPECT_EQ(obb1.getPose().translation(), obb2.getPose().translation());
  EXPECT_EQ(obb1.getPose().rotation(), obb2.getPose().rotation());

  EXPECT_DOUBLE_EQ(-0.5, bbox1.min().x());
  EXPECT_DOUBLE_EQ(1.0, bbox1.min().y());
  EXPECT_DOUBLE_EQ(2.5, bbox1.min().z());
  EXPECT_DOUBLE_EQ(2.5, bbox1.max().x());
  EXPECT_DOUBLE_EQ(3.0, bbox1.max().y());
  EXPECT_DOUBLE_EQ(3.5, bbox1.max().z());
  // for box primitives, we require that the OBB is equal to the box itself
  EXPECT_DOUBLE_EQ(1.0, obb1.getExtents().x());
  EXPECT_DOUBLE_EQ(2.0, obb1.getExtents().y());
  EXPECT_DOUBLE_EQ(3.0, obb1.getExtents().z());
  EXPECT_DOUBLE_EQ(1.0, obb1.getPose().translation().x());
  EXPECT_DOUBLE_EQ(2.0, obb1.getPose().translation().y());
  EXPECT_DOUBLE_EQ(3.0, obb1.getPose().translation().z());
}

TEST(Bodies, ComputeBoundingBoxCylinder)
{
  AxisAlignedBoundingBox bbox1, bbox2;
  OrientedBoundingBox obb1, obb2;

  const Eigen::Isometry3d pose = Eigen::Translation3d(1.0, 2.0, 3.0) *
      Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d(0.0, 1.0, 0.0));

  const shapes::Shape* shape = new shapes::Cylinder(2.0, 3.0);
  const auto cylinder = new Cylinder(shape);
  const auto body = dynamic_cast<Body*>(cylinder);
  body->setPose(pose);

  computeBoundingBox(cylinder, bbox1);
  computeBoundingBox(body, bbox2);
  computeBoundingBox(cylinder, obb1);
  computeBoundingBox(body, obb2);

  // check that bboxes are equal regardless we ask through the exact type or Body*
  EXPECT_EQ(bbox1.min(), bbox2.min());
  EXPECT_EQ(bbox1.max(), bbox2.max());
  EXPECT_EQ(obb1.getExtents(), obb2.getExtents());
  EXPECT_EQ(obb1.getPose().translation(), obb2.getPose().translation());
  EXPECT_EQ(obb1.getPose().rotation(), obb2.getPose().rotation());

  EXPECT_DOUBLE_EQ(-0.5, bbox1.min().x());
  EXPECT_DOUBLE_EQ(0.0, bbox1.min().y());
  EXPECT_DOUBLE_EQ(1.0, bbox1.min().z());
  EXPECT_DOUBLE_EQ(2.5, bbox1.max().x());
  EXPECT_DOUBLE_EQ(4.0, bbox1.max().y());
  EXPECT_DOUBLE_EQ(5.0, bbox1.max().z());
  // for cylinder primitives, we require that the OBB's z-axis is aligned with the cylinder's
  // rotational axis
  EXPECT_DOUBLE_EQ(4.0, obb1.getExtents().x());
  EXPECT_DOUBLE_EQ(4.0, obb1.getExtents().y());
  EXPECT_DOUBLE_EQ(3.0, obb1.getExtents().z());
  EXPECT_DOUBLE_EQ(1.0, obb1.getPose().translation().x());
  EXPECT_DOUBLE_EQ(2.0, obb1.getPose().translation().y());
  EXPECT_DOUBLE_EQ(3.0, obb1.getPose().translation().z());
}

TEST(Bodies, ComputeBoundingBoxConvexMesh)
{
  AxisAlignedBoundingBox bbox1, bbox2;
  OrientedBoundingBox obb1, obb2;

  const Eigen::Isometry3d pose = Eigen::Translation3d(1.0, 2.0, 3.0) *
      Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d(0.0, 1.0, 0.0));

  // box (1.0, 2.0, 3.0)
  const shapes::Shape* shape = shapes::createMeshFromResource(
      "package://robot_body_filter/test/box.dae");
  const auto mesh = new ConvexMesh(shape);
  const auto body = dynamic_cast<Body*>(mesh);
  body->setPose(pose);

  computeBoundingBox(mesh, bbox1);
  computeBoundingBox(body, bbox2);
  computeBoundingBox(mesh, obb1);
  computeBoundingBox(body, obb2);

  // check that bboxes are equal regardless we ask through the exact type or Body*
  EXPECT_EQ(bbox1.min(), bbox2.min());
  EXPECT_EQ(bbox1.max(), bbox2.max());
  EXPECT_EQ(obb1.getExtents(), obb2.getExtents());
  EXPECT_EQ(obb1.getPose().translation(), obb2.getPose().translation());
  EXPECT_EQ(obb1.getPose().rotation(), obb2.getPose().rotation());

  EXPECT_DOUBLE_EQ(-0.5, bbox1.min().x());
  EXPECT_DOUBLE_EQ(1.0, bbox1.min().y());
  EXPECT_DOUBLE_EQ(2.5, bbox1.min().z());
  EXPECT_DOUBLE_EQ(2.5, bbox1.max().x());
  EXPECT_DOUBLE_EQ(3.0, bbox1.max().y());
  EXPECT_DOUBLE_EQ(3.5, bbox1.max().z());

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
}

TEST(Bodies, MergeAABBs)
{
  AxisAlignedBoundingBox bbox1, bbox2, mergedBbox;

  bbox1.min() = {0.0, 0.0, 0.0};
  bbox1.max() = {2.0, 3.0, 4.0};

  bbox2.min() = {-2.0, -3.0, -4.0};
  bbox2.max() = {0.0, 0.0, 0.0};

  mergeAxisAlignedBoundingBoxes({bbox1, bbox2}, mergedBbox);

  ASSERT_EQ(3, mergedBbox.dim());
  EXPECT_DOUBLE_EQ(-2.0, mergedBbox.min().x());
  EXPECT_DOUBLE_EQ(-3.0, mergedBbox.min().y());
  EXPECT_DOUBLE_EQ(-4.0, mergedBbox.min().z());
  EXPECT_DOUBLE_EQ(2.0, mergedBbox.max().x());
  EXPECT_DOUBLE_EQ(3.0, mergedBbox.max().y());
  EXPECT_DOUBLE_EQ(4.0, mergedBbox.max().z());
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

// The following tests are from https://github.com/ros-planning/geometric_shapes/pull/109,
// they're just edited so that they use the implementation from bodies::intersectsRay

#define EXPECT_VECTORS_EQUAL(v1, v2, error)                                                                            \
  EXPECT_NEAR((v1)[0], (v2)[0], (error));                                                                              \
  EXPECT_NEAR((v1)[1], (v2)[1], (error));                                                                              \
  EXPECT_NEAR((v1)[2], (v2)[2], (error));

#define CHECK_NO_INTERSECTION(body, origin, direction)                                                                 \
  {                                                                                                                    \
    EigenSTL::vector_Vector3d intersections;                                                                           \
    Eigen::Vector3d o origin;                                                                                          \
    Eigen::Vector3d d direction;                                                                                       \
    const auto result = intersectsRay(&(body), o, d, &intersections, 2);                                               \
    EXPECT_FALSE(result);                                                                                              \
    EXPECT_EQ(0, intersections.size());                                                                                \
  }

#define CHECK_INTERSECTS(body, origin, direction, numIntersections)                                                    \
  {                                                                                                                    \
    EigenSTL::vector_Vector3d intersections;                                                                           \
    Eigen::Vector3d o origin;                                                                                          \
    Eigen::Vector3d d direction;                                                                                       \
    const auto result = intersectsRay(&(body), o, d, &intersections, 2);                                               \
    EXPECT_TRUE(result);                                                                                               \
    ASSERT_EQ((numIntersections), intersections.size());                                                               \
  }

#define CHECK_INTERSECTS_ONCE(body, origin, direction, intersection, error)                                            \
  {                                                                                                                    \
    EigenSTL::vector_Vector3d intersections;                                                                           \
    Eigen::Vector3d o origin;                                                                                          \
    Eigen::Vector3d d direction;                                                                                       \
    Eigen::Vector3d i intersection;                                                                                    \
    const auto result = intersectsRay(&(body), o, d, &intersections, 2);                                               \
    EXPECT_TRUE(result);                                                                                               \
    ASSERT_EQ(1, intersections.size());                                                                                \
    EXPECT_VECTORS_EQUAL(intersections.at(0), i, (error));                                                             \
  }

#define CHECK_INTERSECTS_TWICE(body, origin, direction, intersc1, intersc2, error)                                     \
  {                                                                                                                    \
    EigenSTL::vector_Vector3d intersections;                                                                           \
    Eigen::Vector3d o origin;                                                                                          \
    Eigen::Vector3d d direction;                                                                                       \
    Eigen::Vector3d i1 intersc1;                                                                                       \
    Eigen::Vector3d i2 intersc2;                                                                                       \
    const auto result = intersectsRay(&(body), o, d, &intersections, 2);                                               \
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

TEST(SphereRay, OriginInside)
{
  shapes::Sphere shape(1.0);
  bodies::Sphere sphere(&shape);

  // clang-format off
  CHECK_INTERSECTS_ONCE(sphere, (0, 0, 0), ( 1,  0,  0), ( 1,  0,  0), 1e-6)
  CHECK_INTERSECTS_ONCE(sphere, (0, 0, 0), (-1,  0,  0), (-1,  0,  0), 1e-6)
  CHECK_INTERSECTS_ONCE(sphere, (0, 0, 0), ( 0,  1,  0), ( 0,  1,  0), 1e-6)
  CHECK_INTERSECTS_ONCE(sphere, (0, 0, 0), ( 0, -1,  0), ( 0, -1,  0), 1e-6)
  CHECK_INTERSECTS_ONCE(sphere, (0, 0, 0), ( 0,  0,  1), ( 0,  0,  1), 1e-6)
  CHECK_INTERSECTS_ONCE(sphere, (0, 0, 0), ( 0,  0, -1), ( 0,  0, -1), 1e-6)
  // clang-format on

  // scaling

  sphere.setScale(1.1);
  // clang-format off
  CHECK_INTERSECTS_ONCE(sphere, (0, 0, 0), ( 1,  0,  0), ( 1.1,    0,    0), 1e-6)
  CHECK_INTERSECTS_ONCE(sphere, (0, 0, 0), (-1,  0,  0), (-1.1,    0,    0), 1e-6)
  CHECK_INTERSECTS_ONCE(sphere, (0, 0, 0), ( 0,  1,  0), ( 0,    1.1,    0), 1e-6)
  CHECK_INTERSECTS_ONCE(sphere, (0, 0, 0), ( 0, -1,  0), ( 0,   -1.1,    0), 1e-6)
  CHECK_INTERSECTS_ONCE(sphere, (0, 0, 0), ( 0,  0,  1), ( 0,      0,  1.1), 1e-6)
  CHECK_INTERSECTS_ONCE(sphere, (0, 0, 0), ( 0,  0, -1), ( 0,      0, -1.1), 1e-6)
  // clang-format on

  // move origin within the sphere

  // clang-format off
  CHECK_INTERSECTS_ONCE(sphere, (0.5, 0  , 0  ), ( 1,  0,  0), ( 1.1,    0,    0), 1e-6)
  CHECK_INTERSECTS_ONCE(sphere, (0.5, 0  , 0  ), (-1,  0,  0), (-1.1,    0,    0), 1e-6)
  CHECK_INTERSECTS_ONCE(sphere, (0  , 0.5, 0  ), ( 0,  1,  0), ( 0,    1.1,    0), 1e-6)
  CHECK_INTERSECTS_ONCE(sphere, (0  , 0.5, 0  ), ( 0, -1,  0), ( 0,   -1.1,    0), 1e-6)
  CHECK_INTERSECTS_ONCE(sphere, (0  , 0  , 0.5), ( 0,  0,  1), ( 0,      0,  1.1), 1e-6)
  CHECK_INTERSECTS_ONCE(sphere, (0  , 0  , 0.5), ( 0,  0, -1), ( 0,      0, -1.1), 1e-6)
  // clang-format on

  // move sphere

  Eigen::Isometry3d pose = Eigen::Translation3d(0.5, 0, 0) * Eigen::Quaterniond::Identity();
  sphere.setPose(pose);
  // clang-format off
  CHECK_INTERSECTS_ONCE(sphere, (0, 0, 0), ( 1, 0, 0), ( 1.6, 0, 0), 1e-6)
  CHECK_INTERSECTS_ONCE(sphere, (0, 0, 0), (-1, 0, 0), (-0.6, 0, 0), 1e-6)
  // clang-format on

  pose = Eigen::Translation3d(0, 0.5, 0) * Eigen::Quaterniond::Identity();
  sphere.setPose(pose);
  // clang-format off
  CHECK_INTERSECTS_ONCE(sphere, (0, 0, 0), ( 0,  1, 0), (0,  1.6, 0), 1e-6)
  CHECK_INTERSECTS_ONCE(sphere, (0, 0, 0), ( 0, -1, 0), (0, -0.6, 0), 1e-6)
  // clang-format on

  pose = Eigen::Translation3d(0, 0, 0.5) * Eigen::Quaterniond::Identity();
  sphere.setPose(pose);
  // clang-format off
  CHECK_INTERSECTS_ONCE(sphere, (0, 0, 0), ( 0, 0,  1), (0, 0,  1.6), 1e-6)
  CHECK_INTERSECTS_ONCE(sphere, (0, 0, 0), ( 0, 0, -1), (0, 0, -0.6), 1e-6)
  // clang-format on

  // 3D diagonal

  sphere.setPose(Eigen::Isometry3d::Identity());
  sphere.setScale(1.0);
  sphere.setPadding(0.1);

  const auto sq3 = sqrt(pow(1 + 0.1, 2) / 3);
  const auto dir3 = Eigen::Vector3d::Ones().normalized();
  // clang-format off
  CHECK_INTERSECTS_ONCE(sphere, (   0,    0,    0), ( dir3), ( sq3,  sq3,  sq3), 1e-4)
  CHECK_INTERSECTS_ONCE(sphere, ( 0.5,  0.5,  0.5), ( dir3), ( sq3,  sq3,  sq3), 1e-4)
  CHECK_INTERSECTS_ONCE(sphere, (-0.5, -0.5, -0.5), ( dir3), ( sq3,  sq3,  sq3), 1e-4)
  CHECK_INTERSECTS_ONCE(sphere, (   0,    0,    0), (-dir3), (-sq3, -sq3, -sq3), 1e-4)
  CHECK_INTERSECTS_ONCE(sphere, ( 0.5,  0.5,  0.5), (-dir3), (-sq3, -sq3, -sq3), 1e-4)
  CHECK_INTERSECTS_ONCE(sphere, (-0.5, -0.5, -0.5), (-dir3), (-sq3, -sq3, -sq3), 1e-4)
  // clang-format on

  // 2D diagonal

  const auto sq2 = sqrt(pow(1 + 0.1, 2) / 2);
  const auto dir2 = 1/sqrt(2);
  // clang-format off
  CHECK_INTERSECTS_ONCE(sphere, (   0,    0,    0), ( dir2,  dir2,     0), ( sq2,  sq2,    0), 1e-4)
  CHECK_INTERSECTS_ONCE(sphere, (   0,    0,    0), (-dir2, -dir2,     0), (-sq2, -sq2,    0), 1e-4)
  CHECK_INTERSECTS_ONCE(sphere, ( 0.5,  0.5,    0), ( dir2,  dir2,     0), ( sq2,  sq2,    0), 1e-4)
  CHECK_INTERSECTS_ONCE(sphere, ( 0.5,  0.5,    0), (-dir2, -dir2,     0), (-sq2, -sq2,    0), 1e-4)
  CHECK_INTERSECTS_ONCE(sphere, (-0.5, -0.5,    0), ( dir2,  dir2,     0), ( sq2,  sq2,    0), 1e-4)
  CHECK_INTERSECTS_ONCE(sphere, (-0.5, -0.5,    0), (-dir2, -dir2,     0), (-sq2, -sq2,    0), 1e-4)

  CHECK_INTERSECTS_ONCE(sphere, (   0,    0,    0), (    0,  dir2,  dir2), (   0,  sq2,  sq2), 1e-4)
  CHECK_INTERSECTS_ONCE(sphere, (   0,    0,    0), (    0, -dir2, -dir2), (   0, -sq2, -sq2), 1e-4)
  CHECK_INTERSECTS_ONCE(sphere, (   0,  0.5,  0.5), (    0,  dir2,  dir2), (   0,  sq2,  sq2), 1e-4)
  CHECK_INTERSECTS_ONCE(sphere, (   0,  0.5,  0.5), (    0, -dir2, -dir2), (   0, -sq2, -sq2), 1e-4)
  CHECK_INTERSECTS_ONCE(sphere, (   0, -0.5, -0.5), (    0,  dir2,  dir2), (   0,  sq2,  sq2), 1e-4)
  CHECK_INTERSECTS_ONCE(sphere, (   0, -0.5, -0.5), (    0, -dir2, -dir2), (   0, -sq2, -sq2), 1e-4)

  CHECK_INTERSECTS_ONCE(sphere, (   0,    0,    0), ( dir2,     0,  dir2), ( sq2,    0,  sq2), 1e-4)
  CHECK_INTERSECTS_ONCE(sphere, (   0,    0,    0), (-dir2,     0, -dir2), (-sq2,    0, -sq2), 1e-4)
  CHECK_INTERSECTS_ONCE(sphere, ( 0.5,    0,  0.5), ( dir2,     0,  dir2), ( sq2,    0,  sq2), 1e-4)
  CHECK_INTERSECTS_ONCE(sphere, ( 0.5,    0,  0.5), (-dir2,     0, -dir2), (-sq2,    0, -sq2), 1e-4)
  CHECK_INTERSECTS_ONCE(sphere, (-0.5,    0, -0.5), ( dir2,     0,  dir2), ( sq2,    0,  sq2), 1e-4)
  CHECK_INTERSECTS_ONCE(sphere, (-0.5,    0, -0.5), (-dir2,     0, -dir2), (-sq2,    0, -sq2), 1e-4)
  // clang-format on
}

TEST(SphereRay, OriginOutsideIntersects)
{
  shapes::Sphere shape(1.0);
  bodies::Sphere sphere(&shape);

  // clang-format off
  CHECK_INTERSECTS_TWICE(sphere, (-2, 0, 0), ( 1,  0,  0), ( 1,  0,  0), (-1,  0, 0), 1e-6)
  CHECK_INTERSECTS_TWICE(sphere, ( 2, 0, 0), (-1,  0,  0), (-1,  0,  0), ( 1,  0, 0), 1e-6)
  CHECK_INTERSECTS_TWICE(sphere, (0, -2, 0), ( 0,  1,  0), ( 0,  1,  0), ( 0, -1, 0), 1e-6)
  CHECK_INTERSECTS_TWICE(sphere, (0,  2, 0), ( 0, -1,  0), ( 0, -1,  0), ( 0,  1, 0), 1e-6)
  CHECK_INTERSECTS_TWICE(sphere, (0, 0, -2), ( 0,  0,  1), ( 0,  0,  1), ( 0,  0, -1), 1e-6)
  CHECK_INTERSECTS_TWICE(sphere, (0, 0,  2), ( 0,  0, -1), ( 0,  0, -1), ( 0,  0,  1), 1e-6)
  // clang-format on

  // scaling

  sphere.setScale(1.1);
  // clang-format off
  CHECK_INTERSECTS_ONCE(sphere, (0, 0, 0), ( 1,  0,  0), ( 1.1,    0,    0), 1e-6)
  CHECK_INTERSECTS_ONCE(sphere, (0, 0, 0), (-1,  0,  0), (-1.1,    0,    0), 1e-6)
  CHECK_INTERSECTS_ONCE(sphere, (0, 0, 0), ( 0,  1,  0), ( 0,    1.1,    0), 1e-6)
  CHECK_INTERSECTS_ONCE(sphere, (0, 0, 0), ( 0, -1,  0), ( 0,   -1.1,    0), 1e-6)
  CHECK_INTERSECTS_ONCE(sphere, (0, 0, 0), ( 0,  0,  1), ( 0,      0,  1.1), 1e-6)
  CHECK_INTERSECTS_ONCE(sphere, (0, 0, 0), ( 0,  0, -1), ( 0,      0, -1.1), 1e-6)
  // clang-format on

  // move origin within the sphere

  // clang-format off
  CHECK_INTERSECTS_ONCE(sphere, (0.5, 0  , 0  ), ( 1,  0,  0), ( 1.1,    0,    0), 1e-6)
  CHECK_INTERSECTS_ONCE(sphere, (0.5, 0  , 0  ), (-1,  0,  0), (-1.1,    0,    0), 1e-6)
  CHECK_INTERSECTS_ONCE(sphere, (0  , 0.5, 0  ), ( 0,  1,  0), ( 0,    1.1,    0), 1e-6)
  CHECK_INTERSECTS_ONCE(sphere, (0  , 0.5, 0  ), ( 0, -1,  0), ( 0,   -1.1,    0), 1e-6)
  CHECK_INTERSECTS_ONCE(sphere, (0  , 0  , 0.5), ( 0,  0,  1), ( 0,      0,  1.1), 1e-6)
  CHECK_INTERSECTS_ONCE(sphere, (0  , 0  , 0.5), ( 0,  0, -1), ( 0,      0, -1.1), 1e-6)
  // clang-format on

  // move sphere

  Eigen::Isometry3d pose = Eigen::Translation3d(0.5, 0, 0) * Eigen::Quaterniond::Identity();
  sphere.setPose(pose);
  // clang-format off
  CHECK_INTERSECTS_ONCE(sphere, (0, 0, 0), ( 1, 0, 0), ( 1.6, 0, 0), 1e-6)
  CHECK_INTERSECTS_ONCE(sphere, (0, 0, 0), (-1, 0, 0), (-0.6, 0, 0), 1e-6)
  // clang-format on

  pose = Eigen::Translation3d(0, 0.5, 0) * Eigen::Quaterniond::Identity();
  sphere.setPose(pose);
  // clang-format off
  CHECK_INTERSECTS_ONCE(sphere, (0, 0, 0), ( 0,  1, 0), (0,  1.6, 0), 1e-6)
  CHECK_INTERSECTS_ONCE(sphere, (0, 0, 0), ( 0, -1, 0), (0, -0.6, 0), 1e-6)
  // clang-format on

  pose = Eigen::Translation3d(0, 0, 0.5) * Eigen::Quaterniond::Identity();
  sphere.setPose(pose);
  // clang-format off
  CHECK_INTERSECTS_ONCE(sphere, (0, 0, 0), ( 0, 0,  1), (0, 0,  1.6), 1e-6)
  CHECK_INTERSECTS_ONCE(sphere, (0, 0, 0), ( 0, 0, -1), (0, 0, -0.6), 1e-6)
  // clang-format on

  // 3D diagonal

  sphere.setPose(Eigen::Isometry3d::Identity());
  sphere.setScale(1.0);
  sphere.setPadding(0.1);

  const auto sq3 = sqrt(pow(1 + 0.1, 2) / 3);
  const auto dir3 = Eigen::Vector3d::Ones().normalized();
  // clang-format off
  CHECK_INTERSECTS_ONCE(sphere, (   0,    0,    0), ( dir3), ( sq3,  sq3,  sq3), 1e-4)
  CHECK_INTERSECTS_ONCE(sphere, ( 0.5,  0.5,  0.5), ( dir3), ( sq3,  sq3,  sq3), 1e-4)
  CHECK_INTERSECTS_ONCE(sphere, (-0.5, -0.5, -0.5), ( dir3), ( sq3,  sq3,  sq3), 1e-4)
  CHECK_INTERSECTS_ONCE(sphere, (   0,    0,    0), (-dir3), (-sq3, -sq3, -sq3), 1e-4)
  CHECK_INTERSECTS_ONCE(sphere, ( 0.5,  0.5,  0.5), (-dir3), (-sq3, -sq3, -sq3), 1e-4)
  CHECK_INTERSECTS_ONCE(sphere, (-0.5, -0.5, -0.5), (-dir3), (-sq3, -sq3, -sq3), 1e-4)
  // clang-format on

  // 2D diagonal

  const auto sq2 = sqrt(pow(1 + 0.1, 2) / 2);
  const auto dir2 = 1/sqrt(2);
  // clang-format off
  CHECK_INTERSECTS_ONCE(sphere, (   0,    0,    0), ( dir2,  dir2,     0), ( sq2,  sq2,    0), 1e-4)
  CHECK_INTERSECTS_ONCE(sphere, (   0,    0,    0), (-dir2, -dir2,     0), (-sq2, -sq2,    0), 1e-4)
  CHECK_INTERSECTS_ONCE(sphere, ( 0.5,  0.5,    0), ( dir2,  dir2,     0), ( sq2,  sq2,    0), 1e-4)
  CHECK_INTERSECTS_ONCE(sphere, ( 0.5,  0.5,    0), (-dir2, -dir2,     0), (-sq2, -sq2,    0), 1e-4)
  CHECK_INTERSECTS_ONCE(sphere, (-0.5, -0.5,    0), ( dir2,  dir2,     0), ( sq2,  sq2,    0), 1e-4)
  CHECK_INTERSECTS_ONCE(sphere, (-0.5, -0.5,    0), (-dir2, -dir2,     0), (-sq2, -sq2,    0), 1e-4)

  CHECK_INTERSECTS_ONCE(sphere, (   0,    0,    0), (    0,  dir2,  dir2), (   0,  sq2,  sq2), 1e-4)
  CHECK_INTERSECTS_ONCE(sphere, (   0,    0,    0), (    0, -dir2, -dir2), (   0, -sq2, -sq2), 1e-4)
  CHECK_INTERSECTS_ONCE(sphere, (   0,  0.5,  0.5), (    0,  dir2,  dir2), (   0,  sq2,  sq2), 1e-4)
  CHECK_INTERSECTS_ONCE(sphere, (   0,  0.5,  0.5), (    0, -dir2, -dir2), (   0, -sq2, -sq2), 1e-4)
  CHECK_INTERSECTS_ONCE(sphere, (   0, -0.5, -0.5), (    0,  dir2,  dir2), (   0,  sq2,  sq2), 1e-4)
  CHECK_INTERSECTS_ONCE(sphere, (   0, -0.5, -0.5), (    0, -dir2, -dir2), (   0, -sq2, -sq2), 1e-4)

  CHECK_INTERSECTS_ONCE(sphere, (   0,    0,    0), ( dir2,     0,  dir2), ( sq2,    0,  sq2), 1e-4)
  CHECK_INTERSECTS_ONCE(sphere, (   0,    0,    0), (-dir2,     0, -dir2), (-sq2,    0, -sq2), 1e-4)
  CHECK_INTERSECTS_ONCE(sphere, ( 0.5,    0,  0.5), ( dir2,     0,  dir2), ( sq2,    0,  sq2), 1e-4)
  CHECK_INTERSECTS_ONCE(sphere, ( 0.5,    0,  0.5), (-dir2,     0, -dir2), (-sq2,    0, -sq2), 1e-4)
  CHECK_INTERSECTS_ONCE(sphere, (-0.5,    0, -0.5), ( dir2,     0,  dir2), ( sq2,    0,  sq2), 1e-4)
  CHECK_INTERSECTS_ONCE(sphere, (-0.5,    0, -0.5), (-dir2,     0, -dir2), (-sq2,    0, -sq2), 1e-4)
  // clang-format on
}

TEST(SphereRayIntersection, SimpleRay1)
{
  shapes::Sphere shape(1.0);
  bodies::Sphere sphere(&shape);
  sphere.setScale(1.05);

  CHECK_INTERSECTS_TWICE(sphere, (5, 0, 0), (-1, 0, 0), (1.05, 0, 0), (-1.05, 0, 0), 1e-6)
}

TEST(SphereRayIntersection, SimpleRay2)
{
  shapes::Sphere shape(1.0);
  bodies::Sphere sphere(&shape);
  sphere.setScale(1.05);

  CHECK_NO_INTERSECTION(sphere, (5, 0, 0), (1, 0, 0))
}

TEST(BoxRayIntersection, SimpleRay1)
{
  shapes::Box shape(1.0, 1.0, 3.0);
  bodies::Box box(&shape);
  box.setScale(0.95);

  CHECK_INTERSECTS_TWICE(box, (10, 0.449, 0), (-1, 0, 0), (0.475, 0.449, 0), (-0.475, 0.449, 0), 1e-4)
}

TEST(BoxRayIntersection, SimpleRay2)
{
  shapes::Box shape(0.9, 0.01, 1.2);
  bodies::Box box(&shape);

  Eigen::Isometry3d pose(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()));
  pose.translation() = Eigen::Vector3d(0, 0.005, 0.6);
  box.setPose(pose);

  const Eigen::Vector3d ray_d(0, -5.195, -0.77);

  CHECK_INTERSECTS(box, (0, 5, 1.6), (ray_d.normalized()), 2)
}

TEST(BoxRayIntersection, SimpleRay3)
{
  shapes::Box shape(0.02, 0.4, 1.2);
  bodies::Box box(&shape);

  Eigen::Isometry3d pose(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()));
  pose.translation() = Eigen::Vector3d(0.45, -0.195, 0.6);
  box.setPose(pose);

  const Eigen::Vector3d ray_d(0, 1.8, -0.669);

  CHECK_NO_INTERSECTION(box, (0, -2, 1.11), (ray_d.normalized()))
}

// This tests if https://github.com/ros-planning/geometric_shapes/pull/109 is fixed
TEST(BoxRayIntersection, FailsInUpstream)
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