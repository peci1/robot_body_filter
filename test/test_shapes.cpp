#include "gtest/gtest.h"
#include <set>
#include <robot_body_filter/utils/shapes.h>
#include <urdf_model/model.h>

TEST(Shapes, Box)
{
  auto g = urdf::Box();
  g.dim = {1.0, 2.0, 3.0};

  const auto shape = robot_body_filter::constructShape(g);
  const auto box = std::dynamic_pointer_cast<const shapes::Box>(shape);
  ASSERT_TRUE(box != nullptr);
  EXPECT_EQ(shapes::BOX, box->type);
  EXPECT_EQ(1.0, box->size[0]);
  EXPECT_EQ(2.0, box->size[1]);
  EXPECT_EQ(3.0, box->size[2]);
}

TEST(Shapes, Cylinder)
{
  auto g = urdf::Cylinder();
  g.length = 1.0;
  g.radius = 2.0;

  const auto shape = robot_body_filter::constructShape(g);
  const auto cylinder = std::dynamic_pointer_cast<const shapes::Cylinder>(shape);
  ASSERT_TRUE(cylinder != nullptr);
  EXPECT_EQ(shapes::CYLINDER, cylinder->type);
  EXPECT_EQ(1.0, cylinder->length);
  EXPECT_EQ(2.0, cylinder->radius);
}

TEST(Shapes, Sphere)
{
  auto g = urdf::Sphere();
  g.radius = 1.0;

  const auto shape = robot_body_filter::constructShape(g);
  const auto sphere = std::dynamic_pointer_cast<const shapes::Sphere>(shape);
  ASSERT_TRUE(sphere != nullptr);
  EXPECT_EQ(shapes::SPHERE, sphere->type);
  EXPECT_EQ(1.0, sphere->radius);
}

TEST(Shapes, MeshEmptyFilename)
{
  auto g = urdf::Mesh();
  g.scale = {1.0, 2.0, 3.0};
  g.filename = "";

  const auto shape = robot_body_filter::constructShape(g);
  const auto mesh = std::dynamic_pointer_cast<const shapes::Mesh>(shape);
  ASSERT_TRUE(mesh == nullptr);  // empty filename
}

TEST(Shapes, Mesh)
{
  auto g = urdf::Mesh();
  g.scale = {1.0, 2.0, 3.0};
  g.filename = "package://robot_body_filter/test/triangle.dae";

  const auto shape = robot_body_filter::constructShape(g);
  const auto mesh = std::dynamic_pointer_cast<const shapes::Mesh>(shape);
  ASSERT_TRUE(mesh != nullptr);  // empty filename
  EXPECT_EQ(shapes::MESH, mesh->type);
  EXPECT_EQ(1, mesh->triangle_count);
  EXPECT_EQ(0, mesh->triangles[0]);
  EXPECT_EQ(1, mesh->triangles[1]);
  EXPECT_EQ(2, mesh->triangles[2]);
  EXPECT_EQ(3, mesh->vertex_count);

  // triangle is ( (0, 0, 2), (0, 1, 2), (1, 0, 2) ) and we apply (1, 2, 3) scaling
  EXPECT_EQ(0.0, mesh->vertices[0]);
  EXPECT_EQ(0.0, mesh->vertices[1]);
  EXPECT_EQ(6.0, mesh->vertices[2]);
  EXPECT_EQ(0.0, mesh->vertices[3]);
  EXPECT_EQ(2.0, mesh->vertices[4]);
  EXPECT_EQ(6.0, mesh->vertices[5]);
  EXPECT_EQ(1.0, mesh->vertices[6]);
  EXPECT_EQ(0.0, mesh->vertices[7]);
  EXPECT_EQ(6.0, mesh->vertices[8]);
}

TEST(Shapes, Unknown)
{
  auto g = urdf::Geometry();
  // create a nonexistent enum item
  g.type = static_cast<decltype(urdf::Geometry::type)>(100u);

  const auto shape = robot_body_filter::constructShape(g);
  ASSERT_TRUE(shape == nullptr);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}