// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <geometric_shapes/mesh_operations.h>
#include <rclcpp/rclcpp.hpp>
#include <robot_body_filter/utils/bodies.hpp>
#include <robot_body_filter/utils/shapes.hpp>
#include <urdf_model/link.h>

shapes::ShapeConstPtr robot_body_filter::constructShape(const urdf::Geometry& geometry) {
  shapes::ShapePtr result;

  switch (geometry.type) {
    case urdf::Geometry::SPHERE:
    {
      result.reset(new shapes::Sphere(static_cast<const urdf::Sphere*>(&geometry)->radius));
      break;
    }
    case urdf::Geometry::BOX:
    {
      const urdf::Vector3 dim = static_cast<const urdf::Box*>(&geometry)->dim;
      result.reset(new shapes::Box(dim.x, dim.y, dim.z));
      break;
    }
    case urdf::Geometry::CYLINDER:
    {
      result.reset(new shapes::Cylinder(
        static_cast<const urdf::Cylinder*>(&geometry)->radius,
        static_cast<const urdf::Cylinder*>(&geometry)->length));
      break;
    }
    case urdf::Geometry::MESH:
    {
      const auto* mesh = static_cast<const urdf::Mesh*>(&geometry);
      if (!mesh->filename.empty()) {
        const Eigen::Vector3d scale(mesh->scale.x, mesh->scale.y, mesh->scale.z);
        result.reset(shapes::createMeshFromResource(mesh->filename, scale));
      } else {
        RCLCPP_WARN(rclcpp::get_logger("shapes"), "Empty mesh filename");
      }
      break;
    }
    default:
    {
      RCLCPP_ERROR(rclcpp::get_logger("shapes"), "Unknown geometry type: %d", static_cast<int>(geometry.type));
      break;
    }
  }
  return result;
}
