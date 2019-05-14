#include <robot_self_filter/utils/shapes.h>

#include <ros/ros.h>
#include <geometric_shapes/mesh_operations.h>
#include <urdf_model/model.h>
#include <robot_self_filter/utils/bodies.h>

shapes::ShapeConstPtr robot_self_filter::constructShape(const urdf::Geometry& geometry) {
  shapes::ShapePtr result;

  switch (geometry.type) {
    case urdf::Geometry::SPHERE: {
      result.reset(new shapes::Sphere(dynamic_cast<const urdf::Sphere*>(&geometry)->radius));
      break;
    }
    case urdf::Geometry::BOX: {
      const urdf::Vector3 dim = dynamic_cast<const urdf::Box*>(&geometry)->dim;
      result.reset(new shapes::Box(dim.x, dim.y, dim.z));
      break;
    }
    case urdf::Geometry::CYLINDER: {
      result.reset(new shapes::Cylinder(
          dynamic_cast<const urdf::Cylinder*>(&geometry)->radius,
          dynamic_cast<const urdf::Cylinder*>(&geometry)->length));
      break;
    }
    case urdf::Geometry::MESH: {
      const auto* mesh = dynamic_cast<const urdf::Mesh*>(&geometry);
      if (!mesh->filename.empty()) {
        result.reset(shapes::createMeshFromResource(mesh->filename));

        // TODO watch https://github.com/ros-planning/geometric_shapes/issues/29 if it is solved
        if (mesh->scale.x != mesh->scale.y || mesh->scale.y != mesh->scale.z || mesh->scale.x != mesh->scale.z) {
          ROS_WARN_STREAM("Nonuniform mesh scaling not supported in meshes for laser filtering." <<
                          " Using X scale as the general scale. The problematic mesh: " << mesh->filename);
        }

        result->scale(mesh->scale.x);
      } else
        ROS_WARN("Empty mesh filename");
      break;
    }
    default: {
      ROS_ERROR("Unknown geometry type: %d", (int) geometry.type);
      break;
    }
  }
  return result;
}