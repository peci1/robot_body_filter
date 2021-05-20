/* HACK HACK HACK */
/* We want to use protected members of the bodies. */
// Once https://github.com/ros-planning/geometric_shapes/pull/138 gets backported to melodic,
// we can switch to using getScaledDimensions() and get rid of this hack.
#define protected public
#include <geometric_shapes/bodies.h>
#undef protected
/* HACK END HACK */

#include <robot_body_filter/utils/bodies.h>

#include <ros/ros.h>
#include <tf2_eigen/tf2_eigen.h>

#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>

namespace bodies
{

void mergeOrientedBoundingBoxesApprox(const std::vector<OrientedBoundingBox>& boxes,
    OrientedBoundingBox& mergedBox)
{
  for (const auto& box : boxes)
    mergedBox.extendApprox(box);
}

shapes::ShapeConstPtr constructShapeFromBody(const bodies::Body* body)
{
  shapes::ShapePtr result;

  if (body == nullptr)
    return result;

  switch (body->getType()) {
    case shapes::SPHERE: {
      bodies::BoundingSphere sphere;
      static_cast<const bodies::Sphere*>(body)->computeBoundingSphere(sphere);
      result.reset(new shapes::Sphere(sphere.radius));
      break;
    }
    case shapes::BOX: {
      bodies::OrientedBoundingBox bbox;
      bodies::computeBoundingBox(static_cast<const bodies::Box*>(body), bbox);
      const Eigen::Vector3d extents = bbox.getExtents();
      result.reset(new shapes::Box(extents.x(), extents.y(), extents.z()));
      break;
    }
    case shapes::CYLINDER: {
      bodies::BoundingCylinder bcyl;
      static_cast<const bodies::Cylinder*>(body)->computeBoundingCylinder(bcyl);
      result.reset(new shapes::Cylinder(bcyl.radius, bcyl.length));
      break;
    }
    case shapes::MESH: {
      const auto mesh = static_cast<const bodies::ConvexMesh*>(body);
      const auto& scaledVertices = mesh->getScaledVertices();

      // createMeshFromVertices requires an "expanded" list of triangles where each triangle is
      // represented by its three vertex positions
      EigenSTL::vector_Vector3d vertexList;
      vertexList.reserve(3 * mesh->getTriangles().size());
      for (const auto& triangle : mesh->getTriangles())
        vertexList.push_back(scaledVertices[triangle]);

      result.reset(shapes::createMeshFromVertices(vertexList));
      break;
    }
    default: {
      ROS_ERROR("Unknown body type: %d", (int) body->getType());
      break;
    }
  }
  return result;
}

void constructMarkerFromBody(const bodies::Body* body,
                             visualization_msgs::Marker& msg)
{
    auto shape = bodies::constructShapeFromBody(body);
    shapes::constructMarkerFromShape(shape.get(), msg, true);
    msg.pose = tf2::toMsg(body->getPose());
}

void computeBoundingBox(const bodies::Body *body,
                                OrientedBoundingBox &bbox) {
  if (body == nullptr)
  {
    bbox = OBB();
    return;
  }

  switch (body->getType()) {

  case shapes::SPHERE:
    computeBoundingBox(static_cast<const bodies::Sphere*>(body), bbox);
    break;
  case shapes::CYLINDER:
    computeBoundingBox(static_cast<const bodies::Cylinder*>(body), bbox);
    break;
  case shapes::BOX:
    computeBoundingBox(static_cast<const bodies::Box*>(body), bbox);
    break;
  case shapes::MESH:
    computeBoundingBox(static_cast<const bodies::ConvexMesh*>(body), bbox);
    break;
  case shapes::PLANE:
  case shapes::CONE:
  case shapes::UNKNOWN_SHAPE:
  case shapes::OCTREE:
  default:
    throw std::runtime_error("Unsupported geometric body type.");
  }
}

void computeBoundingBox(const bodies::Sphere *body,
                                OrientedBoundingBox &bbox) {
  if (body == nullptr)
  {
    bbox = OBB();
    return;
  }

  // it's a sphere, so we do not rotate the bounding box
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = body->getPose().translation();

  bodies::BoundingSphere bsphere;
  body->computeBoundingSphere(bsphere);
  bbox.setPoseAndExtents(transform, 2 * Eigen::Vector3d(bsphere.radius, bsphere.radius, bsphere.radius));
}

void computeBoundingBox(const bodies::Box *body,
                                OrientedBoundingBox &bbox) {
  if (body == nullptr)
  {
    bbox = OBB();
    return;
  }

  bbox.setPoseAndExtents(body->getPose(), 2 * Eigen::Vector3d(body->length2_, body->width2_, body->height2_));
}

void computeBoundingBox(const bodies::Cylinder *body,
                                OrientedBoundingBox &bbox) {
  if (body == nullptr)
  {
    bbox = OBB();
    return;
  }

  bodies::BoundingCylinder bcyl;
  body->computeBoundingCylinder(bcyl);
  bbox.setPoseAndExtents(body->getPose(), Eigen::Vector3d(2 * bcyl.radius, 2 * bcyl.radius, bcyl.length));
}

void computeBoundingBox(const bodies::ConvexMesh *body,
                                OrientedBoundingBox &bbox) {
  if (body == nullptr)
  {
    bbox = OBB();
    return;
  }

  computeBoundingBox(&body->bounding_box_, bbox);
}

}
