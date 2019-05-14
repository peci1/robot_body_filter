/* HACK HACK HACK */
/* We want to use protected members of the bodies. */
// Upstream solution proposed in
// https://github.com/ros-planning/geometric_shapes/issues/89
#define protected public
#include <geometric_shapes/bodies.h>
#undef protected
/* HACK END HACK */

#include <robot_self_filter/utils/bodies.h>

#include <ros/ros.h>

#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>

namespace bodies
{

void computeBoundingBox(const bodies::Body *body, AxisAlignedBoundingBox &bbox)
{
  bbox.setEmpty();

  if (body == nullptr)
    return;

  switch (body->getType()) {

    case shapes::SPHERE:
      computeBoundingBox(dynamic_cast<const bodies::Sphere*>(body), bbox);
      break;
    case shapes::CYLINDER:
      computeBoundingBox(dynamic_cast<const bodies::Cylinder*>(body), bbox);
      break;
    case shapes::BOX:
      computeBoundingBox(dynamic_cast<const bodies::Box*>(body), bbox);
      break;
    case shapes::MESH:
      computeBoundingBox(dynamic_cast<const bodies::ConvexMesh*>(body), bbox);
      break;
    case shapes::PLANE:
    case shapes::CONE:
    case shapes::UNKNOWN_SHAPE:
    case shapes::OCTREE:
      throw std::runtime_error("Unsupported geometric body type.");
  }
}

void computeBoundingBox(const bodies::Box *body, AxisAlignedBoundingBox &bbox)
{
  bbox.setEmpty();

  if (body == nullptr)
    return;

  bbox.extendWithTransformedBox(body->getPose(),
      Eigen::Vector3d(body->length2_, body->width2_, body->height2_));
}

void computeBoundingBox(const bodies::Sphere *body, AxisAlignedBoundingBox &bbox)
{
  bbox.setEmpty();

  if (body == nullptr)
    return;

  // it's a sphere, so we do not rotate the bounding box
  Eigen::Isometry3d transform;
  transform.translation() = body->getPose().translation();

  bbox.extendWithTransformedBox(transform,
      Eigen::Vector3d(2 * body->radiusU_, 2 * body->radiusU_, 2 * body->radiusU_));
}

void computeBoundingBox(const bodies::ConvexMesh *body, AxisAlignedBoundingBox &bbox)
{
  bbox.setEmpty();

  if (body == nullptr)
    return;

  computeBoundingBox(&body->bounding_box_, bbox);
}

void computeBoundingBox(const bodies::Cylinder *body, AxisAlignedBoundingBox &bbox)
{
  bbox.setEmpty();

  if (body == nullptr)
    return;

  // TODO this might be a little suboptimal if the cylinder is rotated around
  //  its z-axis - the resulting bbox might be up to sqrt(2)-times larger
  bbox.extendWithTransformedBox(body->getPose(),
      Eigen::Vector3d(2 * body->radiusU_, 2 * body->radiusU_, 2 * body->length2_));
}

void mergeAxisAlignedBoundingBoxes(const std::vector<AxisAlignedBoundingBox> &boxes,
                                   AxisAlignedBoundingBox &mergedBox)
{
  for (const auto& box : boxes)
    mergedBox.extend(box);
}

shapes::ShapeConstPtr constructShapeFromBody(const bodies::Body &body)
{
  shapes::ShapePtr result;

  switch (body.getType()) {
    case shapes::SPHERE: {
      bodies::BoundingSphere sphere;
      dynamic_cast<const bodies::Sphere*>(&body)->computeBoundingSphere(sphere);
      result.reset(new shapes::Sphere(sphere.radius));
      break;
    }
    case shapes::BOX: {
      auto box = dynamic_cast<const bodies::Box*>(&body);
      result.reset(new shapes::Box(2 * box->length2_, 2 * box->width2_, 2 * box->height2_));
      break;
    }
    case shapes::CYLINDER: {
      bodies::BoundingCylinder cylinder;
      dynamic_cast<const bodies::Cylinder*>(&body)->computeBoundingCylinder(cylinder);
      result.reset(new shapes::Cylinder(cylinder.radius, cylinder.length));
      break;
    }
    case shapes::MESH: {
      auto mesh = dynamic_cast<const bodies::ConvexMesh*>(&body);
      result.reset(shapes::createMeshFromVertices(mesh->getScaledVertices()));
      break;
    }
    default: {
      ROS_ERROR("Unknown body type: %d", (int) body.getType());
      break;
    }
  }
  return result;
}

void constructMarkerFromBody(const bodies::Body& body,
                             visualization_msgs::Marker& msg)
{
    auto shape = bodies::constructShapeFromBody(body);
    shapes::constructMarkerFromShape(shape.get(), msg, false);

    const auto& pose = body.getPose();
    msg.pose.position.x = pose.translation().x();
    msg.pose.position.y = pose.translation().y();
    msg.pose.position.z = pose.translation().z();
    const Eigen::Quaterniond q(pose.linear());
    msg.pose.orientation.x = q.x();
    msg.pose.orientation.y = q.y();
    msg.pose.orientation.z = q.z();
    msg.pose.orientation.w = q.w();
}

}