/* HACK HACK HACK */
/* We want to use protected members of the bodies. */
// Upstream solution proposed in
// https://github.com/ros-planning/geometric_shapes/issues/89
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
      2 * Eigen::Vector3d(body->length2_, body->width2_, body->height2_));
}

void computeBoundingBox(const bodies::Sphere *body, AxisAlignedBoundingBox &bbox)
{
  bbox.setEmpty();

  if (body == nullptr)
    return;

  // it's a sphere, so we do not rotate the bounding box
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
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

  // method taken from http://www.iquilezles.org/www/articles/diskbbox/diskbbox.htm

  const auto a = body->normalH_;
  const auto e = body->radiusU_ * (Eigen::Vector3d::Ones() - a.cwiseProduct(a) / a.dot(a)).cwiseSqrt();
  const auto pa = body->center_ + body->length2_ * body->normalH_;
  const auto pb = body->center_ - body->length2_ * body->normalH_;

  bbox.extend(pa - e);
  bbox.extend(pa + e);
  bbox.extend(pb - e);
  bbox.extend(pb + e);
}

void mergeAxisAlignedBoundingBoxes(const std::vector<AxisAlignedBoundingBox> &boxes,
                                   AxisAlignedBoundingBox &mergedBox)
{
  for (const auto& box : boxes)
    mergedBox.extend(box);
}

void mergeOrientedBoundingBoxesApprox(const std::vector<OrientedBoundingBox>& boxes,
    OrientedBoundingBox& mergedBox)
{
  for (const auto& box : boxes)
    mergedBox.extendApprox(box);
}

shapes::ShapeConstPtr constructShapeFromBody(const bodies::Body* body)
{
  shapes::ShapePtr result;

  switch (body->getType()) {
    case shapes::SPHERE: {
      bodies::BoundingSphere sphere;
      dynamic_cast<const bodies::Sphere*>(body)->computeBoundingSphere(sphere);
      result.reset(new shapes::Sphere(sphere.radius));
      break;
    }
    case shapes::BOX: {
      auto box = dynamic_cast<const bodies::Box*>(body);
      result.reset(new shapes::Box(2 * box->length2_, 2 * box->width2_, 2 * box->height2_));
      break;
    }
    case shapes::CYLINDER: {
      // computation should use bounding cylinder, but there is a bug in it:
      // https://github.com/ros-planning/geometric_shapes/pull/107
      const auto* cylinder = dynamic_cast<const bodies::Cylinder*>(body);
      result.reset(new shapes::Cylinder(cylinder->radiusU_, 2 * cylinder->length2_));
      break;
    }
    case shapes::MESH: {
      auto mesh = dynamic_cast<const bodies::ConvexMesh*>(body);
      result.reset(shapes::createMeshFromVertices(mesh->getScaledVertices()));
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
    shapes::constructMarkerFromShape(shape.get(), msg, false);
    msg.pose = tf2::toMsg(body->getPose());
}

void computeBoundingBox(const bodies::Body *body,
                                OrientedBoundingBox &bbox) {
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
  default:
    throw std::runtime_error("Unsupported geometric body type.");
  }
}

void computeBoundingBox(const bodies::Sphere *body,
                                OrientedBoundingBox &bbox) {
  // it's a sphere, so we do not rotate the bounding box
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = body->getPose().translation();

  bbox.setPoseAndExtents(transform, 2 * Eigen::Vector3d(body->radiusU_, body->radiusU_, body->radiusU_));
}

void computeBoundingBox(const bodies::Box *body,
                                OrientedBoundingBox &bbox) {
  bbox.setPoseAndExtents(body->getPose(), 2 * Eigen::Vector3d(body->length2_, body->width2_, body->height2_));
}

void computeBoundingBox(const bodies::Cylinder *body,
                                OrientedBoundingBox &bbox) {
  bbox.setPoseAndExtents(body->getPose(), 2 * Eigen::Vector3d(body->radiusU_, body->radiusU_, body->length2_));
}

void computeBoundingBox(const bodies::ConvexMesh *body,
                                OrientedBoundingBox &bbox) {
  computeBoundingBox(&body->bounding_box_, bbox);
}

// for some reason, the O2 optimization screws up this function which then gives wrong results
#pragma GCC push_options
#pragma GCC optimize("O1")
bool intersectsRayBox(const bodies::Box *box, const Eigen::Vector3d &origin,
                   const Eigen::Vector3d &dir,
                   EigenSTL::vector_Vector3d *intersections,
                   unsigned int count) {
  const Eigen::Vector3d tmp(box->length2_, box->width2_, box->height2_);
  const Eigen::Vector3d corner1 = box->center_ - tmp;
  const Eigen::Vector3d corner2 = box->center_ + tmp;

  // Brian Smits. Efficient bounding box intersection. Ray tracing news 15(1), 2002
  float tmin, tmax, tymin, tymax, tzmin, tzmax;
  float divx, divy, divz;
  const Eigen::Vector3d o(box->pose_.rotation().inverse() * (origin - box->center_) + box->center_);
  const Eigen::Vector3d d(box->pose_.rotation().inverse() * dir);

  divx = 1 / d.x();
  if (divx >= 0)
  {
    tmin = (corner1.x() - o.x()) * divx;
    tmax = (corner2.x() - o.x()) * divx;
  }
  else
  {
    tmax = (corner1.x() - o.x()) * divx;
    tmin = (corner2.x() - o.x()) * divx;
  }

  divy = 1 / d.y();
  if (d.y() >= 0)
  {
    tymin = (corner1.y() - o.y()) * divy;
    tymax = (corner2.y() - o.y()) * divy;
  }
  else
  {
    tymax = (corner1.y() - o.y()) * divy;
    tymin = (corner2.y() - o.y()) * divy;
  }

  if ((tmin > tymax || tymin > tmax))
    return false;

  if (tymin > tmin)
    tmin = tymin;
  if (tymax < tmax)
    tmax = tymax;

  divz = 1 / d.z();
  if (d.z() >= 0)
  {
    tzmin = (corner1.z() - o.z()) * divz;
    tzmax = (corner2.z() - o.z()) * divz;
  }
  else
  {
    tzmax = (corner1.z() - o.z()) * divz;
    tzmin = (corner2.z() - o.z()) * divz;
  }

  if ((tmin > tzmax || tzmin > tmax))
    return false;

  if (tzmin > tmin)
    tmin = tzmin;
  if (tzmax < tmax)
    tmax = tzmax;

  if (tmax < 0)
    return false;

  if (intersections)
  {
    if (tmax - tmin > 1e-9)
    {
      intersections->push_back(tmin * dir + origin);
      if (count == 0 || count > 1)
        intersections->push_back(tmax * dir + origin);
    }
    else
      intersections->push_back(tmax * dir + origin);
  }

  return true;
}
#pragma GCC pop_options

// the upstream method for box is buggy:
// https://github.com/ros-planning/geometric_shapes/pull/108, https://github.com/ros-planning/geometric_shapes/pull/109
bool intersectsRay(const bodies::Body *body, const Eigen::Vector3d &origin,
                   const Eigen::Vector3d &dir,
                   EigenSTL::vector_Vector3d *intersections,
                   unsigned int count) {
  if (body == nullptr)
    return false;

  switch (body->getType()) {

  case shapes::SPHERE:
  case shapes::CYLINDER:
  case shapes::MESH:
    return body->intersectsRay(origin, dir, intersections, count);
  case shapes::BOX:
    return intersectsRayBox(dynamic_cast<const bodies::Box*>(body), origin, dir, intersections, count);
  default:
    throw std::runtime_error("Unsupported geometric body type.");
  }
}

}