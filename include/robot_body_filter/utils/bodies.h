#ifndef ROBOT_BODY_FILTER_BODIES_H
#define ROBOT_BODY_FILTER_BODIES_H

#include <geometric_shapes/bodies.h>
#include <moveit/robot_model/aabb.h>

#include <robot_body_filter/utils/obb.h>
#include <visualization_msgs/Marker.h>

namespace bodies
{

typedef moveit::core::AABB AxisAlignedBoundingBox;
typedef bodies::OBB OrientedBoundingBox;

void computeBoundingBox(const bodies::Body* body, AxisAlignedBoundingBox& bbox);
void computeBoundingBox(const bodies::Sphere* body, AxisAlignedBoundingBox& bbox);
void computeBoundingBox(const bodies::Box* body, AxisAlignedBoundingBox& bbox);
void computeBoundingBox(const bodies::Cylinder* body, AxisAlignedBoundingBox& bbox);
void computeBoundingBox(const bodies::ConvexMesh* body, AxisAlignedBoundingBox& bbox);

/** \brief Compute AABB for the body at different pose. Can't use setPose() because we want `body` to be const. */
void computeBoundingBoxAt(const bodies::Body* body, AxisAlignedBoundingBox& bbox, const Eigen::Isometry3d& pose);

void computeBoundingBox(const bodies::Body* body, OrientedBoundingBox& bbox);
void computeBoundingBox(const bodies::Sphere* body, OrientedBoundingBox& bbox);
void computeBoundingBox(const bodies::Box* body, OrientedBoundingBox& bbox);
void computeBoundingBox(const bodies::Cylinder* body, OrientedBoundingBox& bbox);
void computeBoundingBox(const bodies::ConvexMesh* body, OrientedBoundingBox& bbox);

/** \brief Compute a bounding box to enclose a set of bounding boxes. */
void mergeAxisAlignedBoundingBoxes(
    const std::vector<AxisAlignedBoundingBox>& boxes,
    AxisAlignedBoundingBox& mergedBox);

/** \brief Compute an approximate oriented bounding box to enclose a set of bounding boxes. */
void mergeOrientedBoundingBoxesApprox(
    const std::vector<OrientedBoundingBox>& boxes,
    OrientedBoundingBox& mergedBox);

shapes::ShapeConstPtr constructShapeFromBody(const bodies::Body* body);

void constructMarkerFromBody(const bodies::Body* body,
                             visualization_msgs::Marker& msg);

bool intersectsRay(const bodies::Body* body, const Eigen::Vector3d& origin, const Eigen::Vector3d& dir,
                   EigenSTL::vector_Vector3d* intersections = nullptr, unsigned int count = 0);

}

#endif //ROBOT_BODY_FILTER_BODIES_H
