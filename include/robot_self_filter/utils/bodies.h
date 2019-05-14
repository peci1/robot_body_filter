#ifndef ROBOT_SELF_FILTER_BODIES_H
#define ROBOT_SELF_FILTER_BODIES_H

#include <geometric_shapes/bodies.h>
#include <moveit/robot_model/aabb.h>

#include <visualization_msgs/Marker.h>

namespace bodies
{

typedef moveit::core::AABB AxisAlignedBoundingBox;

void computeBoundingBox(const bodies::Body* body, AxisAlignedBoundingBox& bbox);
void computeBoundingBox(const bodies::Sphere* body, AxisAlignedBoundingBox& bbox);
void computeBoundingBox(const bodies::Box* body, AxisAlignedBoundingBox& bbox);
void computeBoundingBox(const bodies::Cylinder* body, AxisAlignedBoundingBox& bbox);
void computeBoundingBox(const bodies::ConvexMesh* body, AxisAlignedBoundingBox& bbox);

/** \brief Compute a bounding box to enclose a set of bounding boxes. */
void mergeAxisAlignedBoundingBoxes(
    const std::vector<AxisAlignedBoundingBox>& boxes,
    AxisAlignedBoundingBox& mergedBox);

shapes::ShapeConstPtr constructShapeFromBody(const bodies::Body &body);

void constructMarkerFromBody(const bodies::Body& body,
                             visualization_msgs::Marker& msg);

}

#endif //ROBOT_SELF_FILTER_BODIES_H
