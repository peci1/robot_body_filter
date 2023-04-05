#ifndef ROBOT_BODY_FILTER_BODIES_H
#define ROBOT_BODY_FILTER_BODIES_H

#include <geometric_shapes/bodies.h>
#include <geometric_shapes/aabb.h>

#if __has_include(<geometric_shapes/obb.h>)
#include <geometric_shapes/obb.h>
#else
#error <geometric_shapes/obb.h> not found. Please, update geometric_shapes library to version 0.6.5+ or 0.7.4+ 
#endif

#include <visualization_msgs/Marker.h>

namespace bodies
{

typedef bodies::AABB AxisAlignedBoundingBox;
typedef bodies::OBB OrientedBoundingBox;

/** \brief Compute AABB for the body at different pose. Can't use setPose() because we want `body` to be const. */
void computeBoundingBoxAt(const bodies::Body* body, AxisAlignedBoundingBox& bbox, const Eigen::Isometry3d& pose);

}

#endif //ROBOT_BODY_FILTER_BODIES_H
