#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <geometric_shapes/aabb.h>
#include <geometric_shapes/bodies.h>
#include <geometric_shapes/obb.h>

namespace bodies {

using AxisAlignedBoundingBox = AABB;
using OrientedBoundingBox = OBB;

/**
 * \brief Compute AABB for the body at different pose. Can't use setPose() because we want `body` to be const.
 *
 * \param[in] body The body whose bounding box is to be computed.
 * \param[out] bbox The computed bounding box.
 * \param[in] pose The pose at which the body should be moved for the computation.
 *
 * \throw std::runtime_error For unsupported body types (supported are SPHERE, CYLINDER, BOX, MESH).
 */
void computeBoundingBoxAt(const Body* body, AxisAlignedBoundingBox& bbox, const Eigen::Isometry3d& pose);

}  // namespace bodies
