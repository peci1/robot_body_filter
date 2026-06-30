#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <geometric_shapes/shapes.h>
#include <urdf_model/types.h>

namespace robot_body_filter {

/**
 * \brief Construct a masking shape out of the given URDF geometry.
 *
 * Just a helper function to convert `urdf::Geometry` to the corresponding `shapes::Shape`.
 *
 * \param[in] geometry The URDF geometry object to convert.
 * \return The constructed shape. Empty pointer for unsupported geometries.
 */
shapes::ShapeConstPtr constructShape(const urdf::Geometry& geometry);

}  // namespace robot_body_filter
