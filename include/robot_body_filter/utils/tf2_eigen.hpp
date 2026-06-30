#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <Eigen/Geometry>

#include <geometry_msgs/msg/point32.hpp>

namespace tf2 {

/**
 * \brief Convert the given Eigen 3D vector to `Point32` message.
 *
 * \param[in] in The input 3D vector.
 * \param[in] out The corresponding `Point32` message.
 */
void toMsg(const Eigen::Vector3d& in, geometry_msgs::msg::Point32& out);

}  // namespace tf2
