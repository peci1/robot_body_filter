// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <robot_body_filter/utils/tf2_eigen.hpp>

void tf2::toMsg(const Eigen::Vector3d& in, geometry_msgs::msg::Point32& out) {
  out.x = in.x();
  out.y = in.y();
  out.z = in.z();
}
