// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include "gtest/gtest.h"

#include <Eigen/Core>

#include <geometry_msgs/msg/point32.hpp>
#include <robot_body_filter/utils/tf2_eigen.hpp>

TEST(TF2Eigen, ToMsg) {
  const auto e_point = Eigen::Vector3d(1.0, 2.0, 3.0);
  geometry_msgs::msg::Point32 g_point;
  tf2::toMsg(e_point, g_point);

  EXPECT_EQ(static_cast<float>(e_point.x()), g_point.x);
  EXPECT_EQ(static_cast<float>(e_point.y()), g_point.y);
  EXPECT_EQ(static_cast<float>(e_point.z()), g_point.z);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
