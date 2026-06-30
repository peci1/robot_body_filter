// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include "gtest/gtest.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <eigen_stl_containers/eigen_stl_vector_container.h>
#include <rclcpp/utilities.hpp>

void expectVector3dSetsEqual(
  EigenSTL::vector_Vector3d vec1, EigenSTL::vector_Vector3d vec2, const double up_to_error = 1e-6) {
  ASSERT_EQ(vec1.size(), vec2.size());

  auto vec_compare = [up_to_error](const Eigen::Vector3d& a, const Eigen::Vector3d& b) -> bool {
    if (a.x() < b.x() - up_to_error) {
      return true;
    }
    if (a.x() > b.x() + up_to_error) {
      return false;
    }

    if (a.y() < b.y() - up_to_error) {
      return true;
    }
    if (a.y() > b.y() + up_to_error) {
      return false;
    }

    if (a.z() < b.z() - up_to_error) {
      return true;
    }
    if (a.z() > b.z() + up_to_error) {
      return false;
    }

    return false;
  };

  std::sort(vec1.begin(), vec1.end(), vec_compare);
  std::sort(vec2.begin(), vec2.end(), vec_compare);

  for (size_t i = 0; i < vec1.size(); ++i) {
    EXPECT_NEAR(vec1[i].x(), vec2[i].x(), up_to_error);
    EXPECT_NEAR(vec1[i].y(), vec2[i].y(), up_to_error);
    EXPECT_NEAR(vec1[i].z(), vec2[i].z(), up_to_error);
  }
}

Eigen::Isometry3d randomPose() {
  return Eigen::Quaterniond::UnitRandom() * Eigen::Translation3d(Eigen::Vector3d::Random());
}

void expectTransformsDoubleEq(const Eigen::Isometry3d& i1, const Eigen::Isometry3d& i2) {
  const auto t1 = i1.translation();
  const auto t2 = i2.translation();
  EXPECT_DOUBLE_EQ(t1.x(), t2.x());
  EXPECT_DOUBLE_EQ(t1.y(), t2.y());
  EXPECT_DOUBLE_EQ(t1.z(), t2.z());
  const auto q1 = Eigen::Quaterniond(i1.rotation());
  const auto q2 = Eigen::Quaterniond(i2.rotation());
  EXPECT_DOUBLE_EQ(q1.x(), q2.x());
  EXPECT_DOUBLE_EQ(q1.y(), q2.y());
  EXPECT_DOUBLE_EQ(q1.z(), q2.z());
  EXPECT_DOUBLE_EQ(q1.w(), q2.w());
}

#define WAIT_FOR_MESSAGE(node_ptr, msg) \
  { \
    size_t i = 0; \
    while (rclcpp::ok() && msg == nullptr && i < 100) { \
      rclcpp::spin_some(node_ptr); \
      rclcpp::sleep_for(std::chrono::milliseconds(10)); \
      ++i; \
    } \
    if (i == 100) { \
      GTEST_FAIL(); \
    } \
  }

#define EXPECT_NAN(num) EXPECT_TRUE(std::isnan(num))
