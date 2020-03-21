#include "gtest/gtest.h"
#include <robot_body_filter/utils/tf2_eigen.h>

TEST(TF2Eigen, ToMsg)
{
  const auto ePoint = Eigen::Vector3d(1.0, 2.0, 3.0);
  geometry_msgs::Point32 gPoint;
  tf2::toMsg(ePoint, gPoint);

  EXPECT_EQ(float(ePoint.x()), gPoint.x);
  EXPECT_EQ(float(ePoint.y()), gPoint.y);
  EXPECT_EQ(float(ePoint.z()), gPoint.z);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}