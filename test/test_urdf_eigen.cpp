#include "gtest/gtest.h"
#include <robot_body_filter/utils/urdf_eigen.hpp>
#include <urdf_model/model.h>

TEST(UrdfEigen, PoseTransform)
{
  urdf::Pose uPose;
  uPose.position = {1.0, 2.0, 3.0};
  uPose.rotation.setFromRPY(0, M_PI_2, M_PI);

  const auto ePose = robot_body_filter::urdfPose2EigenTransform(uPose);

  EXPECT_DOUBLE_EQ(1.0, ePose.translation().x());
  EXPECT_DOUBLE_EQ(2.0, ePose.translation().y());
  EXPECT_DOUBLE_EQ(3.0, ePose.translation().z());

  EXPECT_DOUBLE_EQ(0.0, fabs(ePose.rotation().eulerAngles(2, 1, 0).x()));
  EXPECT_DOUBLE_EQ(M_PI_2, fabs(ePose.rotation().eulerAngles(2, 1, 0).y()));
  EXPECT_DOUBLE_EQ(M_PI, fabs(ePose.rotation().eulerAngles(2, 1, 0).z()));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}