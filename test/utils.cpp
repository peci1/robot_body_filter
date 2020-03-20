#include "gtest/gtest.h"
#include <eigen_stl_containers/eigen_stl_vector_container.h>

void expectVector3dSetsEqual(EigenSTL::vector_Vector3d vec1, EigenSTL::vector_Vector3d vec2,
                             const double upToError = 1e-6)
{
  ASSERT_EQ(vec1.size(), vec2.size());

  auto vecCompare = [upToError] (const Eigen::Vector3d& a, const Eigen::Vector3d& b) -> bool
  {
    if (a.x() < b.x() - upToError)
      return true;
    if (a.x() > b.x() + upToError)
      return false;

    if (a.y() < b.y() - upToError)
      return true;
    if (a.y() > b.y() + upToError)
      return false;

    if (a.z() < b.z() - upToError)
      return true;
    if (a.z() > b.z() + upToError)
      return false;

    return false;
  };

  std::sort(vec1.begin(), vec1.end(), vecCompare);
  std::sort(vec2.begin(), vec2.end(), vecCompare);

  for (size_t i = 0; i < vec1.size(); ++i)
  {
    EXPECT_NEAR(vec1[i].x(), vec2[i].x(), upToError);
    EXPECT_NEAR(vec1[i].y(), vec2[i].y(), upToError);
    EXPECT_NEAR(vec1[i].z(), vec2[i].z(), upToError);
  }
}

Eigen::Isometry3d randomPose()
{
  return Eigen::Quaterniond::UnitRandom() * Eigen::Translation3d(Eigen::Vector3d::Random());
}

void expectTransformsDoubleEq(const Eigen::Isometry3d& i1, const Eigen::Isometry3d& i2)
{
  auto t1 = i1.translation();
  auto t2 = i2.translation();
  EXPECT_DOUBLE_EQ(t1.x(), t2.x());
  EXPECT_DOUBLE_EQ(t1.y(), t2.y());
  EXPECT_DOUBLE_EQ(t1.z(), t2.z());
  auto q1 = Eigen::Quaterniond(i1.rotation());
  auto q2 = Eigen::Quaterniond(i2.rotation());
  EXPECT_DOUBLE_EQ(q1.x(), q2.x());
  EXPECT_DOUBLE_EQ(q1.y(), q2.y());
  EXPECT_DOUBLE_EQ(q1.z(), q2.z());
  EXPECT_DOUBLE_EQ(q1.w(), q2.w());
}

#define WAIT_FOR_MESSAGE(msg) \
    { size_t i = 0;\
      while (ros::ok() && msg == nullptr && i < 100) \
      { \
        ros::spinOnce(); \
        ros::WallDuration(0.01).sleep(); \
        ++i;\
      } \
      if (i == 100) GTEST_FAIL(); \
    }

#define EXPECT_NAN(num) EXPECT_TRUE(std::isnan(num))