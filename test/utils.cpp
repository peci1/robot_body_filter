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