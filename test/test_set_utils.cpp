#include "gtest/gtest.h"
#include <set>
#include <robot_body_filter/utils/set_utils.hpp>

TEST(SetUtils, isSetIntersectionEmpty)
{
  std::set<int> a = { 1, 2, 4, -3 };
  std::set<int> b = { -1, -2, -4, 3 };
  std::set<int> c = { 4, 3 };
  std::set<int> e;

  EXPECT_FALSE(robot_body_filter::isSetIntersectionEmpty(a, a));
  EXPECT_FALSE(robot_body_filter::isSetIntersectionEmpty(b, b));
  EXPECT_FALSE(robot_body_filter::isSetIntersectionEmpty(a, c));
  EXPECT_FALSE(robot_body_filter::isSetIntersectionEmpty(b, c));
  EXPECT_FALSE(robot_body_filter::isSetIntersectionEmpty(c, a));
  EXPECT_FALSE(robot_body_filter::isSetIntersectionEmpty(c, b));

  EXPECT_TRUE(robot_body_filter::isSetIntersectionEmpty(a, b));
  EXPECT_TRUE(robot_body_filter::isSetIntersectionEmpty(b, a));

  // empty set handling
  EXPECT_TRUE(robot_body_filter::isSetIntersectionEmpty(a, e));
  EXPECT_TRUE(robot_body_filter::isSetIntersectionEmpty(b, e));
  EXPECT_TRUE(robot_body_filter::isSetIntersectionEmpty(e, e));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}