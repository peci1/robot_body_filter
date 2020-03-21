#include "gtest/gtest.h"
#include <robot_body_filter/utils/string_utils.hpp>

using namespace robot_body_filter;

TEST(StringUtils, StripLeadingSlashInplace)
{
  std::string s("test");
  stripLeadingSlash(s);
  EXPECT_EQ("test", s);

  s = "/test";
  stripLeadingSlash(s);
  EXPECT_EQ("test", s);

  s = "";
  stripLeadingSlash(s);
  EXPECT_EQ("", s);

  s = "/";
  stripLeadingSlash(s);
  EXPECT_EQ("", s);

  s = "//";
  stripLeadingSlash(s);
  EXPECT_EQ("/", s);

  s = "/test/test/";
  stripLeadingSlash(s);
  EXPECT_EQ("test/test/", s);
}

TEST(StringUtils, StripLeadingSlash)
{
  EXPECT_EQ("test", stripLeadingSlash("test"));
  EXPECT_EQ("test", stripLeadingSlash("/test"));
  EXPECT_EQ("", stripLeadingSlash(""));
  EXPECT_EQ("", stripLeadingSlash("/"));
  EXPECT_EQ("/", stripLeadingSlash("//"));
  EXPECT_EQ("test/test/", stripLeadingSlash("/test/test/"));
}

TEST(StringUtils, PrependIfNonempty)
{
  EXPECT_EQ("/test", prependIfNonEmpty("test", "/"));
  EXPECT_EQ("", prependIfNonEmpty("", "/"));
  EXPECT_EQ("//", prependIfNonEmpty("/", "/"));
  EXPECT_EQ("test/", prependIfNonEmpty("/", "test"));
  EXPECT_EQ("prefixtest", prependIfNonEmpty("test", "prefix"));
}

TEST(StringUtils, AppendIfNonempty)
{
  EXPECT_EQ("test/", appendIfNonEmpty("test", "/"));
  EXPECT_EQ("", appendIfNonEmpty("", "/"));
  EXPECT_EQ("//", appendIfNonEmpty("/", "/"));
  EXPECT_EQ("/test", appendIfNonEmpty("/", "test"));
  EXPECT_EQ("testpostfix", appendIfNonEmpty("test", "postfix"));
}

TEST(StringUtils, ToString)
{
  EXPECT_EQ("5", to_string(5));
  EXPECT_EQ("-5", to_string(-5));
  EXPECT_EQ("5.000000", to_string(5.0));
  EXPECT_EQ("-5.000000", to_string(-5.0));
  EXPECT_EQ("False", to_string(false));
  EXPECT_EQ("True", to_string(true));
  EXPECT_EQ("test", to_string(std::string("test")));
  EXPECT_EQ("[1, 2, 3]", to_string(std::vector<int>({1, 2, 3})));
  EXPECT_EQ("[True, False]", to_string(std::vector<bool>({true, false})));
  EXPECT_EQ("[]", to_string(std::vector<bool>()));
  EXPECT_EQ("[1, 2, 3]", to_string(std::set<int>({1, 2, 3}))); // set is ordered
  EXPECT_EQ("[False, True]", to_string(std::set<bool>({true, false}))); // set is ordered
  EXPECT_EQ("[]", to_string(std::set<bool>())); // set is ordered
  EXPECT_EQ("{\"a\": \"1\", \"b\": \"2\"}", to_string(std::map<std::string, int>({{"a", 1}, {"b", 2}}))); // map is ordered
  EXPECT_EQ("{\"True\": \"False\"}", to_string(std::map<bool, bool>({{true, false}})));
  EXPECT_EQ("{}", to_string(std::map<std::string, int>()));
  EXPECT_EQ("1.500000000", to_string(ros::Time(1, 500000000)));
  EXPECT_EQ("1.500000000", to_string(ros::WallTime(1, 500000000)));
  EXPECT_EQ("1.500000000", to_string(ros::SteadyTime(1, 500000000)));
  EXPECT_EQ("1.500000000", to_string(ros::Duration(1, 500000000)));
  EXPECT_EQ("1.500000000", to_string(ros::WallDuration(1, 500000000)));

  XmlRpc::XmlRpcValue v;
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}