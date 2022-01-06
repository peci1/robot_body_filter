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

  s = "/";
  stripLeadingSlash(s, true);
  EXPECT_EQ("", s);
}

TEST(StringUtils, StripLeadingSlash)
{
  EXPECT_EQ("test", stripLeadingSlash("test"));
  EXPECT_EQ("test", stripLeadingSlash("/test"));
  EXPECT_EQ("", stripLeadingSlash(""));
  EXPECT_EQ("", stripLeadingSlash("/"));
  EXPECT_EQ("/", stripLeadingSlash("//"));
  EXPECT_EQ("test/test/", stripLeadingSlash("/test/test/"));
  EXPECT_EQ("", stripLeadingSlash("/", true));
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
  EXPECT_EQ("[\"a\", \"b\"]", to_string(std::vector<std::string>({"a", "b"})));
  EXPECT_EQ("[1, 2, 3]", to_string(std::set<int>({1, 2, 3}))); // set is ordered
  EXPECT_EQ("[False, True]", to_string(std::set<bool>({true, false}))); // set is ordered
  EXPECT_EQ("[]", to_string(std::set<bool>())); // set is ordered
  EXPECT_EQ("{\"a\": 1, \"b\": 2}", to_string(std::map<std::string, int>({{"a", 1}, {"b", 2}}))); // map is ordered
  EXPECT_EQ("{\"a\": \"1\", \"b\": \"2\"}", to_string(std::map<std::string, std::string>({{"a", "1"}, {"b", "2"}}))); // map is ordered
  EXPECT_EQ("{True: False}", to_string(std::map<bool, bool>({{true, false}})));
  EXPECT_EQ("{}", to_string(std::map<std::string, int>()));
  EXPECT_EQ("1.500000000", to_string(ros::Time(1, 500000000)));
  EXPECT_EQ("1.500000000", to_string(ros::WallTime(1, 500000000)));
  EXPECT_EQ("1.500000000", to_string(ros::SteadyTime(1, 500000000)));
  EXPECT_EQ("1.500000000", to_string(ros::Duration(1, 500000000)));
  EXPECT_EQ("1.500000000", to_string(ros::WallDuration(1, 500000000)));
  XmlRpc::XmlRpcValue::setDoubleFormat("%.2f");
  EXPECT_EQ("2.00", to_string(XmlRpc::XmlRpcValue(2.0)));
  EXPECT_EQ("2", to_string(XmlRpc::XmlRpcValue(2)));
  EXPECT_EQ("0", to_string(XmlRpc::XmlRpcValue(false)));
  EXPECT_EQ("1", to_string(XmlRpc::XmlRpcValue(true)));
  EXPECT_EQ("aa", to_string(XmlRpc::XmlRpcValue("aa")));
  {
    tm time;
    time.tm_hour = 1;
    time.tm_min = 2;
    time.tm_sec = 3;
    time.tm_mday = 4;
    time.tm_mon = 5;
    time.tm_year = 2006;
    EXPECT_EQ("20060504T01:02:03", to_string(XmlRpc::XmlRpcValue(&time)));
  }
  {
    int offset = 0;
    EXPECT_EQ("2.00", to_string(XmlRpc::XmlRpcValue("<value><double>2.0</double></value>", &offset)));
  }
  {
    char bytes[] = "123";
    EXPECT_EQ("MTIz\n", to_string(XmlRpc::XmlRpcValue(bytes, 3)));
  }
  {
    XmlRpc::XmlRpcValue v;
    v[0] = 1;
    v[1] = 2;
    v[2] = 3;
    EXPECT_EQ("{1,2,3}", to_string(v));
  }
  {
    XmlRpc::XmlRpcValue v;
    v["0"] = 1;
    v["1"] = 2;
    v["2"] = 3;
    EXPECT_EQ("[0:1,1:2,2:3]", to_string(v));
  }
}

TEST(StringUtils, StartsWith)
{
  EXPECT_TRUE(startsWith("", ""));
  EXPECT_FALSE(startsWith("", "prefix"));
  EXPECT_FALSE(startsWith("p", "prefix"));
  EXPECT_FALSE(startsWith("pr", "prefix"));
  EXPECT_FALSE(startsWith("pre", "prefix"));
  EXPECT_FALSE(startsWith("pref", "prefix"));
  EXPECT_FALSE(startsWith("prefi", "prefix"));
  EXPECT_TRUE(startsWith("prefix", "prefix"));
  EXPECT_TRUE(startsWith("prefixS", "prefix"));
  EXPECT_TRUE(startsWith("prefixSt", "prefix"));
  EXPECT_TRUE(startsWith("prefixStr", "prefix"));
  EXPECT_TRUE(startsWith("prefixStri", "prefix"));
  EXPECT_TRUE(startsWith("prefixStrin", "prefix"));
  EXPECT_TRUE(startsWith("prefixString", "prefix"));
  EXPECT_FALSE(startsWith("sprefix", "prefix"));
  EXPECT_FALSE(startsWith("stprefix", "prefix"));
  EXPECT_FALSE(startsWith("strprefix", "prefix"));
  EXPECT_FALSE(startsWith("striprefix", "prefix"));
  EXPECT_FALSE(startsWith("strinprefix", "prefix"));
  EXPECT_FALSE(startsWith("stringprefix", "prefix"));
  EXPECT_FALSE(startsWith("string_prefix", "prefix"));
}

TEST(StringUtils, EndsWith)
{
  EXPECT_TRUE(endsWith("", ""));
  EXPECT_FALSE(endsWith("", "prefix"));
  EXPECT_FALSE(endsWith("p", "prefix"));
  EXPECT_FALSE(endsWith("pr", "prefix"));
  EXPECT_FALSE(endsWith("pre", "prefix"));
  EXPECT_FALSE(endsWith("pref", "prefix"));
  EXPECT_FALSE(endsWith("prefi", "prefix"));
  EXPECT_TRUE(endsWith("prefix", "prefix"));
  EXPECT_FALSE(endsWith("prefixS", "prefix"));
  EXPECT_FALSE(endsWith("prefixSt", "prefix"));
  EXPECT_FALSE(endsWith("prefixStr", "prefix"));
  EXPECT_FALSE(endsWith("prefixStri", "prefix"));
  EXPECT_FALSE(endsWith("prefixStrin", "prefix"));
  EXPECT_FALSE(endsWith("prefixString", "prefix"));
  EXPECT_TRUE(endsWith("sprefix", "prefix"));
  EXPECT_TRUE(endsWith("stprefix", "prefix"));
  EXPECT_TRUE(endsWith("strprefix", "prefix"));
  EXPECT_TRUE(endsWith("striprefix", "prefix"));
  EXPECT_TRUE(endsWith("strinprefix", "prefix"));
  EXPECT_TRUE(endsWith("stringprefix", "prefix"));
  EXPECT_TRUE(endsWith("string_prefix", "prefix"));
}

TEST(StringUtils, RemovePrefix)
{
  bool hadPrefix;
  EXPECT_EQ("", removePrefix("", ""));
  EXPECT_EQ("", removePrefix("", "prefix"));
  EXPECT_EQ("p", removePrefix("p", "prefix", &hadPrefix)); EXPECT_FALSE(hadPrefix);
  EXPECT_EQ("pr", removePrefix("pr", "prefix"));
  EXPECT_EQ("pre", removePrefix("pre", "prefix"));
  EXPECT_EQ("pref", removePrefix("pref", "prefix"));
  EXPECT_EQ("prefi", removePrefix("prefi", "prefix"));
  EXPECT_EQ("", removePrefix("prefix", "prefix", &hadPrefix)); EXPECT_TRUE(hadPrefix);
  EXPECT_EQ("S", removePrefix("prefixS", "prefix", &hadPrefix)); EXPECT_TRUE(hadPrefix);
  EXPECT_EQ("St", removePrefix("prefixSt", "prefix", &hadPrefix)); EXPECT_TRUE(hadPrefix);
  EXPECT_EQ("Str", removePrefix("prefixStr", "prefix", &hadPrefix)); EXPECT_TRUE(hadPrefix);
  EXPECT_EQ("Stri", removePrefix("prefixStri", "prefix", &hadPrefix)); EXPECT_TRUE(hadPrefix);
  EXPECT_EQ("Strin", removePrefix("prefixStrin", "prefix", &hadPrefix)); EXPECT_TRUE(hadPrefix);
  EXPECT_EQ("String", removePrefix("prefixString", "prefix", &hadPrefix)); EXPECT_TRUE(hadPrefix);
  EXPECT_EQ("sprefix", removePrefix("sprefix", "prefix", &hadPrefix)); EXPECT_FALSE(hadPrefix);
  EXPECT_EQ("stprefix", removePrefix("stprefix", "prefix", &hadPrefix)); EXPECT_FALSE(hadPrefix);
  EXPECT_EQ("strprefix", removePrefix("strprefix", "prefix", &hadPrefix)); EXPECT_FALSE(hadPrefix);
  EXPECT_EQ("striprefix", removePrefix("striprefix", "prefix", &hadPrefix)); EXPECT_FALSE(hadPrefix);
  EXPECT_EQ("strinprefix", removePrefix("strinprefix", "prefix", &hadPrefix)); EXPECT_FALSE(hadPrefix);
  EXPECT_EQ("stringprefix", removePrefix("stringprefix", "prefix", &hadPrefix)); EXPECT_FALSE(hadPrefix);
  EXPECT_EQ("string_prefix", removePrefix("string_prefix", "prefix", &hadPrefix)); EXPECT_FALSE(hadPrefix);
}

TEST(StringUtils, RemoveSuffix)
{
  bool hadSuffix;
  EXPECT_EQ("", removeSuffix("", ""));
  EXPECT_EQ("", removeSuffix("", "suffix"));
  EXPECT_EQ("s", removeSuffix("s", "suffix", &hadSuffix)); EXPECT_FALSE(hadSuffix);
  EXPECT_EQ("su", removeSuffix("su", "suffix"));
  EXPECT_EQ("suf", removeSuffix("suf", "suffix"));
  EXPECT_EQ("suff", removeSuffix("suff", "suffix"));
  EXPECT_EQ("suffi", removeSuffix("suffi", "suffix"));
  EXPECT_EQ("", removeSuffix("suffix", "suffix", &hadSuffix)); EXPECT_TRUE(hadSuffix);
  EXPECT_EQ("suffixS", removeSuffix("suffixS", "suffix", &hadSuffix)); EXPECT_FALSE(hadSuffix);
  EXPECT_EQ("suffixSt", removeSuffix("suffixSt", "suffix", &hadSuffix)); EXPECT_FALSE(hadSuffix);
  EXPECT_EQ("suffixStr", removeSuffix("suffixStr", "suffix", &hadSuffix)); EXPECT_FALSE(hadSuffix);
  EXPECT_EQ("suffixStri", removeSuffix("suffixStri", "suffix", &hadSuffix)); EXPECT_FALSE(hadSuffix);
  EXPECT_EQ("suffixStrin", removeSuffix("suffixStrin", "suffix", &hadSuffix)); EXPECT_FALSE(hadSuffix);
  EXPECT_EQ("suffixString", removeSuffix("suffixString", "suffix", &hadSuffix)); EXPECT_FALSE(hadSuffix);
  EXPECT_EQ("s", removeSuffix("ssuffix", "suffix", &hadSuffix)); EXPECT_TRUE(hadSuffix);
  EXPECT_EQ("st", removeSuffix("stsuffix", "suffix", &hadSuffix)); EXPECT_TRUE(hadSuffix);
  EXPECT_EQ("str", removeSuffix("strsuffix", "suffix", &hadSuffix)); EXPECT_TRUE(hadSuffix);
  EXPECT_EQ("stri", removeSuffix("strisuffix", "suffix", &hadSuffix)); EXPECT_TRUE(hadSuffix);
  EXPECT_EQ("strin", removeSuffix("strinsuffix", "suffix", &hadSuffix)); EXPECT_TRUE(hadSuffix);
  EXPECT_EQ("string", removeSuffix("stringsuffix", "suffix", &hadSuffix)); EXPECT_TRUE(hadSuffix);
  EXPECT_EQ("string_", removeSuffix("string_suffix", "suffix", &hadSuffix)); EXPECT_TRUE(hadSuffix);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}