#include "gtest/gtest.h"
#include <robot_body_filter/utils/xmlrpc_traits.h>

using namespace robot_body_filter;

TEST(XmlRpcTraits, Type)
{
  ASSERT_EQ(XmlRpc::XmlRpcValue::TypeBoolean, XmlRpcTraits<bool>::xmlRpcType);
  ASSERT_EQ(XmlRpc::XmlRpcValue::TypeInt, XmlRpcTraits<int>::xmlRpcType);
  ASSERT_EQ(XmlRpc::XmlRpcValue::TypeDouble, XmlRpcTraits<double>::xmlRpcType);
  ASSERT_EQ(XmlRpc::XmlRpcValue::TypeString, XmlRpcTraits<std::string>::xmlRpcType);
  ASSERT_EQ(XmlRpc::XmlRpcValue::TypeDateTime, XmlRpcTraits<tm>::xmlRpcType);
  ASSERT_EQ(XmlRpc::XmlRpcValue::TypeArray, XmlRpcTraits<std::vector<bool>>::xmlRpcType);
  ASSERT_EQ(XmlRpc::XmlRpcValue::TypeArray, XmlRpcTraits<std::vector<int>>::xmlRpcType);
  ASSERT_EQ(XmlRpc::XmlRpcValue::TypeInvalid, XmlRpcTraits<std::vector<float>>::xmlRpcType);
  ASSERT_EQ(XmlRpc::XmlRpcValue::TypeArray, XmlRpcTraits<std::vector<double>>::xmlRpcType);
  ASSERT_EQ(XmlRpc::XmlRpcValue::TypeArray, XmlRpcTraits<std::vector<std::string>>::xmlRpcType);
  ASSERT_EQ(XmlRpc::XmlRpcValue::TypeBase64, XmlRpcTraits<std::vector<char>>::xmlRpcType);
  ASSERT_EQ(XmlRpc::XmlRpcValue::TypeStruct, (XmlRpcTraits<std::map<std::string, bool> >::xmlRpcType));
  ASSERT_EQ(XmlRpc::XmlRpcValue::TypeStruct, (XmlRpcTraits<std::map<std::string, int> >::xmlRpcType));
  ASSERT_EQ(XmlRpc::XmlRpcValue::TypeInvalid, (XmlRpcTraits<std::map<std::string, float> >::xmlRpcType));
  ASSERT_EQ(XmlRpc::XmlRpcValue::TypeStruct, (XmlRpcTraits<std::map<std::string, double> >::xmlRpcType));
  ASSERT_EQ(XmlRpc::XmlRpcValue::TypeInvalid, (XmlRpcTraits<std::map<std::string, char*> >::xmlRpcType));
  ASSERT_EQ(XmlRpc::XmlRpcValue::TypeStruct, (XmlRpcTraits<std::map<std::string, std::string> >::xmlRpcType));
  ASSERT_EQ(XmlRpc::XmlRpcValue::TypeInvalid, XmlRpcTraits<size_t>::xmlRpcType);
  ASSERT_EQ(XmlRpc::XmlRpcValue::TypeInvalid, XmlRpcTraits<unsigned int>::xmlRpcType);
  ASSERT_EQ(XmlRpc::XmlRpcValue::TypeInvalid, XmlRpcTraits<char>::xmlRpcType);
  ASSERT_EQ(XmlRpc::XmlRpcValue::TypeInvalid, XmlRpcTraits<char*>::xmlRpcType);
  ASSERT_EQ(XmlRpc::XmlRpcValue::TypeInvalid, XmlRpcTraits<float>::xmlRpcType);
  ASSERT_EQ(XmlRpc::XmlRpcValue::TypeInvalid, (XmlRpcTraits<std::map<bool, std::string> >::xmlRpcType));
  ASSERT_EQ(XmlRpc::XmlRpcValue::TypeInvalid, (XmlRpcTraits<std::map<int, std::string> >::xmlRpcType));
  ASSERT_EQ(XmlRpc::XmlRpcValue::TypeInvalid, (XmlRpcTraits<std::map<double, int> >::xmlRpcType));
  ASSERT_EQ(XmlRpc::XmlRpcValue::TypeStruct, (XmlRpcTraits<std::map<std::string, std::map<std::string, int>> >::xmlRpcType));
  ASSERT_EQ(XmlRpc::XmlRpcValue::TypeInvalid, (XmlRpcTraits<std::map<std::string, std::map<std::string, float>> >::xmlRpcType));
}

TEST(XmlRpcTraits, String)
{
  ASSERT_EQ("bool", XmlRpcTraits<bool>::stringType);
  ASSERT_EQ("int", XmlRpcTraits<int>::stringType);
  ASSERT_EQ("double", XmlRpcTraits<double>::stringType);
  ASSERT_EQ("string", XmlRpcTraits<std::string>::stringType);
  ASSERT_EQ("datetime", XmlRpcTraits<tm>::stringType);
  ASSERT_EQ("array", XmlRpcTraits<std::vector<bool>>::stringType);
  ASSERT_EQ("array", XmlRpcTraits<std::vector<int>>::stringType);
  ASSERT_EQ("invalid", XmlRpcTraits<std::vector<float>>::stringType);
  ASSERT_EQ("array", XmlRpcTraits<std::vector<double>>::stringType);
  ASSERT_EQ("array", XmlRpcTraits<std::vector<std::string>>::stringType);
  ASSERT_EQ("binary", XmlRpcTraits<std::vector<char>>::stringType);
  ASSERT_EQ("struct", (XmlRpcTraits<std::map<std::string, bool>>::stringType));
  ASSERT_EQ("struct", (XmlRpcTraits<std::map<std::string, int>>::stringType));
  ASSERT_EQ("invalid", (XmlRpcTraits<std::map<std::string, float>>::stringType));
  ASSERT_EQ("struct", (XmlRpcTraits<std::map<std::string, double>>::stringType));
  ASSERT_EQ("invalid", (XmlRpcTraits<std::map<std::string, char*>>::stringType));
  ASSERT_EQ("struct", (XmlRpcTraits<std::map<std::string, std::string>>::stringType));
  ASSERT_EQ("invalid", XmlRpcTraits<size_t>::stringType);
  ASSERT_EQ("invalid", XmlRpcTraits<unsigned int>::stringType);
  ASSERT_EQ("invalid", XmlRpcTraits<char>::stringType);
  ASSERT_EQ("invalid", XmlRpcTraits<char*>::stringType);
  ASSERT_EQ("invalid", XmlRpcTraits<float>::stringType);
  ASSERT_EQ("invalid", (XmlRpcTraits<std::map<bool, std::string>>::stringType));
  ASSERT_EQ("invalid", (XmlRpcTraits<std::map<int, std::string>>::stringType));
  ASSERT_EQ("invalid", (XmlRpcTraits<std::map<double, int>>::stringType));
  ASSERT_EQ("struct", (XmlRpcTraits<std::map<std::string, std::map<std::string, int>>>::stringType));
  ASSERT_EQ("invalid", (XmlRpcTraits<std::map<std::string, std::map<std::string, float>>>::stringType));
}

TEST(XmlRpcTraits, ToString)
{
  EXPECT_EQ("bool", to_string(XmlRpc::XmlRpcValue::TypeBoolean));
  EXPECT_EQ("int", to_string(XmlRpc::XmlRpcValue::TypeInt));
  EXPECT_EQ("double", to_string(XmlRpc::XmlRpcValue::TypeDouble));
  EXPECT_EQ("string", to_string(XmlRpc::XmlRpcValue::TypeString));
  EXPECT_EQ("datetime", to_string(XmlRpc::XmlRpcValue::TypeDateTime));
  EXPECT_EQ("binary", to_string(XmlRpc::XmlRpcValue::TypeBase64));
  EXPECT_EQ("struct", to_string(XmlRpc::XmlRpcValue::TypeStruct));
  EXPECT_EQ("array", to_string(XmlRpc::XmlRpcValue::TypeArray));
  EXPECT_EQ("invalid", to_string(XmlRpc::XmlRpcValue::TypeInvalid));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}