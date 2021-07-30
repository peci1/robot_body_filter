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
  ASSERT_EQ(XmlRpc::XmlRpcValue::TypeBase64, XmlRpcTraits<std::vector<char>>::xmlRpcType);
  ASSERT_EQ(XmlRpc::XmlRpcValue::TypeInvalid, XmlRpcTraits<size_t>::xmlRpcType);
  ASSERT_EQ(XmlRpc::XmlRpcValue::TypeInvalid, XmlRpcTraits<unsigned int>::xmlRpcType);
  ASSERT_EQ(XmlRpc::XmlRpcValue::TypeInvalid, XmlRpcTraits<char>::xmlRpcType);
  ASSERT_EQ(XmlRpc::XmlRpcValue::TypeInvalid, XmlRpcTraits<char*>::xmlRpcType);
  ASSERT_EQ(XmlRpc::XmlRpcValue::TypeInvalid, XmlRpcTraits<float>::xmlRpcType);
}

TEST(XmlRpcTraits, String)
{
  ASSERT_EQ("bool", XmlRpcTraits<bool>::stringType);
  ASSERT_EQ("int", XmlRpcTraits<int>::stringType);
  ASSERT_EQ("double", XmlRpcTraits<double>::stringType);
  ASSERT_EQ("string", XmlRpcTraits<std::string>::stringType);
  ASSERT_EQ("datetime", XmlRpcTraits<tm>::stringType);
  ASSERT_EQ("binary", XmlRpcTraits<std::vector<char>>::stringType);
  ASSERT_EQ("invalid", XmlRpcTraits<size_t>::stringType);
  ASSERT_EQ("invalid", XmlRpcTraits<unsigned int>::stringType);
  ASSERT_EQ("invalid", XmlRpcTraits<char>::stringType);
  ASSERT_EQ("invalid", XmlRpcTraits<char*>::stringType);
  ASSERT_EQ("invalid", XmlRpcTraits<float>::stringType);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}