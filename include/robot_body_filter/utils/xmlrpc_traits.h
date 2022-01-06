#ifndef ROBOT_BODY_FILTER_XMLRPC_TRAITS_H
#define ROBOT_BODY_FILTER_XMLRPC_TRAITS_H

#include "robot_body_filter/utils/string_utils.hpp"

namespace robot_body_filter
{

constexpr const char* to_cstring(const XmlRpc::XmlRpcValue::Type &value)
{
  switch (value)
  {
    case XmlRpc::XmlRpcValue::TypeBoolean:
      return "bool";
    case XmlRpc::XmlRpcValue::TypeInt:
      return "int";
    case XmlRpc::XmlRpcValue::TypeDouble:
      return "double";
    case XmlRpc::XmlRpcValue::TypeString:
      return "string";
    case XmlRpc::XmlRpcValue::TypeDateTime:
      return "datetime";
    case XmlRpc::XmlRpcValue::TypeBase64:
      return "binary";
    case XmlRpc::XmlRpcValue::TypeArray:
      return "array";
    case XmlRpc::XmlRpcValue::TypeStruct:
      return "struct";
    default:
      return "invalid";
  }
}

template<>
inline std::string to_string(const XmlRpc::XmlRpcValue::Type &value)
{
  return to_cstring(value);
}

template<typename T, class Enable = void>
struct XmlRpcTraits
{
  constexpr static const XmlRpc::XmlRpcValue::Type xmlRpcType { XmlRpc::XmlRpcValue::TypeInvalid };
  constexpr static const char* stringType { to_cstring(xmlRpcType) };
};

template<> struct XmlRpcTraits<bool>
{
  constexpr static const XmlRpc::XmlRpcValue::Type xmlRpcType { XmlRpc::XmlRpcValue::TypeBoolean };
  constexpr static const char* stringType  { to_cstring(xmlRpcType) };
};

template<> struct XmlRpcTraits<int>
{
  constexpr static const XmlRpc::XmlRpcValue::Type xmlRpcType { XmlRpc::XmlRpcValue::TypeInt };
  constexpr static const char* stringType  { to_cstring(xmlRpcType) };
};

template<> struct XmlRpcTraits<double>
{
  constexpr static const XmlRpc::XmlRpcValue::Type xmlRpcType { XmlRpc::XmlRpcValue::TypeDouble };
  constexpr static const char* stringType  { to_cstring(xmlRpcType) };
};

template<> struct XmlRpcTraits<std::string>
{
  constexpr static const XmlRpc::XmlRpcValue::Type xmlRpcType { XmlRpc::XmlRpcValue::TypeString };
  constexpr static const char* stringType  { to_cstring(xmlRpcType) };
};

template<> struct XmlRpcTraits<tm>
{
  constexpr static const XmlRpc::XmlRpcValue::Type xmlRpcType { XmlRpc::XmlRpcValue::TypeDateTime };
  constexpr static const char* stringType  { to_cstring(xmlRpcType) };
};

template<typename T> struct XmlRpcTraits<std::vector<T>, typename std::enable_if<XmlRpcTraits<T>::xmlRpcType != XmlRpc::XmlRpcValue::TypeInvalid>::type>
{
  constexpr static const XmlRpc::XmlRpcValue::Type xmlRpcType { XmlRpc::XmlRpcValue::TypeArray };
  constexpr static const char* stringType  { to_cstring(xmlRpcType) };
};
  
template<> struct XmlRpcTraits<typename XmlRpc::XmlRpcValue::BinaryData>
{
  constexpr static const XmlRpc::XmlRpcValue::Type xmlRpcType { XmlRpc::XmlRpcValue::TypeBase64 };
  constexpr static const char* stringType  { to_cstring(xmlRpcType) };
};

template<typename T> struct XmlRpcTraits<std::map<std::string, T>, typename std::enable_if<XmlRpcTraits<T>::xmlRpcType != XmlRpc::XmlRpcValue::TypeInvalid>::type>
{
  constexpr static const XmlRpc::XmlRpcValue::Type xmlRpcType { XmlRpc::XmlRpcValue::TypeStruct };
  constexpr static const char* stringType  { to_cstring(xmlRpcType) };
};

}

#endif //ROBOT_BODY_FILTER_XMLRPC_TRAITS_H
