#ifndef ROBOT_BODY_FILTER_XMLRPC_TRAITS_H
#define ROBOT_BODY_FILTER_XMLRPC_TRAITS_H

#include "robot_body_filter/utils/string_utils.hpp"

namespace robot_body_filter
{

template<typename T>
struct XmlRpcTraits
{
  inline static const XmlRpc::XmlRpcValue::Type xmlRpcType { XmlRpc::XmlRpcValue::TypeInvalid };
  inline static const std::string stringType { to_string(xmlRpcType) };
};

template<> struct XmlRpcTraits<bool>
{
  inline static const XmlRpc::XmlRpcValue::Type xmlRpcType { XmlRpc::XmlRpcValue::TypeBoolean };
  inline static const std::string stringType { to_string(xmlRpcType) };
};

template<> struct XmlRpcTraits<int>
{
  inline static const XmlRpc::XmlRpcValue::Type xmlRpcType { XmlRpc::XmlRpcValue::TypeInt };
  inline static const std::string stringType { to_string(xmlRpcType) };
};

template<> struct XmlRpcTraits<double>
{
  inline static const XmlRpc::XmlRpcValue::Type xmlRpcType { XmlRpc::XmlRpcValue::TypeDouble };
  inline static const std::string stringType { to_string(xmlRpcType) };
};

template<> struct XmlRpcTraits<std::string>
{
  inline static const XmlRpc::XmlRpcValue::Type xmlRpcType { XmlRpc::XmlRpcValue::TypeString };
  inline static const std::string stringType { to_string(xmlRpcType) };
};

template<> struct XmlRpcTraits<tm>
{
  inline static const XmlRpc::XmlRpcValue::Type xmlRpcType { XmlRpc::XmlRpcValue::TypeDateTime };
  inline static const std::string stringType { to_string(xmlRpcType) };
};

template<> struct XmlRpcTraits<typename XmlRpc::XmlRpcValue::BinaryData>
{
  inline static const XmlRpc::XmlRpcValue::Type xmlRpcType { XmlRpc::XmlRpcValue::TypeBase64 };
  inline static const std::string stringType { to_string(xmlRpcType) };

};

}

#endif //ROBOT_BODY_FILTER_XMLRPC_TRAITS_H
