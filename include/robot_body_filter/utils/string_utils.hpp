#ifndef ROBOT_BODY_FILTER_UTILS_TOPIC_UTILS_HPP
#define ROBOT_BODY_FILTER_UTILS_TOPIC_UTILS_HPP

#include <iterator>
#include <map>
#include <string>
#include <sstream>
#include <vector>
#include <xmlrpcpp/XmlRpcValue.h>

namespace robot_body_filter {

/**
 * \brief Strip leading slash from the given string (if there is one).
 * \param s The string from which slash should be removed.
 * \param warn If true, issue a ROS warning if the string contained the leading slash.
 */
void stripLeadingSlash(std::string &s, bool warn = false);

/**
 * \brief Return a copy of the given string with leading slash removed (if there is one).
 * \param s The string from which slash should be removed.
 * \param warn If true, issue a ROS warning if the string contained the leading slash.
 * \return The string without leading slash.
 */
std::string stripLeadingSlash(const std::string &s, bool warn = false);

/**
 * \brief If `str` is nonempty, returns prefix + str, otherwise empty string.
 * \param str The main string.
 * \param prefix The string's prefix.
 * \return The possibly prefixed string.
 */
std::string prependIfNonEmpty(const std::string &str, const std::string &prefix);

/**
 * \brief If `str` is nonempty, returns str + suffix, otherwise empty string.
 * \param str The main string.
 * \param suffix The string's suffix.
 * \return The possibly suffixed string.
 */
std::string appendIfNonEmpty(const std::string &str, const std::string &suffix);

template<typename T>
inline std::string to_string(const T &value)
{
  return std::to_string(value);
}

template<>
inline std::string to_string(const std::string &value)
{
  return value;
}

template<>
inline std::string to_string(const XmlRpc::XmlRpcValue &value)
{
  return value.toXml();
}

template<typename T>
inline std::string to_string(const std::vector<T> &value)
{
  std::stringstream ss;
  ss << "[";
  std::copy(value.begin(), value.end(), std::ostream_iterator<T>(ss, ", "));
  ss << "]";
  return ss.str();
}

template<typename K, typename V>
inline std::string to_string(const std::map<K, V> &value)
{
  std::stringstream ss;
  ss << "{";
  for (const auto &pair : value)
  {
    ss << "\"" << to_string(pair.first) << "\": \"" << to_string(pair.second) << "\", ";
  }
  ss << "}";
  return ss.str();
}

};
#endif //ROBOT_BODY_FILTER_UTILS_TOPIC_UTILS_HPP