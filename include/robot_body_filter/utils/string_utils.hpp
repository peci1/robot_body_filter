#ifndef ROBOT_BODY_FILTER_UTILS_TOPIC_UTILS_HPP
#define ROBOT_BODY_FILTER_UTILS_TOPIC_UTILS_HPP

#include <iterator>
#include <map>
#include <ros/duration.h>
#include <ros/time.h>
#include <set>
#include <string>
#include <sstream>
#include <type_traits>
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

/**
 * \brief Test if `str` starts with `prefix`.
 * \param str The string to test.
 * \param prefix The prefix to find.
 * \return Whether `str` starts with `prefix`.
 */
bool startsWith(const std::string& str, const std::string& prefix);

/**
 * \brief Test if `str` ends with `suffix`.
 * \param str The string to test.
 * \param suffix The suffix to find.
 * \return Whether `str` ends with `suffix`.
 */
bool endsWith(const std::string& str, const std::string& suffix);

/**
 * \brief Remove `prefix` from start of `str` if it contains it, otherwise return `str` unchanged.
 * \param str The string to work on.
 * \param prefix The prefix to find.
 * \param hadPrefix If non-null, will contain information whether `str` starts with `prefix`.
 * \return If `str` starts with `prefix`, it will return `str` with `prefix` removed. Otherwise, `str` will be returned unchanged.
 */
std::string removePrefix(const std::string& str, const std::string& prefix, bool* hadPrefix = nullptr);

/**
 * \brief Remove `suffix` from end of `str` if it contains it, otherwise return `str` unchanged.
 * \param str The string to work on.
 * \param suffix The suffix to find.
 * \param hadSuffix If non-null, will contain information whether `str` ends with `suffix`.
 * \return If `str` ends with `suffix`, it will return `str` with `suffix` removed. Otherwise, `str` will be returned unchanged.
 */
std::string removeSuffix(const std::string& str, const std::string& suffix, bool* hadSuffix = nullptr);

template<typename T>
inline std::string to_string(const T &value)
{
  return std::to_string(value);
}

template<>
inline std::string to_string(const bool &value)
{
  return value ? "True" : "False";
}

template<>
inline std::string to_string(const std::string &value)
{
  return value;
}

template<>
inline std::string to_string(const XmlRpc::XmlRpcValue &value)
{
  std::stringstream ss;
  ss << value;
  return ss.str();
}

template<typename T>
inline std::string to_string(const std::vector<T> &value)
{
  std::stringstream ss;
  ss << "[";
  for (size_t i = 0; i < value.size(); ++i)
  {
    if (std::is_same<std::string, T>::value)
      ss << "\"" << to_string(value[i]) << "\"";
    else
      ss << to_string(value[i]);
    if (i + 1 < value.size())
      ss << ", ";
  }
  ss << "]";
  return ss.str();
}

template<typename T>
inline std::string to_string(const std::set<T> &value)
{
  std::stringstream ss;
  ss << "[";
  size_t i = 0;
  for (const auto& v : value)
  {
    if (std::is_same<std::string, T>::value)
      ss << "\"" << to_string(v) << "\"";
    else
      ss << to_string(v);
    if (i + 1 < value.size())
      ss << ", ";
    ++i;
  }
  ss << "]";
  return ss.str();
}

template<typename K, typename V>
inline std::string to_string(const std::map<K, V> &value)
{
  std::stringstream ss;
  ss << "{";
  size_t i = 0;
  for (const auto &pair : value)
  {
    if (std::is_same<std::string, K>::value)
      ss << "\"" << to_string(pair.first) << "\"";
    else
      ss << to_string(pair.first);

    ss << ": ";

    if (std::is_same<std::string, V>::value)
      ss << "\"" << to_string(pair.second) << "\"";
    else
      ss << to_string(pair.second);

    if (i + 1 < value.size())
      ss << ", ";
    ++i;
  }
  ss << "}";
  return ss.str();
}

template<>
inline std::string to_string(const ros::Time& value)
{
  std::stringstream ss;
  ss << value;
  return ss.str();
}

template<>
inline std::string to_string(const ros::WallTime& value)
{
  std::stringstream ss;
  ss << value;
  return ss.str();
}

template<>
inline std::string to_string(const ros::SteadyTime& value)
{
  std::stringstream ss;
  ss << value;
  return ss.str();
}

template<>
inline std::string to_string(const ros::Duration& value)
{
  std::stringstream ss;
  ss << value;
  return ss.str();
}

template<>
inline std::string to_string(const ros::WallDuration& value)
{
  std::stringstream ss;
  ss << value;
  return ss.str();
}

};
#endif //ROBOT_BODY_FILTER_UTILS_TOPIC_UTILS_HPP