#include "robot_body_filter/utils/string_utils.hpp"

#include <ros/ros.h>
#include <rosconsole/macros_generated.h>

namespace robot_body_filter {

void warnLeadingSlash(const std::string& s)
{
  ROS_WARN_STREAM_ONCE("Found initial slash in " << s);
}

void stripLeadingSlash(std::string &s, const bool warn)
{
  if (s.length() > 0 && s[0] == '/')
  {
    if (warn) {
      warnLeadingSlash(s);
    }
    s.erase(0, 1);
  }
}

std::string stripLeadingSlash(const std::string &s, const bool warn)
{
  if (s.length() > 0 && s[0] == '/')
  {
    if (warn) {
      warnLeadingSlash(s);
    }
    return s.substr(1);
  }

  return s;
}

std::string prependIfNonEmpty(const std::string &str, const std::string &prefix)
{
  return str.empty() ? str : prefix + str;
}

std::string appendIfNonEmpty(const std::string &str, const std::string &suffix)
{
  return str.empty() ? str : str + suffix;
}

bool startsWith(const std::string &str, const std::string &prefix)
{
  if (prefix.empty())
    return true; // special case
  if (str.length() < prefix.length())
    return false;

  return std::mismatch(prefix.cbegin(), prefix.cend(), str.cbegin()).first == prefix.cend();
}

bool endsWith(const std::string &str, const std::string &suffix)
{
  if (suffix.empty())
    return true; // special case
  if (str.length() < suffix.length())
    return false;

  return std::mismatch(suffix.crbegin(), suffix.crend(), str.crbegin()).first == suffix.crend();
}

std::string removePrefix(const std::string &str, const std::string &prefix, bool *hadPrefix)
{
  const auto hasPrefix = startsWith(str, prefix);
  if (hadPrefix != nullptr)
    *hadPrefix = hasPrefix;

  return hasPrefix ? str.substr(prefix.length()) : str;
}

std::string removeSuffix(const std::string &str, const std::string &suffix, bool *hadSuffix)
{
  const auto hasSuffix = endsWith(str, suffix);
  if (hadSuffix != nullptr)
    *hadSuffix = hasSuffix;

  return hasSuffix ? str.substr(0, str.length() - suffix.length()) : str;
}

};