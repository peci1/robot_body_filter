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

bool startsWith(const std::string &str, const std::string &prefix) {
  return str.size() >= prefix.size() && str.compare(0, prefix.size(), prefix) == 0;
}

bool endsWith(const std::string &str, const std::string &suffix) {
  return str.size() >= suffix.size() && str.compare(str.size() - suffix.size(), suffix.size(), suffix) == 0;
}

}