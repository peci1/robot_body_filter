#ifndef ROBOT_BODY_FILTER_UTILS_FILTER_UTILS_HPP
#define ROBOT_BODY_FILTER_UTILS_FILTER_UTILS_HPP

#include <rosconsole/macros_generated.h>
#include <filters/filter_base.h>

#include "robot_body_filter/utils/string_utils.hpp"

namespace robot_body_filter
{

template<typename F>
class FilterBase : public filters::FilterBase<F>
{

protected:

  /**
   * \brief Get the value of the given filter parameter, falling back to the
   *        specified default value, and print out a ROS info/warning message with
   *        the loaded values.
   * \tparam T Param type.
   * \param name Name of the parameter.
   * \param defaultValue The default value to use.
   * \param unit Optional string serving as a [physical/SI] unit of the parameter, just to make the messages more informative.
   * \return The loaded param value.
   */
  template< typename T>
  T getParamVerbose(const std::string &name, const T &defaultValue = T(),
             const std::string &unit = "")
  {
    T value;
    if (filters::FilterBase<F>::getParam(name, value))
    {
      ROS_INFO_STREAM(this->getName() << ": Found parameter: " << name <<
        ", value: " << to_string(value) <<
        prependIfNonEmpty(unit, " "));
      return value;
    }
    else
    {
      ROS_WARN_STREAM(this->getName() << ": Cannot find value for parameter: "
        << name << ", assigning default: " << to_string(defaultValue)
        << prependIfNonEmpty(unit, " "));
    }
    return defaultValue;
  }

  std::string getParamVerbose(const std::string &name, const char* defaultValue,
                              const std::string &unit = "")
  {
    return this->getParamVerbose(name, std::string(defaultValue), unit);
  }



  // getParam specializations for unsigned values

  uint64_t getParamVerbose(const std::string &name, const uint64_t &defaultValue,
                           const std::string &unit = "")
  {
    return this->getParamUnsigned<uint64_t, int>(name, defaultValue, unit);
  }

  // there actually is an unsigned int implementation of FilterBase::getParam,
  // but it doesn't tell you when the passed value is negative - instead it just
  // returns false
  unsigned int getParamVerbose(const std::string &name,
                               const unsigned int &defaultValue,
                               const std::string &unit = "")
  {
    return this->getParamUnsigned<unsigned int, int>(name, defaultValue, unit);
  }

  // ROS types specializations

  ros::Duration getParamVerbose(const std::string &name,
                                const ros::Duration &defaultValue,
                                const std::string &unit = "")
  {
    return this->getParamCast<ros::Duration, double>(name, defaultValue.toSec(),
      unit);
  }

  template<typename Foo>
  std::set<std::string> getParamVerboseSet(
      const std::string &name,
      const std::set<std::string> &defaultValue = std::set<std::string>(),
      const std::string &unit = "")
  {
    std::vector<std::string> vector(defaultValue.begin(), defaultValue.end());
    vector = this->getParamVerbose(name, vector, unit);
    return std::set<std::string>(vector.begin(), vector.end());
  }

private:

  template<typename Result, typename Param>
  Result getParamUnsigned(const std::string &name, const Result &defaultValue,
                          const std::string &unit = "")
  {
    const Param signedValue = this->getParamVerbose(name,
      static_cast<Param>(defaultValue), unit);
    if (signedValue < 0)
    {
      ROS_ERROR_STREAM(this->getName() << ": Value " << signedValue <<
        " of unsigned parameter " << name << " is negative.");
      throw std::invalid_argument(name);
    }
    return static_cast<Result>(signedValue);
  }

  // generic casting getParam()
  template<typename Result, typename Param>
  Result getParamCast(const std::string &name, const Param &defaultValue,
                      const std::string &unit = "")
  {
    const Param paramValue = this->getParamVerbose(name, defaultValue, unit);
    return Result(paramValue);
  }

};

};
#endif //ROBOT_BODY_FILTER_UTILS_FILTER_UTILS_HPP
