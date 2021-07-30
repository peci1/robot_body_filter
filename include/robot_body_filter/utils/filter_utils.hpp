#ifndef ROBOT_BODY_FILTER_UTILS_FILTER_UTILS_HPP
#define ROBOT_BODY_FILTER_UTILS_FILTER_UTILS_HPP

#include <rosconsole/macros_generated.h>
#include <ros/common.h>

#if ROS_VERSION_MINIMUM(1, 15, 0)
#include <filters/filter_base.hpp>
#else
#include <filters/filter_base.h>
#endif

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
   * \param name Name of the parameter. If the name contains slashes and the full name is not found,
   *             a "recursive" search is tried using the parts of the name separated by slashes.
   *             This is useful if the filter config isn't loaded via a filterchain config, but via
   *             a dict loaded directly to ROS parameter server.
   * \param defaultValue The default value to use.
   * \param unit Optional string serving as a [physical/SI] unit of the parameter, just to make the
   *             messages more informative.
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

    // The parameter has slashes in its name, so try a "recursive" search
    if (name.length() > 1 && name.find_first_of('/', 1) != std::string::npos)
    {
      auto slashPos = name.find_first_of('/', 1);
      auto head = name.substr(0, slashPos);
      auto tail = name.substr(slashPos + 1);
      XmlRpc::XmlRpcValue val;

      if (filters::FilterBase<F>::getParam(head, val))
      {
        while (val.getType() == XmlRpc::XmlRpcValue::TypeStruct)
        {
          if (val.hasMember(tail))
          {
            filters::FilterBase<F>::params_[name] = val[tail];
            return this->getParamVerbose(name, defaultValue, unit);
          } else {
            slashPos = tail.find_first_of('/', 1);
            if (slashPos == std::string::npos)
              break;
            head = tail.substr(0, slashPos);
            tail = tail.substr(slashPos + 1);
            if (!val.hasMember(head))
              break;
            XmlRpc::XmlRpcValue tmp = val[head]; // tmp copy is required, otherwise mem corruption
            val = tmp;
            if (!val.valid())
              break;
          }
        }
      }
    }

    ROS_WARN_STREAM(this->getName() << ": Cannot find value for parameter: "
      << name << ", assigning default: " << to_string(defaultValue)
      << prependIfNonEmpty(unit, " "));
    return defaultValue;
  }

  /** \brief Get the value of the given filter parameter, falling back to the
   *        specified default value, and print out a ROS info/warning message with
   *        the loaded values.
   * \param name Name of the parameter. If the name contains slashes and the full name is not found,
   *             a "recursive" search is tried using the parts of the name separated by slashes.
   *             This is useful if the filter config isn't loaded via a filterchain config, but via
   *             a dict loaded directly to ROS parameter server.
   * \param defaultValue The default value to use.
   * \param unit Optional string serving as a [physical/SI] unit of the parameter, just to make the
   *             messages more informative.
   * \return The loaded param value.
   */
  std::string getParamVerbose(const std::string &name, const char* defaultValue,
                              const std::string &unit = "")
  {
    return this->getParamVerbose(name, std::string(defaultValue), unit);
  }



  // getParam specializations for unsigned values

  /** \brief Get the value of the given filter parameter, falling back to the
   *        specified default value, and print out a ROS info/warning message with
   *        the loaded values.
   * \param name Name of the parameter. If the name contains slashes and the full name is not found,
   *             a "recursive" search is tried using the parts of the name separated by slashes.
   *             This is useful if the filter config isn't loaded via a filterchain config, but via
   *             a dict loaded directly to ROS parameter server.
   * \param defaultValue The default value to use.
   * \param unit Optional string serving as a [physical/SI] unit of the parameter, just to make the
   *             messages more informative.
   * \return The loaded param value.
   * \throw std::invalid_argument If the loaded value is negative.
   */
  uint64_t getParamVerbose(const std::string &name, const uint64_t &defaultValue,
                           const std::string &unit = "")
  {
    return this->getParamUnsigned<uint64_t, int>(name, defaultValue, unit);
  }

  // there actually is an unsigned int implementation of FilterBase::getParam,
  // but it doesn't tell you when the passed value is negative - instead it just
  // returns false
  /** \brief Get the value of the given filter parameter, falling back to the
   *        specified default value, and print out a ROS info/warning message with
   *        the loaded values.
   * \param name Name of the parameter. If the name contains slashes and the full name is not found,
   *             a "recursive" search is tried using the parts of the name separated by slashes.
   *             This is useful if the filter config isn't loaded via a filterchain config, but via
   *             a dict loaded directly to ROS parameter server.
   * \param defaultValue The default value to use.
   * \param unit Optional string serving as a [physical/SI] unit of the parameter, just to make the
   *             messages more informative.
   * \return The loaded param value.
   * \throw std::invalid_argument If the loaded value is negative.
   */
  unsigned int getParamVerbose(const std::string &name,
                               const unsigned int &defaultValue,
                               const std::string &unit = "")
  {
    return this->getParamUnsigned<unsigned int, int>(name, defaultValue, unit);
  }

  // ROS types specializations

  /** \brief Get the value of the given filter parameter, falling back to the
   *        specified default value, and print out a ROS info/warning message with
   *        the loaded values.
   * \param name Name of the parameter. If the name contains slashes and the full name is not found,
   *             a "recursive" search is tried using the parts of the name separated by slashes.
   *             This is useful if the filter config isn't loaded via a filterchain config, but via
   *             a dict loaded directly to ROS parameter server.
   * \param defaultValue The default value to use.
   * \param unit Optional string serving as a [physical/SI] unit of the parameter, just to make the
   *             messages more informative.
   * \return The loaded param value.
   */
  ros::Duration getParamVerbose(const std::string &name,
                                const ros::Duration &defaultValue,
                                const std::string &unit = "")
  {
    return this->getParamCast<ros::Duration, double>(name, defaultValue.toSec(),
      unit);
  }

  /** \brief Get the value of the given filter parameter as a set of strings, falling back to the
   *        specified default value, and print out a ROS info/warning message with
   *        the loaded values.
   * \tparam Foo Ignored. Just needed for compilation to succeed.
   * \param name Name of the parameter. If the name contains slashes and the full name is not found,
   *             a "recursive" search is tried using the parts of the name separated by slashes.
   *             This is useful if the filter config isn't loaded via a filterchain config, but via
   *             a dict loaded directly to ROS parameter server.
   * \param defaultValue The default value to use.
   * \param unit Optional string serving as a [physical/SI] unit of the parameter, just to make the
   *             messages more informative.
   * \return The loaded param value.
   * \throw std::invalid_argument If the loaded value is negative.
   */
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
