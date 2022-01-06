#ifndef ROBOT_BODY_FILTER_UTILS_FILTER_UTILS_HPP
#define ROBOT_BODY_FILTER_UTILS_FILTER_UTILS_HPP

#include <rosconsole/macros_generated.h>
#include <ros/common.h>
#include <map>

#if ROS_VERSION_MINIMUM(1, 15, 0)
#include <filters/filter_base.hpp>
#else
#include <filters/filter_base.h>
#endif

#include "robot_body_filter/utils/string_utils.hpp"
#include "robot_body_filter/utils/xmlrpc_traits.h"

namespace robot_body_filter
{

namespace
{
/**
 * \brief Internal use only. This class exposes the XmlRpcValue -> typed data conversion
 * provided by filters::FilterBase::getParam(), which is normally protected. We use this
 * workaround to avoid copy-pasting the code here.
 */
template<typename F>
class FilterParamHelper : public filters::FilterBase<F>
{
public:
  FilterParamHelper(const std::string& key, const XmlRpc::XmlRpcValue& value)
  {
    this->params_[key] = value[key];
  }
  template<typename T> bool getParamHelper(const std::string& name, T& value) const
  {
    return filters::FilterBase<F>::getParam(name, value);
  }
  bool update(const F& data_in, F& data_out) override {return false;}
protected:
  bool configure() override {return false;}
};
}

template<typename F>
class FilterBase : public filters::FilterBase<F>
{

protected:

  /** \brief Type of function that converts anything to a string. */
  template <typename T> using ToStringFn = std::string (*)(const T&);

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
   * \param defaultUsed Whether the default value was used.
   * \param valueToStringFn Function that converts valid/default values to string (for console
   *             logging). Set to nullptr to disable logging.
   * \return The loaded param value.
   */
  template<typename T>
  T getParamVerbose(const std::string &name, const T &defaultValue = T(),
             const std::string &unit = "", bool* defaultUsed = nullptr,
             ToStringFn<T> valueToStringFn = &to_string) const
  {
    T value;
    if (filters::FilterBase<F>::getParam(name, value))
    {
      if (valueToStringFn != nullptr)
      {
        ROS_INFO_STREAM(this->getName() << ": Found parameter: " << name <<
                                        ", value: " << valueToStringFn(value) <<
                                        prependIfNonEmpty(unit, " "));
      }
      if (defaultUsed != nullptr)
        *defaultUsed = false;
      return value;
    }
    else if (this->params_.find(name) != this->params_.end())
    {  // the parameter was found, but has a wrong type
      ROS_ERROR_STREAM(this->getName() << ": Parameter " << name << " found, "
        "but its value has a wrong type. Expected XmlRpc type " <<
        XmlRpcTraits<T>::stringType << ", got type: " <<
        to_string(this->params_.at(name).getType()) <<
        ". Using the default value instead.");
    }
    else if (name.length() > 1 && name.find_first_of('/', 1) != std::string::npos)
    {  // The parameter has slashes in its name, so try a "recursive" search
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
            auto tmpFilter = FilterParamHelper<F>(tail, val);
            if (tmpFilter.getParamHelper(tail, value))
            {
              if (defaultUsed != nullptr)
                *defaultUsed = false;
              if (valueToStringFn != nullptr)
              {
                ROS_INFO_STREAM(this->getName() << ": Found parameter: " << name <<
                  ", value: " << valueToStringFn(value) << prependIfNonEmpty(unit, " "));
              }
              return value;
            }
            else
            {
              ROS_ERROR_STREAM(this->getName() << ": Parameter " << name << " found, "
                "but its value has a wrong type. Expected XmlRpc type " <<
                XmlRpcTraits<T>::stringType << ", got type: " <<
                to_string(val[tail].getType()) <<
                ". Using the default value instead.");
              break;
            }
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

    if (valueToStringFn != nullptr)
    {
      ROS_INFO_STREAM(this->getName() << ": Parameter " << name
                                      << " not defined, assigning default: "
                                      << valueToStringFn(defaultValue)
                                      << prependIfNonEmpty(unit, " "));
    }
    if (defaultUsed != nullptr)
      *defaultUsed = true;
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
                              const std::string &unit = "", bool* defaultUsed = nullptr,
                              ToStringFn<std::string> valueToStringFn = &to_string) const
  {
    return this->getParamVerbose(name, std::string(defaultValue), unit, defaultUsed, valueToStringFn);
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
                           const std::string &unit = "", bool* defaultUsed = nullptr,
                           ToStringFn<int> valueToStringFn = &to_string) const
  {
    return this->getParamUnsigned<uint64_t, int>(name, defaultValue, unit, defaultUsed,
        valueToStringFn);
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
                               const std::string &unit = "",
                               bool* defaultUsed = nullptr,
                               ToStringFn<int> valueToStringFn = &to_string) const
  {
    return this->getParamUnsigned<unsigned int, int>(name, defaultValue, unit, defaultUsed,
        valueToStringFn);
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
                                const std::string &unit = "",
                                bool* defaultUsed = nullptr,
                                ToStringFn<double> valueToStringFn = &to_string) const
  {
    return this->getParamCast<ros::Duration, double>(name, defaultValue.toSec(), unit, defaultUsed,
        valueToStringFn);
  }

  /** \brief Get the value of the given filter parameter as a set of strings, falling back to the
   *        specified default value, and print out a ROS info/warning message with
   *        the loaded values.
   * \tparam T Type of the values in the set. Only std::string and double are supported.
   * \param name Name of the parameter. If the name contains slashes and the full name is not found,
   *             a "recursive" search is tried using the parts of the name separated by slashes.
   *             This is useful if the filter config isn't loaded via a filterchain config, but via
   *             a dict loaded directly to ROS parameter server.
   * \param defaultValue The default value to use.
   * \param unit Optional string serving as a [physical/SI] unit of the parameter, just to make the
   *             messages more informative.
   * \return The loaded param value.
   */
  template<typename T>
  std::set<T> getParamVerboseSet(
      const std::string &name,
      const std::set<T> &defaultValue = std::set<T>(),
      const std::string &unit = "",
      bool* defaultUsed = nullptr,
      ToStringFn<std::vector<T>> valueToStringFn = &to_string) const
  {
    std::vector<T> vector(defaultValue.begin(), defaultValue.end());
    vector = this->getParamVerbose(name, vector, unit, defaultUsed, valueToStringFn);
    return std::set<T>(vector.begin(), vector.end());
  }

  template<typename T, typename MapType=std::map<std::string, T>>
  MapType getParamVerboseMap(
      const std::string &name,
      const std::map<std::string, T> &defaultValue = std::map<std::string, T>(),
      const std::string &unit = "",
      bool* defaultUsed = nullptr,
      ToStringFn<MapType> valueToStringFn = &to_string) const
  {
    // convert default value to XmlRpc so that we can utilize FilterBase::getParam(XmlRpcValue).
    XmlRpc::XmlRpcValue defaultValueXmlRpc;
    defaultValueXmlRpc.begin(); // calls assertStruct() which mutates this value into a struct
    for (const auto& val : defaultValue)
      defaultValueXmlRpc[val.first] = val.second;

    // get the param value as a XmlRpcValue
    bool innerDefaultUsed;
    auto valueXmlRpc = this->getParamVerbose(name, defaultValueXmlRpc, unit, &innerDefaultUsed,
                                             (ToStringFn<XmlRpc::XmlRpcValue>)nullptr);

    // convert to map
    MapType value;
    bool hasWrongTypeItems = false;
    for (auto& pairXmlRpc : valueXmlRpc)
    {
      if (pairXmlRpc.second.getType() == XmlRpcTraits<T>::xmlRpcType)
      {
        value[pairXmlRpc.first] = pairXmlRpc.second.operator T&();
      }
      else if (XmlRpcTraits<T>::xmlRpcType == XmlRpc::XmlRpcValue::TypeDouble && pairXmlRpc.second.getType() == XmlRpc::XmlRpcValue::TypeInt)
      {
        // special handling of the case when doubles are expected but an int is provided
        value[pairXmlRpc.first] = static_cast<T>(pairXmlRpc.second.operator int&());
      }
      else
      {
        ROS_WARN_STREAM(this->getName() << ": Invalid value for dict parameter " << name
          << " key " << pairXmlRpc.first << ". Expected XmlRpc type " << XmlRpcTraits<T>::stringType
          << ", got type: " << to_string(pairXmlRpc.second.getType()) << ". Skipping value.");
        hasWrongTypeItems = true;
      }
    }

    if (value.empty() && hasWrongTypeItems)
    {
      value = defaultValue;
      if (defaultUsed != nullptr)
        *defaultUsed = true;
      if (valueToStringFn != nullptr)
      {
        ROS_ERROR_STREAM(this->getName() << ": Dict parameter " << name
                                         << " got only invalid types of values, assigning default: "
                                         << valueToStringFn(defaultValue)
                                         << prependIfNonEmpty(unit, " "));
      }
    } else {
      if (defaultUsed != nullptr)
        *defaultUsed = innerDefaultUsed;
      if (valueToStringFn != nullptr)
      {
        if (innerDefaultUsed)
        {
          ROS_INFO_STREAM(this->getName() << ": Parameter " << name
                                          << " not defined, assigning default: "
                                          << valueToStringFn(defaultValue)
                                          << prependIfNonEmpty(unit, " "));
        }
        else
        {
          ROS_INFO_STREAM(this->getName() << ": Found parameter: " << name <<
                                          ", value: " << valueToStringFn(value) <<
                                          prependIfNonEmpty(unit, " "));
        }
      }
    }

    return value;
  }

private:

  template<typename Result, typename Param>
  Result getParamUnsigned(const std::string &name, const Result &defaultValue,
                          const std::string &unit = "", bool* defaultUsed = nullptr,
                          ToStringFn<Param> valueToStringFn = &to_string) const
  {
    const Param signedValue = this->getParamVerbose(name, static_cast<Param>(defaultValue), unit,
        defaultUsed, valueToStringFn);
    if (signedValue < 0)
    {
      if (valueToStringFn != nullptr)
      {
        ROS_ERROR_STREAM(this->getName() << ": Value " << valueToStringFn(signedValue) <<
                                         " of unsigned parameter " << name << " is negative.");
      }
      throw std::invalid_argument(name);
    }
    return static_cast<Result>(signedValue);
  }

  // generic casting getParam()
  template<typename Result, typename Param>
  Result getParamCast(const std::string &name, const Param &defaultValue,
                      const std::string &unit = "", bool* defaultUsed = nullptr,
                      ToStringFn<Param> valueToStringFn = &to_string) const
  {
    const Param paramValue = this->getParamVerbose(name, defaultValue, unit, defaultUsed,
        valueToStringFn);
    return Result(paramValue);
  }

};

}
#endif //ROBOT_BODY_FILTER_UTILS_FILTER_UTILS_HPP
