#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <map>
#include <set>
#include <string>
#include <vector>

#include <cras_cpp_common/string_utils.hpp>
#include <filters/filter_base.hpp>

namespace robot_body_filter {

template<typename F>
class FilterBase : public filters::FilterBase<F> {
protected:
  /** \brief Type of function that converts anything to a string. */
  template<typename T>
  using ToStringFn = std::string (*)(const T&);

  /**
   * \brief Tell whether a parameter has been specified.
   *
   * \param[in] name Name of the parameter.
   * \return Whether the parameter is specified.
   */
  bool hasParam(const std::string& name) const {
    const auto param_name = this->param_prefix_ + name;
    if (this->params_interface_->has_parameter(param_name)) {
      return true;
    }
    const auto overrides = this->params_interface_->get_parameter_overrides();
    return overrides.count(param_name) > 0;
  }

  /**
   * \brief Get the value of the given filter parameter, falling back to the
   *        specified default value, and print out a ROS info/warning message with
   *        the loaded values.
   * \tparam T Param type.
   * \param[in] name Name of the parameter (without the `filterN.params.` prefix).
   * \param[in] default_value The default value to use.
   * \param[in] unit Optional string serving as a [physical/SI] unit of the parameter, just to make the
   *                 messages more informative.
   * \param[out] default_used Whether the default value was used.
   * \param[in] value_to_string_fn Function that converts valid/default values to string (for console logging).
   *                            Set to nullptr to disable logging.
   * \return The loaded param value.
   */
  template<typename T>
  T getParamVerbose(
    const std::string &name, const T& default_value = T(), const std::string& unit = "", bool* default_used = nullptr,
    ToStringFn<T> value_to_string_fn = &cras::to_string) {

    T value;
    try {
      if (hasParam(name) && filters::FilterBase<F>::getParam(name, value)) {
        if (value_to_string_fn != nullptr) {
          RCLCPP_INFO_STREAM(
            this->logging_interface_->get_logger(), this->getName() << ": Found parameter: " << name <<
            ", value: " << value_to_string_fn(value) << cras::prependIfNonEmpty(unit, " "));
        }
        if (default_used != nullptr) {
          *default_used = false;
        }
        return value;
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR_STREAM(
        this->logging_interface_->get_logger(),
        this->getName() << ": Error getting value of parameter " << name << ": " << e.what());
    }

    if (value_to_string_fn != nullptr) {
      RCLCPP_INFO_STREAM(
        this->logging_interface_->get_logger(), this->getName() << ": Parameter " << name <<
        " not defined, assigning default: " << value_to_string_fn(default_value) << cras::prependIfNonEmpty(unit, " "));
    }
    if (default_used != nullptr) {
      *default_used = true;
    }
    return default_value;
  }

  /** \brief Get the value of the given filter parameter, falling back to the
   *         specified default value, and print out a ROS info/warning message with
   *         the loaded values.
   * \param[in] name Name of the parameter (without the `filterN.params.` prefix).
   * \param[in] default_value The default value to use.
   * \param[in] unit Optional string serving as a [physical/SI] unit of the parameter, just to make the
   *                 messages more informative.
   * \param[out] default_used Whether the default value was used.
   * \param[in] value_to_string_fn Function that converts valid/default values to string (for console logging).
   *                            Set to nullptr to disable logging.
   * \return The loaded param value.
   */
  std::string getParamVerbose(
    const std::string& name, const char* default_value, const std::string& unit = "", bool* default_used = nullptr,
    const ToStringFn<std::string> value_to_string_fn = &cras::to_string) {

    return getParamVerbose(name, std::string(default_value), unit, default_used, value_to_string_fn);
  }

  // getParam specializations for unsigned values

  /** \brief Get the value of the given filter parameter, falling back to the
   *         specified default value, and print out a ROS info/warning message with
   *         the loaded values.
   * \param[in] name Name of the parameter (without the `filterN.params.` prefix).
   * \param[in] default_value The default value to use.
   * \param[in] unit Optional string serving as a [physical/SI] unit of the parameter, just to make the
   *                 messages more informative.
   * \param[out] default_used Whether the default value was used.
   * \param[in] value_to_string_fn Function that converts valid/default values to string (for console logging).
   *                            Set to nullptr to disable logging.
   * \return The loaded param value.
   * \throw std::invalid_argument If the loaded value is negative.
   */
  uint64_t getParamVerbose(
    const std::string& name, const uint64_t& default_value, const std::string& unit = "", bool* default_used = nullptr,
    const ToStringFn<int> value_to_string_fn = &cras::to_string) {

    return getParamUnsigned<uint64_t, int>(name, default_value, unit, default_used, value_to_string_fn);
  }

  // there actually is an unsigned int implementation of FilterBase::getParam,
  // but it doesn't tell you when the passed value is negative - instead it just
  // returns false
  /** \brief Get the value of the given filter parameter, falling back to the
   *         specified default value, and print out a ROS info/warning message with
   *         the loaded values.
   * \param[in] name Name of the parameter (without the `filterN.params.` prefix).
   * \param[in] default_value The default value to use.
   * \param[in] unit Optional string serving as a [physical/SI] unit of the parameter, just to make the
   *                 messages more informative.
   * \param[out] default_used Whether the default value was used.
   * \param[in] value_to_string_fn Function that converts valid/default values to string (for console logging).
   *                            Set to nullptr to disable logging.
   * \return The loaded param value.
   * \throw std::invalid_argument If the loaded value is negative.
   */
  unsigned int getParamVerbose(
    const std::string& name, const unsigned int& default_value, const std::string& unit = "",
    bool* default_used = nullptr, const ToStringFn<int> value_to_string_fn = &cras::to_string) {

    return getParamUnsigned<unsigned int, int>(name, default_value, unit, default_used, value_to_string_fn);
  }

  // ROS types specializations

  /** \brief Get the value of the given filter parameter, falling back to the
   *         specified default value, and print out a ROS info/warning message with
   *         the loaded values.
   * \param[in] name Name of the parameter (without the `filterN.params.` prefix).
   * \param[in] default_value The default value to use.
   * \param[in] unit Optional string serving as a [physical/SI] unit of the parameter, just to make the
   *                 messages more informative.
   * \param[out] default_used Whether the default value was used.
   * \param[in] value_to_string_fn Function that converts valid/default values to string (for console logging).
   *                            Set to nullptr to disable logging.
   * \return The loaded param value.
   */
  rclcpp::Duration getParamDuration(
    const std::string& name, const rclcpp::Duration& default_value, const std::string& unit = "",
    bool* default_used = nullptr, const ToStringFn<double> value_to_string_fn = &cras::to_string) {

    const double temp_value = getParamVerbose(name, default_value.seconds(), unit, default_used, value_to_string_fn);
    return rclcpp::Duration::from_seconds(temp_value);
  }

  /** \brief Get the value of the given filter parameter as a set of strings, falling back to the
   *         specified default value, and print out a ROS info/warning message with
   *         the loaded values.
   * \tparam T Type of the values in the set. Only std::string and double are supported.
   * \param[in] name Name of the parameter (without the `filterN.params.` prefix).
   * \param[in] default_value The default value to use.
   * \param[in] unit Optional string serving as a [physical/SI] unit of the parameter, just to make the
   *                 messages more informative.
   * \param[out] default_used Whether the default value was used.
   * \param[in] value_to_string_fn Function that converts valid/default values to string (for console logging).
   *                            Set to nullptr to disable logging.
   * \return The loaded param value.
   */
  template<typename T>
  std::set<T> getParamVerboseSet(
    const std::string& name, const std::set<T>& default_value = std::set<T>(), const std::string& unit = "",
    bool* default_used = nullptr, const ToStringFn<std::vector<T>> value_to_string_fn = &cras::to_string) {

    std::vector<T> vector(default_value.begin(), default_value.end());
    vector = getParamVerbose(name, vector, unit, default_used, value_to_string_fn);
    return std::set<T>(vector.begin(), vector.end());
  }

  /** \brief Get the value of the given filter parameter as a map with string keys, falling back to the
   *         specified default value, and print out a ROS info/warning message with
   *         the loaded values.
   * \tparam T Type of the values in the map.
   * \tparam MapType Type of the map. Only maps with string keys are expected to be used.
   * \param[in] name Name of the parameter (without the `filterN.params.` prefix).
   * \param[in] default_value The default value to use.
   * \param[in] unit Optional string serving as a [physical/SI] unit of the parameter, just to make the
   *             messages more informative.
   * \param[out] default_used Whether the default value was used.
   * \param[in] value_to_string_fn Function that converts valid/default values to string (for console logging).
   *                            Set to nullptr to disable logging.
   * \return The loaded param value.
   */
  template<typename T, typename MapType = std::map<std::string, T>>
  MapType getParamVerboseMap(
    const std::string& name, const std::map<std::string, T>& default_value = std::map<std::string, T>(),
    const std::string& unit = "", bool* default_used = nullptr,
    const ToStringFn<MapType> value_to_string_fn = &cras::to_string) {

    MapType value;

    std::string prefix = this->param_prefix_ + name + ".";
    auto parameter_value_map = this->params_interface_->get_parameter_overrides();
    for (const auto& [paramName, paramValue] : parameter_value_map) {
      if (!cras::startsWith(paramName, prefix)) {
        continue;
      }
      std::string sub_name = paramName.substr(prefix.length());
      try {
        value[sub_name] = paramValue.template get<T>();
      } catch (const std::exception& e) {
        if constexpr (std::is_same_v<double, T>) {
          try {
            value[sub_name] = static_cast<double>(paramValue.template get<int>());
          } catch (const std::exception&) {
            RCLCPP_ERROR_STREAM(
              this->logging_interface_->get_logger(),
              this->getName() << ": Error getting value of parameter " << paramName << ": " << e.what());
          }
        } else {
          RCLCPP_ERROR_STREAM(
            this->logging_interface_->get_logger(),
            this->getName() << ": Error getting value of parameter " << paramName << ": " << e.what());
        }
      }
    }

    if (value.empty()) {
      value = default_value;
      if (default_used != nullptr) {
        *default_used = true;
      }
      if (value_to_string_fn != nullptr) {
        RCLCPP_INFO_STREAM(
          this->logging_interface_->get_logger(), this->getName() << ": Parameter " << name <<
          " not defined, assigning default: " << value_to_string_fn(default_value) <<
          cras::prependIfNonEmpty(unit, " "));
      }
    } else {
      if (default_used != nullptr) {
        *default_used = false;
      }
      RCLCPP_INFO_STREAM(
        this->logging_interface_->get_logger(), this->getName() << ": Found parameter: " << name <<
        ", value: " << value_to_string_fn(value) << cras::prependIfNonEmpty(unit, " "));
    }

    return value;
  }

private:
  template<typename Result, typename Param>
  Result getParamUnsigned(
    const std::string& name, const Result& default_value, const std::string& unit = "", bool* default_used = nullptr,
    const ToStringFn<Param> value_to_string_fn = &cras::to_string) {

    const Param signed_value = getParamVerbose(
      name, static_cast<Param>(default_value), unit, default_used, value_to_string_fn);
    if (signed_value < 0) {
      if (value_to_string_fn != nullptr) {
        RCLCPP_ERROR_STREAM(
          this->logging_interface_->get_logger(), this->getName() << ": Value " << value_to_string_fn(signed_value) <<
          " of unsigned parameter " << name << " is negative.");
      }
      throw std::invalid_argument(name);
    }
    return static_cast<Result>(signed_value);
  }

  // generic casting getParam()
  template<typename Result, typename Param>
  Result getParamCast(
    const std::string& name, const Param& default_value, const std::string& unit = "", bool* default_used = nullptr,
    const ToStringFn<Param> value_to_string_fn = &cras::to_string) {

    const Param param_value = getParamVerbose(name, default_value, unit, default_used, value_to_string_fn);
    return Result(param_value);
  }
};

}  // namespace robot_body_filter
