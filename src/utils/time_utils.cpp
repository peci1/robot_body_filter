#include "robot_body_filter/utils/time_utils.hpp"

namespace robot_body_filter {

rclcpp::Duration remainingTime(rclcpp::Clock clock, const rclcpp::Time &query, const double timeout)
{
  // rclcpp::Time::waitForValid(rclcpp::Duration::from_seconds(timeout));
  // if (!rclcpp::Time::isValid()) {
  //   ROS_ERROR("ROS time is not yet initialized");
  //   return rclcpp::Duration(0);
  // }

  const auto passed = (clock.now() - query).seconds();
  return rclcpp::Duration::from_seconds(std::max(0.0, timeout - passed));
}

rclcpp::Duration remainingTime(rclcpp::Clock clock, const rclcpp::Time &query, const rclcpp::Duration &timeout)
{
  // rclcpp::Time::waitForValid(rclcpp::Duration(timeout.seconds(), timeout.nanoseconds()));
  // if (!rclcpp::Time::isValid()) {
  //   // ROS_ERROR("ROS time is not yet initialized");
  //   return rclcpp::Duration(0);
  // }

  const auto passed = clock.now() - query;
  const auto remaining = timeout - passed;
  return (remaining.seconds() >= 0) ? remaining : rclcpp::Duration::from_seconds(0);
}

};