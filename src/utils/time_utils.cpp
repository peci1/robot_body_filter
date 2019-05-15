#include "robot_body_filter/utils/time_utils.hpp"

namespace robot_body_filter {

ros::Duration remainingTime(const ros::Time &query, const double timeout)
{
  ros::Time::waitForValid(ros::WallDuration().fromSec(timeout));
  if (!ros::Time::isValid()) {
    ROS_ERROR("ROS time is not yet initialized");
    return ros::Duration(0);
  }

  const auto passed = (ros::Time::now() - query).toSec();
  return ros::Duration(std::max(0.0, timeout - passed));
}

ros::Duration remainingTime(const ros::Time &query,
                            const ros::Duration &timeout)
{
  ros::Time::waitForValid(ros::WallDuration(timeout.sec, timeout.nsec));
  if (!ros::Time::isValid()) {
    ROS_ERROR("ROS time is not yet initialized");
    return ros::Duration(0);
  }

  const auto passed = ros::Time::now() - query;
  const auto remaining = timeout - passed;
  return (remaining.sec >= 0) ? remaining : ros::Duration(0);
}

};