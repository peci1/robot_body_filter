#include "robot_self_filter/utils/time_utils.hpp"

namespace robot_self_filter {

ros::Duration remainingTime(const ros::Time &query, const double timeout)
{
  const auto passed = (ros::Time::now() - query).toSec();
  return ros::Duration(std::max(0.0, timeout - passed));
}

ros::Duration remainingTime(const ros::Time &query,
                            const ros::Duration &timeout)
{
  const auto passed = ros::Time::now() - query;
  const auto remaining = timeout - passed;
  return (remaining.sec >= 0) ? remaining : ros::Duration(0);
}

};