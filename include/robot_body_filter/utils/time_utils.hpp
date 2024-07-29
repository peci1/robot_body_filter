#ifndef ROBOT_BODY_FILTER_UTILS_TIME_UTILS_HPP
#define ROBOT_BODY_FILTER_UTILS_TIME_UTILS_HPP

#include <rclcpp/rclcpp.hpp>

namespace robot_body_filter {

/**
 * @brief remainingTime Return remaining time to timeout from the query time.
 * @param query The query time, e.g. of the tf transform.
 * @param timeout Maximum time to wait from the query time onwards.
 * @return The remaining time, or zero duration if the time is negative or ROS
 * time isn't initialized.
 */
rclcpp::Duration remainingTime(rclcpp::Clock clock, const rclcpp::Time &query, const double timeout);

/**
 * @brief remainingTime Return remaining time to timeout from the query time.
 * @param query The query time, e.g. of the tf transform.
 * @param timeout Maximum time to wait from the query time onwards.
 * @return The remaining time, or zero duration if the time is negative or ROS
 * time isn't initialized.
 */
rclcpp::Duration remainingTime(rclcpp::Clock clock, const rclcpp::Time &query, const rclcpp::Duration &timeout);

};

#endif //ROBOT_BODY_FILTER_UTILS_TIME_UTILS_HPP