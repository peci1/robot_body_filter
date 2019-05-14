#ifndef ROBOT_SELF_FILTER_UTILS_TIME_UTILS_HPP
#define ROBOT_SELF_FILTER_UTILS_TIME_UTILS_HPP

#include <ros/ros.h>

namespace robot_self_filter {

/**
 * @brief remainingTime Return remaining time to timeout from the query time.
 * @param query The query time, e.g. of the tf transform.
 * @param timeout Maximum time to wait from the query time onwards.
 * @return
 */
ros::Duration remainingTime(const ros::Time &query, double timeout);

/**
 * @brief remainingTime Return remaining time to timeout from the query time.
 * @param query The query time, e.g. of the tf transform.
 * @param timeout Maximum time to wait from the query time onwards.
 * @return
 */
ros::Duration remainingTime(const ros::Time &query,
                            const ros::Duration &timeout);

};

#endif //ROBOT_SELF_FILTER_UTILS_TIME_UTILS_HPP