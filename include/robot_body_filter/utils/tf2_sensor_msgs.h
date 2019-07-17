#ifndef ROBOT_BODY_FILTER_TF2_SENSOR_MSGS_H
#define ROBOT_BODY_FILTER_TF2_SENSOR_MSGS_H

#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

namespace robot_body_filter
{

sensor_msgs::PointCloud2& transformWithChannels(
    const sensor_msgs::PointCloud2& in, sensor_msgs::PointCloud2& out,
    tf2_ros::Buffer& tfBuffer, const std::string& targetFrame);

}

#endif // ROBOT_BODY_FILTER_TF2_SENSOR_MSGS_H
