#ifndef ROBOT_BODY_FILTER_TF2_SENSOR_MSGS_H
#define ROBOT_BODY_FILTER_TF2_SENSOR_MSGS_H

#include <unordered_map>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

namespace robot_body_filter
{

enum class CloudChannelType { POINT, DIRECTION, SCALAR };

void transformChannel(sensor_msgs::PointCloud2& cloud, const geometry_msgs::Transform& t,
                      const std::string& channelPrefix, CloudChannelType type);

sensor_msgs::PointCloud2& transformWithChannels(
    const sensor_msgs::PointCloud2& in, sensor_msgs::PointCloud2& out, const geometry_msgs::TransformStamped& tf);

sensor_msgs::PointCloud2& transformWithChannels(
    const sensor_msgs::PointCloud2& in, sensor_msgs::PointCloud2& out, const geometry_msgs::TransformStamped& tf,
    const std::unordered_map<std::string, CloudChannelType>& channels);

sensor_msgs::PointCloud2& transformWithChannels(
    const sensor_msgs::PointCloud2& in, sensor_msgs::PointCloud2& out,
    tf2_ros::Buffer& tfBuffer, const std::string& targetFrame);

sensor_msgs::PointCloud2& transformWithChannels(
    const sensor_msgs::PointCloud2& in, sensor_msgs::PointCloud2& out,
    tf2_ros::Buffer& tfBuffer, const std::string& targetFrame,
    const std::unordered_map<std::string, CloudChannelType>& channels);

sensor_msgs::PointCloud2& transformOnlyChannels(
    const sensor_msgs::PointCloud2& in, sensor_msgs::PointCloud2& out, const geometry_msgs::TransformStamped& tf,
    const std::unordered_map<std::string, CloudChannelType>& channels);

sensor_msgs::PointCloud2& transformOnlyXYZ(
    const sensor_msgs::PointCloud2& in, sensor_msgs::PointCloud2& out, const geometry_msgs::TransformStamped& tf);

}

#endif // ROBOT_BODY_FILTER_TF2_SENSOR_MSGS_H
