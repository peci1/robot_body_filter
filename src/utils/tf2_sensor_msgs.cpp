#include <robot_body_filter/utils/tf2_sensor_msgs.h>

#include <robot_body_filter/utils/cloud.h>
#include <robot_body_filter/utils/string_utils.hpp>

#include <tf2_eigen/tf2_eigen.h>

#include <Eigen/Geometry>  // needs to be implementation-private as we want -march=native optimizations

#include <unordered_set>

namespace robot_body_filter {

const static std::unordered_map<std::string, CloudChannelType> XYZ_CHANNELS({
  {"", CloudChannelType::POINT},
});

const static std::unordered_map<std::string, CloudChannelType> DEFAULT_CHANNELS({
  {"vp_", CloudChannelType::POINT},
  {"normal_", CloudChannelType::DIRECTION}
});

bool fieldNameMatchesChannel(const std::string& fieldName, const std::string& channelName, CloudChannelType channelType)
{
  if (channelType == CloudChannelType::SCALAR) {
    return fieldName == channelName;
  } else if (channelName.empty()) {
    return fieldName == "x" || fieldName == "y" || fieldName == "z";
  } else {
    return fieldName.length() == channelName.length() + 1 && startsWith(fieldName, channelName) && (
        endsWith(fieldName, "x") || endsWith(fieldName, "y") || endsWith(fieldName, "z"));
  }
}

void transformChannel(const sensor_msgs::PointCloud2& cloudIn, sensor_msgs::PointCloud2& cloudOut,
                      const Eigen::Isometry3f& t, const std::string& channelPrefix, const CloudChannelType type)
{
  if (num_points(cloudIn) == 0)
    return;

  if (type == CloudChannelType::SCALAR)
    return;

  CloudConstIter x_in(cloudIn, channelPrefix + "x");
  CloudConstIter y_in(cloudIn, channelPrefix + "y");
  CloudConstIter z_in(cloudIn, channelPrefix + "z");

  CloudIter x_out(cloudOut, channelPrefix + "x");
  CloudIter y_out(cloudOut, channelPrefix + "y");
  CloudIter z_out(cloudOut, channelPrefix + "z");

  Eigen::Vector3f point;
  // the switch has to be outside the for loop for performance reasons
  switch (type)
  {
    case CloudChannelType::POINT:
      for (; x_in != x_in.end(); ++x_in, ++y_in, ++z_in, ++x_out, ++y_out, ++z_out)
      {
        point = t * Eigen::Vector3f(*x_in, *y_in, *z_in);  // apply the whole transform
        *x_out = point.x();
        *y_out = point.y();
        *z_out = point.z();
      }
      break;
    case CloudChannelType::DIRECTION:
      for (; x_out != x_out.end(); ++x_in, ++y_in, ++z_in, ++x_out, ++y_out, ++z_out)
      {
        point = t.linear() * Eigen::Vector3f(*x_in, *y_in, *z_in);  // apply only rotation
        *x_out = point.x();
        *y_out = point.y();
        *z_out = point.z();
      }
      break;
  }
}

void transformChannel(sensor_msgs::PointCloud2& cloud, const geometry_msgs::Transform& tf,
                      const std::string& channelPrefix, CloudChannelType type)
{
  const auto t = tf2::transformToEigen(tf).cast<float>();
  transformChannel(cloud, cloud, t, channelPrefix, type);
}

sensor_msgs::PointCloud2& transformWithChannels(
    const sensor_msgs::PointCloud2& in, sensor_msgs::PointCloud2& out, const geometry_msgs::TransformStamped& tf)
{
  return transformWithChannels(in, out, tf, DEFAULT_CHANNELS);
}

sensor_msgs::PointCloud2& transformWithChannels(
    const sensor_msgs::PointCloud2& in, sensor_msgs::PointCloud2& out, const geometry_msgs::TransformStamped& tf,
    const std::unordered_map<std::string, CloudChannelType>& channels)
{
  std::unordered_set<std::string> channelsPresent;
  for (const auto& field: in.fields) {
    for (const auto& channelAndType : channels)
    {
      const std::string& channel = channelAndType.first;
      const auto& channelType = channelAndType.second;
      if (channelType != CloudChannelType::SCALAR && fieldNameMatchesChannel(field.name, channel, channelType))
        channelsPresent.insert(channel);
    }
  }

  out = in;
  out.header = tf.header;

  const auto t = tf2::transformToEigen(tf).cast<float>();

  transformChannel(in, out, t, "", CloudChannelType::POINT);
  for (const auto& channel : channelsPresent)
    transformChannel(in, out, t, channel, channels.at(channel));

  return out;
}

sensor_msgs::PointCloud2& transformOnlyChannels(
    const sensor_msgs::PointCloud2& in, sensor_msgs::PointCloud2& out, const geometry_msgs::TransformStamped& tf,
    const std::unordered_map<std::string, CloudChannelType>& channels)
{
  std::unordered_set<std::string> channelsPresent;
  out.point_step = 0;
  for (const auto& field: in.fields) {
    for (const auto& channelAndType : channels)
    {
      const std::string& channel = channelAndType.first;
      const auto& channelType = channelAndType.second;
      if (fieldNameMatchesChannel(field.name, channel, channelType)) {
        channelsPresent.insert(channel);
        out.fields.push_back(field);
        out.fields.back().offset = out.point_step;
        out.point_step += sizeOfPointField(field.datatype);
      }
    }
  }

  out.header = tf.header;
  out.is_dense = in.is_dense;
  out.height = in.height;
  out.width = in.width;
  out.is_bigendian = in.is_bigendian;

  CloudModifier mod(out);
  mod.resize(num_points(in));

  const auto t = tf2::transformToEigen(tf).cast<float>();

  for (const auto& channel : channelsPresent) {
    const auto channelType = channels.at(channel);
    if (channelType != CloudChannelType::SCALAR) {
      transformChannel(in, out, t, channel, channelType);
    } else {
      copyChannelData(in, out, channel);
    }
  }

  return out;
}

sensor_msgs::PointCloud2& transformOnlyXYZ(const sensor_msgs::PointCloud2& in, sensor_msgs::PointCloud2& out,
                                           const geometry_msgs::TransformStamped& tf) {
  return transformOnlyChannels(in, out, tf, XYZ_CHANNELS);
}

sensor_msgs::PointCloud2& transformWithChannels(
    const sensor_msgs::PointCloud2& in, sensor_msgs::PointCloud2& out,
    tf2_ros::Buffer& tfBuffer, const std::string& targetFrame)
{
  return transformWithChannels(in, out, tfBuffer, targetFrame, DEFAULT_CHANNELS);
}

sensor_msgs::PointCloud2& transformWithChannels(
    const sensor_msgs::PointCloud2& in, sensor_msgs::PointCloud2& out,
    tf2_ros::Buffer& tfBuffer, const std::string& targetFrame,
    const std::unordered_map<std::string, CloudChannelType>& channels)
{
  const auto tf = tfBuffer.lookupTransform(targetFrame, in.header.frame_id, in.header.stamp);
  return transformWithChannels(in, out, tf, channels);
}

}