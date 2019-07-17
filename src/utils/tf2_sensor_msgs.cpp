#include <robot_body_filter/utils/tf2_sensor_msgs.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2_eigen/tf2_eigen.h>

namespace robot_body_filter {

void transformChannel(sensor_msgs::PointCloud2& cloud, const Eigen::Isometry3f& t,
    const std::string& channelPrefix)
{
  sensor_msgs::PointCloud2Iterator<float> x_out(cloud, channelPrefix + "x");
  sensor_msgs::PointCloud2Iterator<float> y_out(cloud, channelPrefix + "y");
  sensor_msgs::PointCloud2Iterator<float> z_out(cloud, channelPrefix + "z");

  Eigen::Vector3f point;
  for (; x_out != x_out.end(); ++x_out, ++y_out, ++z_out) {
    point = t * Eigen::Vector3f(*x_out, *y_out, *z_out);
    *x_out = point.x();
    *y_out = point.y();
    *z_out = point.z();
  }
}

sensor_msgs::PointCloud2& transformWithChannels(
    const sensor_msgs::PointCloud2& in, sensor_msgs::PointCloud2& out,
    tf2_ros::Buffer& tfBuffer, const std::string& targetFrame) {
  bool hasViewPoint = false;
  bool hasNormals = false;
  {
    for (const auto& field: in.fields) {
      if (!hasViewPoint && field.name == "vp_x")
        hasViewPoint = true;
      if (!hasNormals && field.name == "normal_x")
        hasNormals = true;
    }
  }

  tfBuffer.transform(in, out, targetFrame);

  if (!hasViewPoint && !hasNormals)
    return out;

  const auto tf = tfBuffer.lookupTransform(targetFrame, in.header.frame_id, in.header.stamp);
  const auto t = tf2::transformToEigen(tf).cast<float>();

  if (hasViewPoint)
    transformChannel(out, t, "vp_");

  if (hasNormals)
    transformChannel(out, t, "normal_");

  return out;
}

}