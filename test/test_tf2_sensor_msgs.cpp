#include "gtest/gtest.h"
#include <robot_body_filter/utils/tf2_sensor_msgs.h>
#include <sensor_msgs/point_cloud2_iterator.h>

TEST(TF2SensorMsgs, CreateFilteredCloudUnorganized)
{
  sensor_msgs::PointCloud2 msg;
  sensor_msgs::PointCloud2Modifier mod(msg);

  mod.setPointCloud2Fields(11,
      "x", 1, sensor_msgs::PointField::FLOAT32,
      "y", 1, sensor_msgs::PointField::FLOAT32,
      "z", 1, sensor_msgs::PointField::FLOAT32,
      "rgb", 1, sensor_msgs::PointField::FLOAT32,
      "vp_x", 1, sensor_msgs::PointField::FLOAT32,
      "vp_y", 1, sensor_msgs::PointField::FLOAT32,
      "vp_z", 1, sensor_msgs::PointField::FLOAT32,
      "normal_x", 1, sensor_msgs::PointField::FLOAT32,
      "normal_y", 1, sensor_msgs::PointField::FLOAT32,
      "normal_z", 1, sensor_msgs::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::PointField::UINT16);

  mod.resize(1);

  sensor_msgs::PointCloud2Iterator<float> it_x(msg, "x");
  sensor_msgs::PointCloud2Iterator<float> it_y(msg, "y");
  sensor_msgs::PointCloud2Iterator<float> it_z(msg, "z");

  sensor_msgs::PointCloud2Iterator<uint8_t> it_rgb(msg, "rgb");
  uint8_t* it_r = &it_rgb[0];
  uint8_t* it_g = &it_rgb[1];
  uint8_t* it_b = &it_rgb[2];

  sensor_msgs::PointCloud2Iterator<float> it_vp_x(msg, "vp_x");
  sensor_msgs::PointCloud2Iterator<float> it_vp_y(msg, "vp_y");
  sensor_msgs::PointCloud2Iterator<float> it_vp_z(msg, "vp_z");

  sensor_msgs::PointCloud2Iterator<float> it_normal_x(msg, "normal_x");
  sensor_msgs::PointCloud2Iterator<float> it_normal_y(msg, "normal_y");
  sensor_msgs::PointCloud2Iterator<float> it_normal_z(msg, "normal_z");

  sensor_msgs::PointCloud2Iterator<uint16_t> it_intensity(msg, "intensity");

  *it_x = *it_vp_x = *it_normal_x = 1;
  *it_y = *it_vp_y = *it_normal_y = 2;
  *it_z = *it_vp_z = *it_normal_z = 3;

  *it_r = 0;
  *it_g = 128;
  *it_b = 255;

  *it_intensity = 10000;

  geometry_msgs::TransformStamped tf;
  tf.header.stamp.sec = 2;
  tf.header.frame_id = "odom";
  tf.child_frame_id = "base_link";
  tf.transform.translation.x = 3;
  tf.transform.translation.y = 2;
  tf.transform.translation.z = 1;
  // rotate 90 deg around y-axis
  tf.transform.rotation.x = tf.transform.rotation.z = 0;
  tf.transform.rotation.y = tf.transform.rotation.w = M_SQRT1_2;

  tf2_ros::Buffer buffer;
  buffer.setTransform(tf, "test");

  msg.header.stamp.sec = 2;
  msg.header.frame_id = "odom";
  sensor_msgs::PointCloud2 out;
  robot_body_filter::transformWithChannels(msg, out, buffer, "base_link");

  ASSERT_EQ(1, out.width);
  ASSERT_EQ(1, out.height);

  sensor_msgs::PointCloud2Iterator<float> it_out_x(out, "x");
  sensor_msgs::PointCloud2Iterator<float> it_out_y(out, "y");
  sensor_msgs::PointCloud2Iterator<float> it_out_z(out, "z");

  sensor_msgs::PointCloud2Iterator<uint8_t> it_out_rgb(out, "rgb");
  uint8_t* it_out_r = &it_out_rgb[0];
  uint8_t* it_out_g = &it_out_rgb[1];
  uint8_t* it_out_b = &it_out_rgb[2];

  sensor_msgs::PointCloud2Iterator<float> it_out_vp_x(out, "vp_x");
  sensor_msgs::PointCloud2Iterator<float> it_out_vp_y(out, "vp_y");
  sensor_msgs::PointCloud2Iterator<float> it_out_vp_z(out, "vp_z");

  sensor_msgs::PointCloud2Iterator<float> it_out_normal_x(out, "normal_x");
  sensor_msgs::PointCloud2Iterator<float> it_out_normal_y(out, "normal_y");
  sensor_msgs::PointCloud2Iterator<float> it_out_normal_z(out, "normal_z");

  sensor_msgs::PointCloud2Iterator<uint16_t> it_out_intensity(out, "intensity");

  EXPECT_NEAR(-2.0, *it_out_x, 1e-6);
  EXPECT_NEAR(0.0, *it_out_y, 1e-6);
  EXPECT_NEAR(-2.0, *it_out_z, 1e-6);

  EXPECT_NEAR(*it_out_x, *it_out_vp_x, 1e-6);
  EXPECT_NEAR(*it_out_y, *it_out_vp_y, 1e-6);
  EXPECT_NEAR(*it_out_z, *it_out_vp_z, 1e-6);

  // should be rotated as vector, not as a point
  EXPECT_NEAR(-3, *it_out_normal_x, 1e-6);
  EXPECT_NEAR(2, *it_out_normal_y, 1e-6);
  EXPECT_NEAR(1, *it_out_normal_z, 1e-6);

  EXPECT_EQ(0, *it_out_r);
  EXPECT_EQ(128, *it_out_g);
  EXPECT_EQ(255, *it_out_b);

  EXPECT_EQ(10000, *it_out_intensity);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}