#include "gtest/gtest.h"
#include <robot_body_filter/utils/cloud.h>

TEST(Cloud, NumPoints)
{
  sensor_msgs::PointCloud2 msg;
  sensor_msgs::PointCloud2Modifier mod(msg);

  mod.setPointCloud2FieldsByString(1, "xyz");

  mod.resize(3);
  EXPECT_EQ(3, robot_body_filter::num_points(msg));

  mod.resize(1);
  EXPECT_EQ(1, robot_body_filter::num_points(msg));

  mod.resize(0);
  EXPECT_EQ(0, robot_body_filter::num_points(msg));
}

TEST(Cloud, CreateFilteredCloudUnorganized)
{
  sensor_msgs::PointCloud2 msg;
  sensor_msgs::PointCloud2Modifier mod(msg);

  mod.setPointCloud2FieldsByString(1, "xyz");
  mod.resize(3);

  sensor_msgs::PointCloud2Iterator<float> it_x(msg, "x");
  sensor_msgs::PointCloud2Iterator<float> it_y(msg, "y");
  sensor_msgs::PointCloud2Iterator<float> it_z(msg, "z");

  for (size_t i = 0; i < 3; ++i)
  {
    *it_x = i;
    *it_y = i;
    *it_z = i;

    ++it_x;
    ++it_y;
    ++it_z;
  }

  sensor_msgs::PointCloud2 out;
  CREATE_FILTERED_CLOUD(msg, out, true, *x_it == 2.0)

  EXPECT_EQ(1, robot_body_filter::num_points(out));
  EXPECT_EQ(1, out.width);
  EXPECT_EQ(1, out.height);

  sensor_msgs::PointCloud2ConstIterator<float> out_x(out, "x");
  sensor_msgs::PointCloud2ConstIterator<float> out_y(out, "y");
  sensor_msgs::PointCloud2ConstIterator<float> out_z(out, "z");

  EXPECT_EQ(2.0, *out_x);
  EXPECT_EQ(2.0, *out_y);
  EXPECT_EQ(2.0, *out_z);
}

TEST(Cloud, CreateFilteredCloudOrganized)
{
  sensor_msgs::PointCloud2 msg;
  sensor_msgs::PointCloud2Modifier mod(msg);

  mod.setPointCloud2FieldsByString(1, "xyz");
  mod.resize(4);

  msg.width = 2;
  msg.height = 2;
  msg.row_step = msg.row_step / 2;
  msg.is_dense = false;

  sensor_msgs::PointCloud2Iterator<float> it_x(msg, "x");
  sensor_msgs::PointCloud2Iterator<float> it_y(msg, "y");
  sensor_msgs::PointCloud2Iterator<float> it_z(msg, "z");

  for (size_t i = 0; i < 4; ++i)
  {
    *it_x = i;
    *it_y = i;
    *it_z = i;

    ++it_x;
    ++it_y;
    ++it_z;
  }

  sensor_msgs::PointCloud2 out;
  CREATE_FILTERED_CLOUD(msg, out, true, int(*x_it) % 2 == 0)

  ASSERT_EQ(4, robot_body_filter::num_points(out));
  EXPECT_EQ(2, out.width);
  EXPECT_EQ(2, out.height);
  EXPECT_EQ(false, out.is_dense);

  sensor_msgs::PointCloud2ConstIterator<float> out_x(out, "x");
  sensor_msgs::PointCloud2ConstIterator<float> out_y(out, "y");
  sensor_msgs::PointCloud2ConstIterator<float> out_z(out, "z");

  EXPECT_EQ(0.0, *out_x);
  EXPECT_EQ(0.0, *out_y);
  EXPECT_EQ(0.0, *out_z);

  ++out_x; ++out_y; ++out_z;

  EXPECT_TRUE(std::isnan(*out_x));
  EXPECT_TRUE(std::isnan(*out_y));
  EXPECT_TRUE(std::isnan(*out_z));

  ++out_x; ++out_y; ++out_z;

  EXPECT_EQ(2.0, *out_x);
  EXPECT_EQ(2.0, *out_y);
  EXPECT_EQ(2.0, *out_z);

  ++out_x; ++out_y; ++out_z;

  EXPECT_TRUE(std::isnan(*out_x));
  EXPECT_TRUE(std::isnan(*out_y));
  EXPECT_TRUE(std::isnan(*out_z));
}

TEST(Cloud, CreateFilteredCloudOrganizedDoNotKeep)
{
  sensor_msgs::PointCloud2 msg;
  sensor_msgs::PointCloud2Modifier mod(msg);

  mod.setPointCloud2FieldsByString(1, "xyz");
  mod.resize(4);

  msg.width = 2;
  msg.height = 2;
  msg.row_step = msg.row_step / 2;
  msg.is_dense = false;

  sensor_msgs::PointCloud2Iterator<float> it_x(msg, "x");
  sensor_msgs::PointCloud2Iterator<float> it_y(msg, "y");
  sensor_msgs::PointCloud2Iterator<float> it_z(msg, "z");

  for (size_t i = 0; i < 4; ++i)
  {
    *it_x = i;
    *it_y = i;
    *it_z = i;

    ++it_x;
    ++it_y;
    ++it_z;
  }

  sensor_msgs::PointCloud2 out;
  CREATE_FILTERED_CLOUD(msg, out, false, int(*x_it) % 2 == 0)

  ASSERT_EQ(2, robot_body_filter::num_points(out));
  EXPECT_EQ(2, out.width);
  EXPECT_EQ(1, out.height);
  EXPECT_EQ(true, out.is_dense);

  sensor_msgs::PointCloud2ConstIterator<float> out_x(out, "x");
  sensor_msgs::PointCloud2ConstIterator<float> out_y(out, "y");
  sensor_msgs::PointCloud2ConstIterator<float> out_z(out, "z");

  EXPECT_EQ(0.0, *out_x);
  EXPECT_EQ(0.0, *out_y);
  EXPECT_EQ(0.0, *out_z);

  ++out_x; ++out_y; ++out_z;

  EXPECT_EQ(2.0, *out_x);
  EXPECT_EQ(2.0, *out_y);
  EXPECT_EQ(2.0, *out_z);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}