// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include "gtest/gtest.h"

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <cras_cpp_common/cloud.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <moveit/point_containment_filter/shape_mask.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>
#include <robot_body_filter/msg/oriented_bounding_box_stamped.hpp>
#include <robot_body_filter/msg/sphere_stamped.hpp>
#include <robot_body_filter/RayCastingShapeMask.hpp>
#include <robot_body_filter/RobotBodyFilter.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "utils.cpp"  // NOLINT

using robot_body_filter::msg::OrientedBoundingBoxStamped;
using robot_body_filter::msg::SphereStamped;
using robot_body_filter::RayCastingShapeMask;
using robot_body_filter::ScaleAndPadding;

// This model corresponds to what is in test_ray_casting_shape_mask.blend.
// However, the sizes of the objects are smaller so that they reach the desired
// size after applying scale 1.1 and padding 0.01 (as set in test_robot_body_filter.yaml).
constexpr char ROBOT_URDF[] =
  "<?xml version=\"1.0\" ?>\n"
  "<robot name=\"NIFTi\">\n"
  "  <link name=\"base_link\">\n"
  "    <visual>\n"
  "      <origin rpy=\"0 0 0\" xyz=\"-0.1220 0 0\"/>\n"
  "      <geometry><box size=\"1.8 1.8 1.8\"/></geometry>\n"
  "    </visual>\n"
  "    <collision>\n"
  "      <origin rpy=\"0 0 0\" xyz=\"-0.1220 0 0\"/>\n"
  "      <geometry><box size=\"1.8 1.8 1.8\"/></geometry>\n"
  "    </collision>\n"
  "    <collision name=\"big_collision_box\">\n"
  "      <origin rpy=\"0 0 0\" xyz=\"-0.1 0 0\"/>\n"
  "      <geometry><box size=\"2.2545 2.2545 2.2545\"/></geometry>\n"
  "    </collision>\n"
  "    <inertial>\n"
  "      <origin rpy=\"0 0 0\" xyz=\"-0.034 0 0.142\"/>\n"
  "      <mass value=\"6.0\"/>\n"
  "      <inertia ixx=\"0.001\" ixy=\"0.0001\" ixz=\"0.0\" iyy=\"0.02\" iyz=\"-0.0001\" izz=\"0.03\"/>\n"
  "    </inertial>\n"
  "  </link>\n"
  "  <link name=\"antenna\">\n"
  "    <visual>\n"
  "      <origin rpy=\"0 0 0\" xyz=\"-0.01864 0 0\"/>\n"
  "      <geometry><sphere radius=\"1.24091\" /></geometry>\n"
  "    </visual>\n"
  "    <collision>\n"
  "      <origin rpy=\"0 0 0\" xyz=\"-0.01864 0 0\"/>\n"
  "      <geometry><sphere radius=\"1.24091\"/></geometry>\n"
  "    </collision>\n"
  "    <inertial>\n"
  "      <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n"
  "      <mass value=\"0.5\"/>\n"
  "      <inertia ixx=\"0.005\" ixy=\"0\" ixz=\"0\" iyy=\"0.001\" iyz=\"-0.0001\" izz=\"0.004\"/>\n"
  "    </inertial>\n"
  "  </link>\n"
  "  <joint name=\"antenna_j\" type=\"fixed\">\n"
  "    <parent link=\"base_link\"/>\n"
  "    <child link=\"antenna\"/>\n"
  "    <origin rpy=\"0 0 0\" xyz=\"0.01864 0 0\"/>\n"
  "  </joint>\n"
  "  <link name=\"laser_base\">\n"
  "    <visual>\n"
  "      <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n"
  "      <geometry><box size=\"0.0005 0.0005 0.0005\"/></geometry>\n"
  "    </visual>\n"
  "    <inertial>\n"
  "      <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n"
  "      <mass value=\"1e-5\"/>\n"
  "      <inertia ixx=\"1e-3\" ixy=\"1e-6\" ixz=\"1e-6\" iyy=\"1e-3\" iyz=\"1e-6\" izz=\"1e-3\"/>\n"
  "    </inertial>\n"
  "  </link>\n"
  "  <joint name=\"laser_base_j\" type=\"fixed\">\n"
  "    <parent link=\"base_link\"/>\n"
  "    <child link=\"laser_base\"/>\n"
  "    <origin rpy=\"0 0 0\" xyz=\"-1.5 0 0\"/>\n"
  "  </joint>\n"
  "  <link name=\"laser\">\n"
  "    <visual>\n"
  "      <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n"
  "      <geometry><box size=\"0.07273 0.07273 0.07273\"/></geometry>\n"
  "    </visual>\n"
  "    <collision>\n"
  "      <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n"
  "      <geometry><box size=\"0.07273 0.07273 0.07273\"/></geometry>\n"
  "    </collision>\n"
  "    <inertial>\n"
  "      <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n"
  "      <mass value=\"1\"/>\n"
  "      <inertia ixx=\"0.003\" ixy=\"0\" ixz=\"0\" iyy=\"0.003\" iyz=\"0\" izz=\"0.002\"/>\n"
  "    </inertial>\n"
  "  </link>\n"
  "  <joint name=\"laser_j\" type=\"revolute\">\n"
  "    <parent link=\"laser_base\"/>\n"
  "    <child link=\"laser\"/>\n"
  "    <axis xyz=\"1 0 0\"/>\n"
  "    <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n"
  "    <limit effort=\"0\" lower=\"-2.3561945\" upper=\"2.3561945\" velocity=\"4\"/>\n"
  "  </joint>\n"
  "</robot>";

class RobotBodyFilterLaserScanTest : public robot_body_filter::RobotBodyFilterLaserScan {
public:
  explicit RobotBodyFilterLaserScanTest(rclcpp::Node::SharedPtr node) : node_(std::move(node)) {
    fail_without_robot_description_ = true;
  }

  ~RobotBodyFilterLaserScanTest() override {
    // this prevents spurious SIGABRTs caused probably by some too fast cleanup
    // after tf_frames_watchdog_, or I don't know what...
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  friend class RobotBodyFilter_InitFromArray_Test;
  friend class RobotBodyFilter_InitFromDict_Test;
  friend class RobotBodyFilter_LoadParams_Test;
  friend class RobotBodyFilter_LoadParamsAllConfig_Test;
  friend class RobotBodyFilter_ParseRobot_Test;
  friend class RobotBodyFilter_Transforms_Test;
  friend class RobotBodyFilter_ComputeMaskPointByPoint_Test;
  friend class RobotBodyFilter_UpdateLaserScan_Test;

protected:
  RequiredInterfaces createNodeInterfaces() override {
    if (node_ != nullptr) {
      return *node_;
    }
    return RobotBodyFilterLaserScan::createNodeInterfaces();
  }

private:
  rclcpp::Node::SharedPtr node_;
};

class RobotBodyFilterPointCloud2Test : public robot_body_filter::RobotBodyFilterPointCloud2 {
public:
  explicit RobotBodyFilterPointCloud2Test(rclcpp::Node::SharedPtr node) : node_(std::move(node)) {
    fail_without_robot_description_ = true;
  }

  ~RobotBodyFilterPointCloud2Test() override {
    // this prevents spurious SIGABRTs caused probably by some too fast cleanup
    // after tf_frames_watchdog_, or I don't know what...
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }

  friend class RobotBodyFilter_ComputeMaskAllAtOnce_Test;
  friend class RobotBodyFilter_UpdatePointCloud2_Test;

protected:
  RequiredInterfaces createNodeInterfaces() override {
    if (node_ != nullptr) {
      return *node_;
    }
    return RobotBodyFilterPointCloud2::createNodeInterfaces();
  }

private:
  rclcpp::Node::SharedPtr node_;
};

TEST(RobotBodyFilter, InitFromDict) {
  const auto nh = std::make_shared<rclcpp::Node>("test_chain_config");

  const auto filter = std::make_shared<RobotBodyFilterLaserScanTest>(nh);
  const auto filter_base = std::dynamic_pointer_cast<filters::FilterBase<sensor_msgs::msg::LaserScan>>(filter);

  // test that invalid robot model doesn't throw any exception, but also generates no filter shapes
  filter_base->configure(
    "filter1.params", "test_dict_config", nh->get_node_logging_interface(), nh->get_node_parameters_interface());

  const std_msgs::msg::String::SharedPtr msg(new std_msgs::msg::String);
  msg->data = "<robot name='test'></robot>";
  filter->onRobotModelMsg(msg);

  EXPECT_TRUE(filter->hasModel());
  EXPECT_EQ("test_robot_description", filter->robot_description_topic_);
  EXPECT_EQ(0, filter->shapes_to_links_.size());
  EXPECT_TRUE(filter->configured_);
  EXPECT_GT(rclcpp::Duration::from_seconds(0.1), nh->get_clock()->now() - filter->time_configured_);
  ASSERT_NE(nullptr, filter->tf_frames_watchdog_);
  EXPECT_TRUE(filter->tf_frames_watchdog_->isMonitored("laser"));  // Issue #6, monitor sensor frame
}

TEST(RobotBodyFilter, LoadParams) {
  const auto nh = std::make_shared<rclcpp::Node>("test_chain_config");

  const auto filter = std::make_shared<RobotBodyFilterLaserScanTest>(nh);
  const auto filter_base = std::dynamic_pointer_cast<filters::FilterBase<sensor_msgs::msg::LaserScan>>(filter);

  filter_base->configure(
    "filter1.params", "test_chain_config", nh->get_node_logging_interface(), nh->get_node_parameters_interface());

  // test that invalid robot model doesn't throw any exception, but also generates no filter shapes
  const std_msgs::msg::String::SharedPtr msg(new std_msgs::msg::String);
  msg->data = "<robot name='test'></robot>";
  filter->onRobotModelMsg(msg);

  EXPECT_EQ("odom", filter->fixed_frame_);
  EXPECT_EQ("laser", filter->sensor_frame_);
  EXPECT_EQ("odom", filter->filtering_frame_);
  EXPECT_TRUE(filter->keep_clouds_organized_);
  EXPECT_FLOAT_EQ(0.002, filter->model_pose_update_interval_.seconds());
  EXPECT_TRUE(filter->point_by_point_scan_);
  EXPECT_FLOAT_EQ(0.1, filter->min_distance_);
  EXPECT_FLOAT_EQ(10.0, filter->max_distance_);
  EXPECT_EQ(std::set<std::string>({"antenna", "base_link::big_collision_box"}),
            filter->links_ignored_in_bounding_sphere_);
  EXPECT_EQ(std::set<std::string>({"laser", "base_link::big_collision_box"}),
            filter->links_ignored_in_shadow_test_);
  EXPECT_TRUE(filter->links_ignored_in_bounding_box_.empty());
  EXPECT_TRUE(filter->links_ignored_in_contains_test_.empty());
  EXPECT_TRUE(filter->links_ignored_everywhere_.empty());
  EXPECT_TRUE(filter->only_links_.empty());
  EXPECT_DOUBLE_EQ(1.1, filter->default_contains_inflation_.scale);
  EXPECT_DOUBLE_EQ(0.01, filter->default_contains_inflation_.padding);
  EXPECT_DOUBLE_EQ(1.1, filter->default_shadow_inflation_.scale);
  EXPECT_DOUBLE_EQ(0.01, filter->default_shadow_inflation_.padding);
  EXPECT_EQ(
    (std::map<std::string, ScaleAndPadding>({
      {"*::big_collision_box", ScaleAndPadding(2.0, 0.01)},
      {"base_link", ScaleAndPadding(1.1, 0.05)},
      {"antenna", ScaleAndPadding(1.2, 0.01)},
      })),
    filter->per_link_contains_inflation_);
  EXPECT_EQ(
    (std::map<std::string, ScaleAndPadding>({
      {"*::big_collision_box", ScaleAndPadding(3.0, 0.01)},
      {"base_link", ScaleAndPadding(1.1, 0.05)},
      {"laser", ScaleAndPadding(1.1, 0.015)},
      })),
    filter->per_link_shadow_inflation_);
  EXPECT_EQ("test_robot_description", filter->robot_description_topic_);
  EXPECT_DOUBLE_EQ(60.0, filter->tf_buffer_length_.seconds());
  EXPECT_DOUBLE_EQ(0.2, filter->reachable_transform_timeout_.seconds());
  EXPECT_DOUBLE_EQ(0.2, filter->unreachable_transform_timeout_.seconds());
  EXPECT_TRUE(filter->compute_bounding_sphere_);
  EXPECT_FALSE(filter->compute_debug_bounding_sphere_);
  EXPECT_FALSE(filter->publish_bounding_sphere_marker_);
  EXPECT_FALSE(filter->publish_no_bounding_sphere_pointcloud_);
  EXPECT_FALSE(filter->compute_bounding_box_);
  EXPECT_FALSE(filter->compute_debug_bounding_box_);
  EXPECT_FALSE(filter->publish_bounding_box_marker_);
  EXPECT_FALSE(filter->publish_no_bounding_box_pointcloud_);
  EXPECT_FALSE(filter->compute_oriented_bounding_box_);
  EXPECT_FALSE(filter->compute_debug_oriented_bounding_box_);
  EXPECT_FALSE(filter->publish_oriented_bounding_box_marker_);
  EXPECT_FALSE(filter->publish_no_oriented_bounding_box_pointcloud_);
  EXPECT_TRUE(filter->compute_local_bounding_box_);
  EXPECT_FALSE(filter->compute_debug_local_bounding_box_);
  EXPECT_FALSE(filter->publish_local_bounding_box_marker_);
  EXPECT_FALSE(filter->publish_no_local_bounding_box_pointcloud_);
  EXPECT_EQ("base_link", filter->local_bounding_box_frame_);
  EXPECT_FALSE(filter->publish_debug_pcl_inside_);
  EXPECT_FALSE(filter->publish_debug_pcl_clip_);
  EXPECT_FALSE(filter->publish_debug_pcl_shadow_);
  EXPECT_FALSE(filter->publish_debug_contains_marker_);
  EXPECT_FALSE(filter->publish_debug_shadow_marker_);
  EXPECT_FALSE(filter->require_all_frames_reachable_);

  EXPECT_STREQ("/robot_bounding_sphere", filter->bounding_sphere_publisher_->get_topic_name());
  // EXPECT_STREQ("", filter->bounding_box_publisher_->get_topic_name());
  // EXPECT_STREQ("", filter->oriented_bounding_box_publisher_->get_topic_name());
  EXPECT_STREQ("/robot_local_bounding_box", filter->local_bounding_box_publisher_->get_topic_name());
  // EXPECT_STREQ("", filter->bounding_sphere_marker_publisher_->get_topic_name());
  // EXPECT_STREQ("", filter->bounding_box_marker_publisher_->get_topic_name());
  // EXPECT_STREQ("", filter->oriented_bounding_box_marker_publisher_->get_topic_name());
  // EXPECT_STREQ("", filter->local_bounding_box_marker_publisher_->get_topic_name());
  // EXPECT_STREQ("", filter->bounding_sphere_debug_marker_publisher_->get_topic_name());
  // EXPECT_STREQ("", filter->bounding_box_debug_marker_publisher_->get_topic_name());
  // EXPECT_STREQ("", filter->oriented_bounding_box_debug_marker_publisher_->get_topic_name());
  // EXPECT_STREQ("", filter->local_bounding_box_debug_marker_publisher_->get_topic_name());
  // EXPECT_STREQ("", filter->scan_point_cloud_no_bounding_sphere_publisher_->get_topic_name());
  // EXPECT_STREQ("", filter->scan_point_cloud_no_bounding_box_publisher_->get_topic_name());
  // EXPECT_STREQ("", filter->scan_point_cloud_no_oriented_bounding_box_publisher_->get_topic_name());
  // EXPECT_STREQ("", filter->scan_point_cloud_no_local_bounding_box_publisher_->get_topic_name());
  // EXPECT_STREQ("", filter->debug_point_cloud_inside_publisher_->get_topic_name());
  // EXPECT_STREQ("", filter->debug_point_cloud_clip_publisher_->get_topic_name());
  // EXPECT_STREQ("", filter->debug_point_cloud_shadow_publisher_->get_topic_name());
  // EXPECT_STREQ("", filter->debug_contains_marker_publisher_->get_topic_name());
  // EXPECT_STREQ("", filter->debug_shadow_marker_publisher_->get_topic_name());

  EXPECT_EQ(std::string("/") + nh->get_name() + "/reload_model",
            filter->reload_robot_model_service_server_->get_service_name());
}

TEST(RobotBodyFilter, LoadParamsAllConfig) {
  const auto nh = std::make_shared<rclcpp::Node>("all_config");

  const auto filter = std::make_shared<RobotBodyFilterLaserScanTest>(nh);
  const auto filter_base = std::dynamic_pointer_cast<filters::FilterBase<sensor_msgs::msg::LaserScan>>(filter);

  filter_base->configure(
    "filter1.params", "all_config", nh->get_node_logging_interface(), nh->get_node_parameters_interface());

  // test that invalid robot model doesn't throw any exception, but also generates no filter shapes
  const std_msgs::msg::String::SharedPtr msg(new std_msgs::msg::String);
  msg->data = "<robot name='test'></robot>";
  filter->onRobotModelMsg(msg);

  EXPECT_EQ("odom", filter->fixed_frame_);
  EXPECT_EQ("laser", filter->sensor_frame_);
  EXPECT_EQ("base_link", filter->filtering_frame_);
  EXPECT_TRUE(filter->keep_clouds_organized_);
  EXPECT_FLOAT_EQ(0.002, filter->model_pose_update_interval_.seconds());
  EXPECT_TRUE(filter->point_by_point_scan_);
  EXPECT_FLOAT_EQ(0.1, filter->min_distance_);
  EXPECT_FLOAT_EQ(10.0, filter->max_distance_);
  EXPECT_EQ(std::set<std::string>({"antenna", "base_link::big_collision_box"}),
            filter->links_ignored_in_bounding_sphere_);
  EXPECT_EQ(std::set<std::string>({"laser", "base_link::big_collision_box"}),
            filter->links_ignored_in_shadow_test_);
  EXPECT_EQ(std::set<std::string>({"base_link"}), filter->links_ignored_in_bounding_box_);
  EXPECT_EQ(std::set<std::string>({"base_link"}), filter->links_ignored_in_contains_test_);
  EXPECT_EQ(std::set<std::string>({"base_link"}), filter->links_ignored_everywhere_);
  EXPECT_EQ(std::set<std::string>({"laser"}), filter->only_links_);
  EXPECT_DOUBLE_EQ(1.1, filter->default_contains_inflation_.scale);
  EXPECT_DOUBLE_EQ(0.01, filter->default_contains_inflation_.padding);
  EXPECT_DOUBLE_EQ(1.1, filter->default_shadow_inflation_.scale);
  EXPECT_DOUBLE_EQ(0.01, filter->default_shadow_inflation_.padding);
  EXPECT_EQ("test_robot_description", filter->robot_description_topic_);
  EXPECT_DOUBLE_EQ(60.0, filter->tf_buffer_length_.seconds());
  EXPECT_DOUBLE_EQ(0.2, filter->reachable_transform_timeout_.seconds());
  EXPECT_DOUBLE_EQ(0.2, filter->unreachable_transform_timeout_.seconds());
  EXPECT_TRUE(filter->compute_bounding_sphere_);
  EXPECT_TRUE(filter->compute_debug_bounding_sphere_);
  EXPECT_TRUE(filter->publish_bounding_sphere_marker_);
  EXPECT_TRUE(filter->publish_no_bounding_sphere_pointcloud_);
  EXPECT_TRUE(filter->compute_bounding_box_);
  EXPECT_TRUE(filter->compute_debug_bounding_box_);
  EXPECT_TRUE(filter->publish_bounding_box_marker_);
  EXPECT_TRUE(filter->publish_no_bounding_box_pointcloud_);
  EXPECT_TRUE(filter->compute_oriented_bounding_box_);
  EXPECT_TRUE(filter->compute_debug_oriented_bounding_box_);
  EXPECT_TRUE(filter->publish_oriented_bounding_box_marker_);
  EXPECT_TRUE(filter->publish_no_oriented_bounding_box_pointcloud_);
  EXPECT_TRUE(filter->compute_local_bounding_box_);
  EXPECT_TRUE(filter->compute_debug_local_bounding_box_);
  EXPECT_TRUE(filter->publish_local_bounding_box_marker_);
  EXPECT_TRUE(filter->publish_no_local_bounding_box_pointcloud_);
  EXPECT_EQ("base_link", filter->local_bounding_box_frame_);
  EXPECT_TRUE(filter->publish_debug_pcl_inside_);
  EXPECT_TRUE(filter->publish_debug_pcl_clip_);
  EXPECT_TRUE(filter->publish_debug_pcl_shadow_);
  EXPECT_TRUE(filter->publish_debug_contains_marker_);
  EXPECT_TRUE(filter->publish_debug_shadow_marker_);
  EXPECT_TRUE(filter->require_all_frames_reachable_);

  EXPECT_STREQ("/robot_bounding_sphere", filter->bounding_sphere_publisher_->get_topic_name());
  EXPECT_STREQ("/robot_bounding_box", filter->bounding_box_publisher_->get_topic_name());
  EXPECT_STREQ("/robot_oriented_bounding_box", filter->oriented_bounding_box_publisher_->get_topic_name());
  EXPECT_STREQ("/robot_local_bounding_box", filter->local_bounding_box_publisher_->get_topic_name());
  EXPECT_STREQ("/robot_bounding_sphere_marker", filter->bounding_sphere_marker_publisher_->get_topic_name());
  EXPECT_STREQ("/robot_bounding_box_marker", filter->bounding_box_marker_publisher_->get_topic_name());
  EXPECT_STREQ("/robot_oriented_bounding_box_marker",
               filter->oriented_bounding_box_marker_publisher_->get_topic_name());
  EXPECT_STREQ("/robot_local_bounding_box_marker", filter->local_bounding_box_marker_publisher_->get_topic_name());
  EXPECT_STREQ("/robot_bounding_sphere_debug", filter->bounding_sphere_debug_marker_publisher_->get_topic_name());
  EXPECT_STREQ("/robot_bounding_box_debug", filter->bounding_box_debug_marker_publisher_->get_topic_name());
  EXPECT_STREQ("/robot_oriented_bounding_box_debug",
               filter->oriented_bounding_box_debug_marker_publisher_->get_topic_name());
  EXPECT_STREQ("/robot_local_bounding_box_debug", filter->local_bounding_box_debug_marker_publisher_->get_topic_name());
  EXPECT_STREQ("/scan_point_cloud_no_bsphere",
               filter->scan_point_cloud_no_bounding_sphere_publisher_->get_topic_name());
  EXPECT_STREQ("/scan_point_cloud_no_bbox", filter->scan_point_cloud_no_bounding_box_publisher_->get_topic_name());
  EXPECT_STREQ("/scan_point_cloud_no_oriented_bbox",
               filter->scan_point_cloud_no_oriented_bounding_box_publisher_->get_topic_name());
  EXPECT_STREQ("/scan_point_cloud_no_local_bbox",
               filter->scan_point_cloud_no_local_bounding_box_publisher_->get_topic_name());
  EXPECT_STREQ("/scan_point_cloud_inside", filter->debug_point_cloud_inside_publisher_->get_topic_name());
  EXPECT_STREQ("/scan_point_cloud_clip", filter->debug_point_cloud_clip_publisher_->get_topic_name());
  EXPECT_STREQ("/scan_point_cloud_shadow", filter->debug_point_cloud_shadow_publisher_->get_topic_name());
  EXPECT_STREQ("/robot_model_for_contains_test", filter->debug_contains_marker_publisher_->get_topic_name());
  EXPECT_STREQ("/robot_model_for_shadow_test", filter->debug_shadow_marker_publisher_->get_topic_name());

  EXPECT_EQ(std::string("/") + nh->get_name() + "/reload_model",
            filter->reload_robot_model_service_server_->get_service_name());
}

TEST(RobotBodyFilter, ParseRobot) {
  const auto nh = std::make_shared<rclcpp::Node>("test_chain_config");

  const auto filter = std::make_shared<RobotBodyFilterLaserScanTest>(nh);
  const auto filter_base = std::dynamic_pointer_cast<filters::FilterBase<sensor_msgs::msg::LaserScan>>(filter);

  filter_base->configure(
    "filter1.params", "test_chain_config", nh->get_node_logging_interface(), nh->get_node_parameters_interface());

  const std_msgs::msg::String::SharedPtr msg(new std_msgs::msg::String);
  msg->data = ROBOT_URDF;
  filter->onRobotModelMsg(msg);

  // base_link, base_link::big_collision_box::contains/shadow, laser::contains/shadow, antenna::contains/shadow
  EXPECT_EQ(7, filter->shapes_to_links_.size());
  EXPECT_TRUE(filter->tf_frames_watchdog_->isMonitored("laser"));
  EXPECT_TRUE(filter->tf_frames_watchdog_->isMonitored("base_link"));
  EXPECT_EQ(7, filter->shape_mask_->getBodies().size());
  EXPECT_EQ(4, filter->shape_mask_->getBodiesForContainsTest().size());
  EXPECT_EQ(2, filter->shape_mask_->getBodiesForShadowTest().size());

  filter->clearRobotMask();
  EXPECT_EQ(0, filter->shapes_to_links_.size());
  EXPECT_FALSE(filter->tf_frames_watchdog_->isMonitored("laser"));
  EXPECT_FALSE(filter->tf_frames_watchdog_->isMonitored("base_link"));
  EXPECT_EQ(0, filter->shape_mask_->getBodies().size());
  EXPECT_EQ(0, filter->shape_mask_->getBodiesForContainsTest().size());
  EXPECT_EQ(0, filter->shape_mask_->getBodiesForShadowTest().size());

  filter->addRobotMaskFromUrdf(ROBOT_URDF);
  EXPECT_EQ(7, filter->shapes_to_links_.size());
  EXPECT_TRUE(filter->tf_frames_watchdog_->isMonitored("laser"));
  EXPECT_TRUE(filter->tf_frames_watchdog_->isMonitored("base_link"));
  EXPECT_EQ(7, filter->shape_mask_->getBodies().size());
  EXPECT_EQ(4, filter->shape_mask_->getBodiesForContainsTest().size());
  EXPECT_EQ(2, filter->shape_mask_->getBodiesForShadowTest().size());

  std_msgs::msg::String::SharedPtr msg2(new std_msgs::msg::String);
  msg2->data = "<robot name='test'></robot>";
  filter->onRobotModelMsg(msg2);
  auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto resp = std::make_shared<std_srvs::srv::Trigger::Response>();
  filter->triggerModelReload(nullptr, req, resp);
  EXPECT_TRUE(resp->success);

  EXPECT_EQ(0, filter->shapes_to_links_.size());
  EXPECT_TRUE(filter->tf_frames_watchdog_->isMonitored("laser"));  // sensor frame is always monitored
  EXPECT_FALSE(filter->tf_frames_watchdog_->isMonitored("base_link"));
  EXPECT_EQ(0, filter->shape_mask_->getBodies().size());
  EXPECT_EQ(0, filter->shape_mask_->getBodiesForContainsTest().size());
  EXPECT_EQ(0, filter->shape_mask_->getBodiesForShadowTest().size());

  filter->onRobotModelMsg(msg2);
  EXPECT_EQ(0, filter->shapes_to_links_.size());
  EXPECT_TRUE(filter->tf_frames_watchdog_->isMonitored("laser"));
  EXPECT_FALSE(filter->tf_frames_watchdog_->isMonitored("base_link"));
  EXPECT_EQ(0, filter->shape_mask_->getBodies().size());
  EXPECT_EQ(0, filter->shape_mask_->getBodiesForContainsTest().size());
  EXPECT_EQ(0, filter->shape_mask_->getBodiesForShadowTest().size());

  // test reconfiguring (this happens when playing back a bag file and a new one starts playing)
  filter->clearRobotMask();
  filter->onRobotModelMsg(msg);
  filter_base->configure(
    "filter1.params", "test_chain_config", nh->get_node_logging_interface(), nh->get_node_parameters_interface());
  EXPECT_EQ(7, filter->shapes_to_links_.size());
  EXPECT_TRUE(filter->tf_frames_watchdog_->isMonitored("laser"));
  EXPECT_TRUE(filter->tf_frames_watchdog_->isMonitored("base_link"));
  EXPECT_EQ(7, filter->shape_mask_->getBodies().size());
  EXPECT_EQ(4, filter->shape_mask_->getBodiesForContainsTest().size());
  EXPECT_EQ(2, filter->shape_mask_->getBodiesForShadowTest().size());
}

TEST(RobotBodyFilter, Transforms) {
  const auto nh = std::make_shared<rclcpp::Node>("test_chain_config");

  const auto filter = std::make_shared<RobotBodyFilterLaserScanTest>(nh);
  const auto filter_base = std::dynamic_pointer_cast<filters::FilterBase<sensor_msgs::msg::LaserScan>>(filter);

  filter_base->configure(
    "filter1.params", "test_chain_config", nh->get_node_logging_interface(), nh->get_node_parameters_interface());

  const std_msgs::msg::String::SharedPtr msg(new std_msgs::msg::String);
  msg->data = ROBOT_URDF;
  filter->onRobotModelMsg(msg);

  geometry_msgs::msg::TransformStamped tf;
  tf.transform.rotation.w = 1.0;
  for (double d = -5.0; d < 5.0; d += 0.1) {
    tf.header.stamp = nh->get_clock()->now() + rclcpp::Duration::from_seconds(d);

    tf.transform.translation.x = 1;
    tf.header.frame_id = "odom";
    tf.child_frame_id = "base_link";
    filter->tf_buffer_->setTransform(tf, "test");

    tf.transform.translation.x = -1.5;
    tf.header.frame_id = "base_link";
    tf.child_frame_id = "laser";
    filter->tf_buffer_->setTransform(tf, "test");

    tf.transform.translation.x = 0.01864;
    tf.header.frame_id = "base_link";
    tf.child_frame_id = "antenna";
    filter->tf_buffer_->setTransform(tf, "test");
  }

  for (double d = 25.0; d < 35.0; d += 0.1) {
    tf.header.stamp = nh->get_clock()->now() + rclcpp::Duration::from_seconds(d);

    tf.transform.translation.x = 11;
    tf.header.frame_id = "odom";
    tf.child_frame_id = "base_link";
    filter->tf_buffer_->setTransform(tf, "test");

    tf.transform.translation.x = -8.5;
    tf.header.frame_id = "base_link";
    tf.child_frame_id = "laser";
    filter->tf_buffer_->setTransform(tf, "test");

    tf.transform.translation.x = 10.01864;
    tf.header.frame_id = "base_link";
    tf.child_frame_id = "antenna";
    filter->tf_buffer_->setTransform(tf, "test");
  }

  while (!filter->tf_frames_watchdog_->isReachable("antenna")) {
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }
  filter->updateTransformCache(nh->get_clock()->now(), nh->get_clock()->now() + rclcpp::Duration::from_seconds(30));

  std::map<std::string, point_containment_filter::ShapeHandle> shapes;
  for (const auto& [shape, link] : filter->shapes_to_links_) {
    shapes[link.cache_key] = shape;
  }

  ASSERT_NE(shapes.end(), shapes.find("base_link-0"));
  ASSERT_NE(shapes.end(), shapes.find("base_link-1"));
  ASSERT_NE(shapes.end(), shapes.find("antenna-0"));
  ASSERT_NE(shapes.end(), shapes.find("laser-0"));

  Eigen::Isometry3d transform;

  filter->cache_lookup_between_scans_ratio_ = 0.0;
  // the positions do not exactly correspond to the ones from URDF; instead, they're composed of the
  // transforms defined above and the offsets of the collision shapes from their links' origins
  ASSERT_TRUE(filter->getShapeTransform(shapes["antenna-0"], transform));
  EXPECT_NEAR(1.0 - 0.01864 + 0.01864, transform.translation().x(), 1e-6);
  ASSERT_TRUE(filter->getShapeTransform(shapes["laser-0"], transform));
  EXPECT_NEAR(1 - 1.5, transform.translation().x(), 1e-6);
  ASSERT_TRUE(filter->getShapeTransform(shapes["base_link-0"], transform));
  EXPECT_NEAR(1.0 - 0.1220, transform.translation().x(), 1e-6);
  ASSERT_TRUE(filter->getShapeTransform(shapes["base_link-1"], transform));
  EXPECT_NEAR(1.0 - 0.1, transform.translation().x(), 1e-6);

  filter->cache_lookup_between_scans_ratio_ = 1.0;
  ASSERT_TRUE(filter->getShapeTransform(shapes["antenna-0"], transform));
  EXPECT_NEAR(11 - 0.01864 + 10.01864, transform.translation().x(), 1e-6);
  ASSERT_TRUE(filter->getShapeTransform(shapes["laser-0"], transform));
  EXPECT_NEAR(11 - 8.5, transform.translation().x(), 1e-6);
  ASSERT_TRUE(filter->getShapeTransform(shapes["base_link-0"], transform));
  EXPECT_NEAR(11 - 0.1220, transform.translation().x(), 1e-6);
  ASSERT_TRUE(filter->getShapeTransform(shapes["base_link-1"], transform));
  EXPECT_NEAR(11 - 0.1, transform.translation().x(), 1e-6);
}

TEST(RobotBodyFilter, ComputeMaskPointByPoint) {
  const auto nh = std::make_shared<rclcpp::Node>("compute_mask_config_point_by_point");

  const auto filter = std::make_shared<RobotBodyFilterLaserScanTest>(nullptr);
  const auto filter_base = std::dynamic_pointer_cast<filters::FilterBase<sensor_msgs::msg::LaserScan>>(filter);

  filter_base->configure(
    "filter1.params", "compute_mask_config_point_by_point",
    nh->get_node_logging_interface(), nh->get_node_parameters_interface());

  std_msgs::msg::String::SharedPtr msg(new std_msgs::msg::String);
  msg->data = ROBOT_URDF;
  filter->onRobotModelMsg(msg);

  cras::Cloud cloud;
  cloud.header.frame_id = filter->filtering_frame_;
  cras::CloudModifier mod(cloud);
  mod.setPointCloud2Fields(
    7,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "vp_x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "vp_y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "vp_z", 1, sensor_msgs::msg::PointField::FLOAT32,
    "stamps", 1, sensor_msgs::msg::PointField::FLOAT32);
  mod.resize(11);

  {
    cras::CloudIter x_it(cloud, "x");
    cras::CloudIter y_it(cloud, "y");
    cras::CloudIter z_it(cloud, "z");
    cras::CloudIter vp_x_it(cloud, "vp_x");
    cras::CloudIter vp_y_it(cloud, "vp_y");
    cras::CloudIter vp_z_it(cloud, "vp_z");
    cras::CloudIter stamps_it(cloud, "stamps");

    // pointSensor
    *x_it = -1.5; *y_it = 0; *z_it = 0; *vp_x_it = -1.5; *vp_y_it = 0; *vp_z_it = 0; *stamps_it = 0;
    ++x_it, ++y_it, ++z_it, ++vp_x_it, ++vp_y_it, ++vp_z_it, ++stamps_it;
    // pointSensor2
    *x_it = -1.47; *y_it = 0; *z_it = 0; *vp_x_it = -1.5; *vp_y_it = 0; *vp_z_it = 0; *stamps_it = 0;
    ++x_it, ++y_it, ++z_it, ++vp_x_it, ++vp_y_it, ++vp_z_it, ++stamps_it;
    // pointClipMin
    *x_it = -1.42; *y_it = 0; *z_it = 0; *vp_x_it = -1.5; *vp_y_it = 0; *vp_z_it = 0; *stamps_it = 0;
    ++x_it, ++y_it, ++z_it, ++vp_x_it, ++vp_y_it, ++vp_z_it, ++stamps_it;
    // pointClipMax
    *x_it = 10; *y_it = 0; *z_it = 0; *vp_x_it = -1.5; *vp_y_it = 0; *vp_z_it = 0; *stamps_it = 0;
    ++x_it, ++y_it, ++z_it, ++vp_x_it, ++vp_y_it, ++vp_z_it, ++stamps_it;
    // pointInBox
    *x_it = 0.85; *y_it = 0.85; *z_it = 0.85; *vp_x_it = -1.5; *vp_y_it = 0; *vp_z_it = 0; *stamps_it = 0;
    ++x_it, ++y_it, ++z_it, ++vp_x_it, ++vp_y_it, ++vp_z_it, ++stamps_it;
    // pointInSphere
    *x_it = 1.35; *y_it = 0; *z_it = 0; *vp_x_it = -1.5; *vp_y_it = 0; *vp_z_it = 0; *stamps_it = 0;
    ++x_it, ++y_it, ++z_it, ++vp_x_it, ++vp_y_it, ++vp_z_it, ++stamps_it;
    // pointInBoth
    *x_it = 0; *y_it = 0; *z_it = 0; *vp_x_it = -1.5; *vp_y_it = 0; *vp_z_it = 0; *stamps_it = 0;
    ++x_it, ++y_it, ++z_it, ++vp_x_it, ++vp_y_it, ++vp_z_it, ++stamps_it;
    // pointShadowBox
    *x_it = -0.25; *y_it = -2; *z_it = 2; *vp_x_it = -1.5; *vp_y_it = 0; *vp_z_it = 0; *stamps_it = 0;
    ++x_it, ++y_it, ++z_it, ++vp_x_it, ++vp_y_it, ++vp_z_it, ++stamps_it;
    // pointShadowSphere
    *x_it = -0.560762; *y_it = 0; *z_it = 1.83871; *vp_x_it = -1.5; *vp_y_it = 0; *vp_z_it = 0; *stamps_it = 0;
    ++x_it, ++y_it, ++z_it, ++vp_x_it, ++vp_y_it, ++vp_z_it, ++stamps_it;
    // pointShadowBoth
    *x_it = 1.5; *y_it = 0; *z_it = 0; *vp_x_it = -1.5; *vp_y_it = 0; *vp_z_it = 0; *stamps_it = 0;
    ++x_it, ++y_it, ++z_it, ++vp_x_it, ++vp_y_it, ++vp_z_it, ++stamps_it;
    // pointOutside
    *x_it = -3; *y_it = 0; *z_it = 0; *vp_x_it = -1.5; *vp_y_it = 0; *vp_z_it = 0; *stamps_it = 0;
    ++x_it, ++y_it, ++z_it, ++vp_x_it, ++vp_y_it, ++vp_z_it, ++stamps_it;
  }

  {
    rclcpp::Time now = nh->get_clock()->now();
    cloud.header.stamp = now;
    geometry_msgs::msg::TransformStamped tf;
    tf.transform.rotation.w = 1.0;
    for (double d = -5.0; d < 5.0; d += 0.1) {
      tf.header.stamp = now + rclcpp::Duration::from_seconds(d);

      tf.transform.translation.x = 0.122;
      tf.header.frame_id = "odom";
      tf.child_frame_id = "base_link";
      filter->tf_buffer_->setTransform(tf, "test");

      tf.transform.translation.x = -1.5 - 0.122;
      tf.header.frame_id = "base_link";
      tf.child_frame_id = "laser";
      filter->tf_buffer_->setTransform(tf, "test");

      tf.transform.translation.x = 0.01864 - 0.122;
      tf.header.frame_id = "base_link";
      tf.child_frame_id = "antenna";
      filter->tf_buffer_->setTransform(tf, "test");
    }
  }

  while (!filter->tf_frames_watchdog_->isReachable("antenna")) {
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }
  filter->updateTransformCache(cloud.header.stamp, cloud.header.stamp + rclcpp::Duration::from_seconds(1));

  std::map<std::string, point_containment_filter::ShapeHandle> shapes;
  for (const auto& [shape, link] : filter->shapes_to_links_) {
    shapes[link.cache_key] = shape;
  }

  ASSERT_NE(shapes.end(), shapes.find("base_link-0"));
  ASSERT_NE(shapes.end(), shapes.find("base_link-1"));
  ASSERT_NE(shapes.end(), shapes.find("antenna-0"));
  ASSERT_NE(shapes.end(), shapes.find("laser-0"));

  // we reconstruct the scene from test_ray_casting_shape_mask.blend
  Eigen::Isometry3d transform;
  filter->cache_lookup_between_scans_ratio_ = 0.0;
  ASSERT_TRUE(filter->getShapeTransform(shapes["antenna-0"], transform));
  EXPECT_NEAR(0, transform.translation().x(), 1e-6);
  ASSERT_TRUE(filter->getShapeTransform(shapes["laser-0"], transform));
  EXPECT_NEAR(-1.5, transform.translation().x(), 1e-6);
  ASSERT_TRUE(filter->getShapeTransform(shapes["base_link-0"], transform));
  EXPECT_NEAR(0, transform.translation().x(), 1e-6);
  ASSERT_TRUE(filter->getShapeTransform(shapes["base_link-1"], transform));
  EXPECT_NEAR(0.022, transform.translation().x(), 1e-6);

  std::vector<RayCastingShapeMask::MaskValue> mask;
  filter->computeMask(cloud, mask);

  ASSERT_EQ(cras::numPoints(cloud), mask.size());
  EXPECT_EQ(RayCastingShapeMask::MaskValue::CLIP, mask[0]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::CLIP, mask[1]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::CLIP, mask[2]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::CLIP, mask[3]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::INSIDE, mask[4]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::INSIDE, mask[5]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::INSIDE, mask[6]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, mask[7]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, mask[8]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, mask[9]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, mask[10]);

  // move some points by 5 meters, and some by 10 meters in x and change the relative timestamps so
  // that the 5-meter motion is 10 seconds in future and the 10-meter motion is 20 seconds
  // this will test the filter->cache_lookup_between_scans_ratio_ usage
  {
    cras::CloudIter x_it(cloud, "x");
    cras::CloudIter y_it(cloud, "y");
    cras::CloudIter z_it(cloud, "z");
    cras::CloudIter vp_x_it(cloud, "vp_x");
    cras::CloudIter vp_y_it(cloud, "vp_y");
    cras::CloudIter vp_z_it(cloud, "vp_z");
    cras::CloudIter stamps_it(cloud, "stamps");

    // pointSensor
    *x_it = -1.5; *y_it = 0; *z_it = 0; *vp_x_it = -1.5; *vp_y_it = 0; *vp_z_it = 0; *stamps_it = 0;
    ++x_it, ++y_it, ++z_it, ++vp_x_it, ++vp_y_it, ++vp_z_it, ++stamps_it;
    // pointSensor2
    *x_it = -1.47; *y_it = 0; *z_it = 0; *vp_x_it = -1.5; *vp_y_it = 0; *vp_z_it = 0; *stamps_it = 0;
    ++x_it, ++y_it, ++z_it, ++vp_x_it, ++vp_y_it, ++vp_z_it, ++stamps_it;
    // pointClipMin
    *x_it = -1.42; *y_it = 0; *z_it = 0; *vp_x_it = -1.5; *vp_y_it = 0; *vp_z_it = 0; *stamps_it = 0;
    ++x_it, ++y_it, ++z_it, ++vp_x_it, ++vp_y_it, ++vp_z_it, ++stamps_it;
    // pointClipMax
    *x_it = 10; *y_it = 0; *z_it = 0; *vp_x_it = -1.5; *vp_y_it = 0; *vp_z_it = 0; *stamps_it = 0;
    ++x_it, ++y_it, ++z_it, ++vp_x_it, ++vp_y_it, ++vp_z_it, ++stamps_it;
    // pointInBox
    *x_it = 5.85; *y_it = 0.85; *z_it = 0.85; *vp_x_it = 3.5; *vp_y_it = 0; *vp_z_it = 0; *stamps_it = 10;
    ++x_it, ++y_it, ++z_it, ++vp_x_it, ++vp_y_it, ++vp_z_it, ++stamps_it;
    // pointInSphere
    *x_it = 6.35; *y_it = 0; *z_it = 0; *vp_x_it = 3.5; *vp_y_it = 0; *vp_z_it = 0; *stamps_it = 10;
    ++x_it, ++y_it, ++z_it, ++vp_x_it, ++vp_y_it, ++vp_z_it, ++stamps_it;
    // pointInBoth
    *x_it = 5; *y_it = 0; *z_it = 0; *vp_x_it = 3.5; *vp_y_it = 0; *vp_z_it = 0; *stamps_it = 10;
    ++x_it, ++y_it, ++z_it, ++vp_x_it, ++vp_y_it, ++vp_z_it, ++stamps_it;
    // pointShadowBox
    *x_it = 5 - 0.25; *y_it = -2; *z_it = 2; *vp_x_it = 3.5; *vp_y_it = 0; *vp_z_it = 0; *stamps_it = 10;
    ++x_it, ++y_it, ++z_it, ++vp_x_it, ++vp_y_it, ++vp_z_it, ++stamps_it;
    // pointShadowSphere
    *x_it = 10 - 0.560762; *y_it = 0; *z_it = 1.83871; *vp_x_it = 8.5; *vp_y_it = 0; *vp_z_it = 0; *stamps_it = 20;
    ++x_it, ++y_it, ++z_it, ++vp_x_it, ++vp_y_it, ++vp_z_it, ++stamps_it;
    // pointShadowBoth
    *x_it = 11.5; *y_it = 0; *z_it = 0; *vp_x_it = 8.5; *vp_y_it = 0; *vp_z_it = 0; *stamps_it = 20;
    ++x_it, ++y_it, ++z_it, ++vp_x_it, ++vp_y_it, ++vp_z_it, ++stamps_it;
    // pointOutside
    *x_it = 7; *y_it = 0; *z_it = 0; *vp_x_it = 8.5; *vp_y_it = 0; *vp_z_it = 0; *stamps_it = 20;
    ++x_it, ++y_it, ++z_it, ++vp_x_it, ++vp_y_it, ++vp_z_it, ++stamps_it;
  }

  {
    geometry_msgs::msg::TransformStamped tf;
    rclcpp::Time now = nh->get_clock()->now();
    cloud.header.stamp = now;
    tf.transform.rotation.w = 1;
    for (double d = -5.0; d < 5.0; d += 0.1) {
      tf.header.stamp = now + rclcpp::Duration::from_seconds(d);

      tf.transform.translation.x = 0.122;
      tf.header.frame_id = "odom";
      tf.child_frame_id = "base_link";
      ASSERT_TRUE(filter->tf_buffer_->setTransform(tf, "test"));

      tf.transform.translation.x = -1.5 - 0.122;
      tf.header.frame_id = "base_link";
      tf.child_frame_id = "laser";
      ASSERT_TRUE(filter->tf_buffer_->setTransform(tf, "test"));

      tf.transform.translation.x = 0.01864 - 0.122;
      tf.header.frame_id = "base_link";
      tf.child_frame_id = "antenna";
      ASSERT_TRUE(filter->tf_buffer_->setTransform(tf, "test"));

      tf.header.stamp = rclcpp::Time(tf.header.stamp) + rclcpp::Duration::from_seconds(20);

      tf.transform.translation.x = 10.122;
      tf.header.frame_id = "odom";
      tf.child_frame_id = "base_link";
      ASSERT_TRUE(filter->tf_buffer_->setTransform(tf, "test"));

      tf.transform.translation.x = -1.5 - 0.122;
      tf.header.frame_id = "base_link";
      tf.child_frame_id = "laser";
      ASSERT_TRUE(filter->tf_buffer_->setTransform(tf, "test"));

      tf.transform.translation.x = 0.01864 - 0.122;
      tf.header.frame_id = "base_link";
      tf.child_frame_id = "antenna";
      ASSERT_TRUE(filter->tf_buffer_->setTransform(tf, "test"));
    }
  }
  filter->updateTransformCache(cloud.header.stamp, cloud.header.stamp + rclcpp::Duration::from_seconds(20));

  SphereStamped::ConstSharedPtr bounding_sphere;
  geometry_msgs::msg::PolygonStamped::ConstSharedPtr bounding_box;
  OrientedBoundingBoxStamped::ConstSharedPtr oriented_bounding_box;
  geometry_msgs::msg::PolygonStamped::ConstSharedPtr local_bounding_box;
  visualization_msgs::msg::Marker::ConstSharedPtr bounding_sphere_marker;
  visualization_msgs::msg::Marker::ConstSharedPtr bounding_box_marker;
  visualization_msgs::msg::Marker::ConstSharedPtr oriented_bounding_box_marker;
  visualization_msgs::msg::Marker::ConstSharedPtr local_bounding_box_marker;
  visualization_msgs::msg::MarkerArray::ConstSharedPtr bounding_sphere_debug_marker;
  visualization_msgs::msg::MarkerArray::ConstSharedPtr bounding_box_debug_marker;
  visualization_msgs::msg::MarkerArray::ConstSharedPtr oriented_bounding_box_debug_marker;
  visualization_msgs::msg::MarkerArray::ConstSharedPtr local_bounding_box_debug_marker;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr pcl_no_bounding_sphere;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr pcl_no_bounding_box;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr pcl_no_oriented_bounding_box;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr pcl_no_local_bounding_box;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr pcl_inside;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr pcl_clip;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr pcl_shadow;
  visualization_msgs::msg::MarkerArray::ConstSharedPtr robot_model_contains_test;
  visualization_msgs::msg::MarkerArray::ConstSharedPtr robot_model_shadow_test;

  auto bounding_sphereSubscriber = nh->create_subscription<SphereStamped>(
    "/robot_bounding_sphere", 10,
    [&](const SphereStamped::ConstSharedPtr& m){bounding_sphere = m;});
  auto bounding_boxSubscriber = nh->create_subscription<geometry_msgs::msg::PolygonStamped>(
    "/robot_bounding_box", 10,
    [&](const geometry_msgs::msg::PolygonStamped::ConstSharedPtr& m){bounding_box = m;});
  auto oriented_bounding_boxSubscriber = nh->create_subscription<OrientedBoundingBoxStamped>(
    "/robot_oriented_bounding_box", 10,
    [&](const OrientedBoundingBoxStamped::ConstSharedPtr& m){oriented_bounding_box = m;});
  auto local_bounding_boxSubscriber = nh->create_subscription<geometry_msgs::msg::PolygonStamped>(
    "/robot_local_bounding_box", 10,
    [&](const geometry_msgs::msg::PolygonStamped::ConstSharedPtr& m){local_bounding_box = m;});
  auto bounding_sphere_markerSubscriber = nh->create_subscription<visualization_msgs::msg::Marker>(
    "/robot_bounding_sphere_marker", 10,
    [&](const visualization_msgs::msg::Marker::ConstSharedPtr& m){bounding_sphere_marker = m;});
  auto bounding_box_markerSubscriber = nh->create_subscription<visualization_msgs::msg::Marker>(
    "/robot_bounding_box_marker", 10,
    [&](const visualization_msgs::msg::Marker::ConstSharedPtr& m){bounding_box_marker = m;});
  auto oriented_bounding_box_markerSubscriber = nh->create_subscription<visualization_msgs::msg::Marker>(
    "/robot_oriented_bounding_box_marker", 10,
    [&](const visualization_msgs::msg::Marker::ConstSharedPtr& m){oriented_bounding_box_marker = m;});
  auto local_bounding_box_markerSubscriber = nh->create_subscription<visualization_msgs::msg::Marker>(
    "/robot_local_bounding_box_marker", 10,
    [&](const visualization_msgs::msg::Marker::ConstSharedPtr& m){local_bounding_box_marker = m;});
  auto bounding_sphere_debug_markerSubscriber = nh->create_subscription<visualization_msgs::msg::MarkerArray>(
    "/robot_bounding_sphere_debug", 10,
    [&](const visualization_msgs::msg::MarkerArray::ConstSharedPtr& m){bounding_sphere_debug_marker = m;});
  auto bounding_box_debug_markerSubscriber = nh->create_subscription<visualization_msgs::msg::MarkerArray>(
    "/robot_bounding_box_debug", 10,
    [&](const visualization_msgs::msg::MarkerArray::ConstSharedPtr& m){bounding_box_debug_marker = m;});
  auto oriented_bounding_box_debug_markerSubscriber = nh->create_subscription<visualization_msgs::msg::MarkerArray>(
    "/robot_oriented_bounding_box_debug", 10,
    [&](const visualization_msgs::msg::MarkerArray::ConstSharedPtr& m){oriented_bounding_box_debug_marker = m;});
  auto local_bounding_box_debug_markerSubscriber = nh->create_subscription<visualization_msgs::msg::MarkerArray>(
    "/robot_local_bounding_box_debug", 10,
    [&](const visualization_msgs::msg::MarkerArray::ConstSharedPtr& m){local_bounding_box_debug_marker = m;});
  auto scanPointCloudNoBoundingSphereSubscriber = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/scan_point_cloud_no_bsphere", 10,
    [&](const sensor_msgs::msg::PointCloud2::ConstSharedPtr& m){pcl_no_bounding_sphere = m;});
  auto scanPointCloudNoBoundingBoxSubscriber = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/scan_point_cloud_no_bbox", 10,
    [&](const sensor_msgs::msg::PointCloud2::ConstSharedPtr& m){pcl_no_bounding_box = m;});
  auto scanPointCloudNoOrientedBoundingBoxSubscriber = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/scan_point_cloud_no_oriented_bbox", 10,
    [&](const sensor_msgs::msg::PointCloud2::ConstSharedPtr& m){pcl_no_oriented_bounding_box = m;});
  auto scanPointCloudNoLocalBoundingBoxSubscriber = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/scan_point_cloud_no_local_bbox", 10,
    [&](const sensor_msgs::msg::PointCloud2::ConstSharedPtr& m){pcl_no_local_bounding_box = m;});
  auto debugPointCloudInsideSubscriber = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/scan_point_cloud_inside", 10,
    [&](const sensor_msgs::msg::PointCloud2::ConstSharedPtr& m){pcl_inside = m;});
  auto debugPointCloudClipSubscriber = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/scan_point_cloud_clip", 10,
    [&](const sensor_msgs::msg::PointCloud2::ConstSharedPtr& m){pcl_clip = m;});
  auto debugPointCloudShadowSubscriber = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/scan_point_cloud_shadow", 10,
    [&](const sensor_msgs::msg::PointCloud2::ConstSharedPtr& m){pcl_shadow = m;});
  auto debugContainsMarkerSubscriber = nh->create_subscription<visualization_msgs::msg::MarkerArray>(
    "/robot_model_for_contains_test", 10,
    [&](const visualization_msgs::msg::MarkerArray::ConstSharedPtr& m){robot_model_contains_test = m;});
  auto debugShadowMarkerSubscriber = nh->create_subscription<visualization_msgs::msg::MarkerArray>(
    "/robot_model_for_shadow_test", 10,
    [&](const visualization_msgs::msg::MarkerArray::ConstSharedPtr& m){robot_model_shadow_test = m;});

  filter->computeMask(cloud, mask);

  ASSERT_EQ(cras::numPoints(cloud), mask.size());
  EXPECT_EQ(RayCastingShapeMask::MaskValue::CLIP, mask[0]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::CLIP, mask[1]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::CLIP, mask[2]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::CLIP, mask[3]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::INSIDE, mask[4]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::INSIDE, mask[5]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::INSIDE, mask[6]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, mask[7]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, mask[8]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, mask[9]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, mask[10]);

  WAIT_FOR_MESSAGE(nh, bounding_sphere)
  WAIT_FOR_MESSAGE(nh, bounding_box)
  WAIT_FOR_MESSAGE(nh, oriented_bounding_box)
  WAIT_FOR_MESSAGE(nh, local_bounding_box)
  WAIT_FOR_MESSAGE(nh, bounding_sphere_marker)
  WAIT_FOR_MESSAGE(nh, bounding_box_marker)
  WAIT_FOR_MESSAGE(nh, oriented_bounding_box_marker)
  WAIT_FOR_MESSAGE(nh, local_bounding_box_marker)
  WAIT_FOR_MESSAGE(nh, bounding_sphere_debug_marker)
  WAIT_FOR_MESSAGE(nh, bounding_box_debug_marker)
  WAIT_FOR_MESSAGE(nh, oriented_bounding_box_debug_marker)
  WAIT_FOR_MESSAGE(nh, local_bounding_box_debug_marker)
  WAIT_FOR_MESSAGE(nh, pcl_no_bounding_sphere)
  WAIT_FOR_MESSAGE(nh, pcl_no_bounding_box)
  WAIT_FOR_MESSAGE(nh, pcl_no_oriented_bounding_box)
  WAIT_FOR_MESSAGE(nh, pcl_no_local_bounding_box)
  WAIT_FOR_MESSAGE(nh, pcl_inside)
  WAIT_FOR_MESSAGE(nh, pcl_clip)
  WAIT_FOR_MESSAGE(nh, pcl_shadow)
  WAIT_FOR_MESSAGE(nh, robot_model_contains_test)
  WAIT_FOR_MESSAGE(nh, robot_model_shadow_test)

  EXPECT_EQ("odom", bounding_sphere->header.frame_id);
  EXPECT_EQ(cloud.header.stamp, bounding_sphere->header.stamp);
  EXPECT_NEAR(sqrt(3) * 0.92, bounding_sphere->sphere.radius, 1e-6);
  EXPECT_NEAR(0.0, bounding_sphere->sphere.center.x, 1e-6);
  EXPECT_DOUBLE_EQ(0, bounding_sphere->sphere.center.y);
  EXPECT_DOUBLE_EQ(0, bounding_sphere->sphere.center.z);
  EXPECT_EQ("odom", bounding_sphere_marker->header.frame_id);
  EXPECT_EQ(cloud.header.stamp, bounding_sphere_marker->header.stamp);
  EXPECT_NEAR(2 * sqrt(3) * 0.92, bounding_sphere_marker->scale.x, 1e-6);
  EXPECT_NEAR(2 * sqrt(3) * 0.92, bounding_sphere_marker->scale.y, 1e-6);
  EXPECT_NEAR(2 * sqrt(3) * 0.92, bounding_sphere_marker->scale.z, 1e-6);
  EXPECT_NEAR(0, bounding_sphere_marker->pose.position.x, 1e-6);
  EXPECT_NEAR(0, bounding_sphere_marker->pose.position.y, 1e-6);
  EXPECT_NEAR(0, bounding_sphere_marker->pose.position.z, 1e-6);
  EXPECT_NEAR(0, bounding_sphere_marker->pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, bounding_sphere_marker->pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, bounding_sphere_marker->pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, bounding_sphere_marker->pose.orientation.w, 1e-6);
  EXPECT_LT(0, bounding_sphere_marker->color.a);
  // make sure the marker has some color
  EXPECT_LT(0, bounding_sphere_marker->color.r + bounding_sphere_marker->color.g + bounding_sphere_marker->color.b);
  EXPECT_EQ(visualization_msgs::msg::Marker::SPHERE, bounding_sphere_marker->type);
  EXPECT_EQ(visualization_msgs::msg::Marker::ADD, bounding_sphere_marker->action);
  EXPECT_EQ("bounding_sphere", bounding_sphere_marker->ns);
  EXPECT_EQ(1, bounding_sphere_marker->frame_locked);

  EXPECT_EQ("odom", bounding_box->header.frame_id);
  EXPECT_EQ(cloud.header.stamp, bounding_box->header.stamp);
  ASSERT_EQ(2, bounding_box->polygon.points.size());
  EXPECT_NEAR(-1.55, bounding_box->polygon.points[0].x, 1e-5);
  EXPECT_NEAR(-1.375, bounding_box->polygon.points[0].y, 1e-6);
  EXPECT_NEAR(-1.375, bounding_box->polygon.points[0].z, 1e-6);
  EXPECT_NEAR(1.375, bounding_box->polygon.points[1].x, 1e-5);
  EXPECT_NEAR(1.375, bounding_box->polygon.points[1].y, 1e-6);
  EXPECT_NEAR(1.375, bounding_box->polygon.points[1].z, 1e-6);
  EXPECT_EQ("odom", bounding_box_marker->header.frame_id);
  EXPECT_EQ(cloud.header.stamp, bounding_box_marker->header.stamp);
  EXPECT_NEAR(2.925, bounding_box_marker->scale.x, 1e-5);
  EXPECT_NEAR(2.75, bounding_box_marker->scale.y, 1e-5);
  EXPECT_NEAR(2.75, bounding_box_marker->scale.z, 1e-5);
  EXPECT_NEAR(-0.0875, bounding_box_marker->pose.position.x, 1e-6);
  EXPECT_NEAR(0, bounding_box_marker->pose.position.y, 1e-6);
  EXPECT_NEAR(0, bounding_box_marker->pose.position.z, 1e-6);
  EXPECT_NEAR(0, bounding_box_marker->pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, bounding_box_marker->pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, bounding_box_marker->pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, bounding_box_marker->pose.orientation.w, 1e-6);
  EXPECT_LT(0, bounding_box_marker->color.a);
  // make sure the marker has some color
  EXPECT_LT(0, bounding_box_marker->color.r + bounding_box_marker->color.g + bounding_box_marker->color.b);
  EXPECT_EQ(visualization_msgs::msg::Marker::CUBE, bounding_box_marker->type);
  EXPECT_EQ(visualization_msgs::msg::Marker::ADD, bounding_box_marker->action);
  EXPECT_EQ("bounding_box", bounding_box_marker->ns);
  EXPECT_EQ(1, bounding_box_marker->frame_locked);

  EXPECT_EQ("odom", oriented_bounding_box->header.frame_id);
  EXPECT_EQ(cloud.header.stamp, oriented_bounding_box->header.stamp);
  EXPECT_NEAR(2.925, oriented_bounding_box->obb.extents.x, 1e-5);
  EXPECT_NEAR(2.75, oriented_bounding_box->obb.extents.y, 1e-5);
  EXPECT_NEAR(2.75, oriented_bounding_box->obb.extents.z, 1e-5);
  EXPECT_NEAR(-0.0875, oriented_bounding_box->obb.pose.translation.x, 1e-5);
  EXPECT_NEAR(0, oriented_bounding_box->obb.pose.translation.y, 1e-6);
  EXPECT_NEAR(0, oriented_bounding_box->obb.pose.translation.z, 1e-6);
  EXPECT_NEAR(0, oriented_bounding_box->obb.pose.rotation.x, 1e-6);
  EXPECT_NEAR(0, oriented_bounding_box->obb.pose.rotation.y, 1e-6);
  EXPECT_NEAR(0, oriented_bounding_box->obb.pose.rotation.z, 1e-6);
  EXPECT_NEAR(1, oriented_bounding_box->obb.pose.rotation.w, 1e-6);
  EXPECT_EQ("odom", oriented_bounding_box_marker->header.frame_id);
  EXPECT_EQ(cloud.header.stamp, oriented_bounding_box_marker->header.stamp);
  EXPECT_NEAR(2.925, oriented_bounding_box_marker->scale.x, 1e-5);
  EXPECT_NEAR(2.75, oriented_bounding_box_marker->scale.y, 1e-5);
  EXPECT_NEAR(2.75, oriented_bounding_box_marker->scale.z, 1e-5);
  EXPECT_NEAR(-0.0875, oriented_bounding_box_marker->pose.position.x, 1e-6);
  EXPECT_NEAR(0, oriented_bounding_box_marker->pose.position.y, 1e-6);
  EXPECT_NEAR(0, oriented_bounding_box_marker->pose.position.z, 1e-6);
  EXPECT_NEAR(0, oriented_bounding_box_marker->pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, oriented_bounding_box_marker->pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, oriented_bounding_box_marker->pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, oriented_bounding_box_marker->pose.orientation.w, 1e-6);
  EXPECT_LT(0, oriented_bounding_box_marker->color.a);
  // make sure the marker has some color
  EXPECT_LT(
    0,
    oriented_bounding_box_marker->color.r + oriented_bounding_box_marker->color.g +
    oriented_bounding_box_marker->color.b);
  EXPECT_EQ(visualization_msgs::msg::Marker::CUBE, oriented_bounding_box_marker->type);
  EXPECT_EQ(visualization_msgs::msg::Marker::ADD, oriented_bounding_box_marker->action);
  EXPECT_EQ("oriented_bounding_box", oriented_bounding_box_marker->ns);
  EXPECT_EQ(1, oriented_bounding_box_marker->frame_locked);

  EXPECT_EQ("base_link", local_bounding_box->header.frame_id);
  EXPECT_EQ(cloud.header.stamp, local_bounding_box->header.stamp);
  ASSERT_EQ(2, local_bounding_box->polygon.points.size());
  EXPECT_NEAR(-1.55 - 0.122, local_bounding_box->polygon.points[0].x, 1e-5);
  EXPECT_NEAR(-1.375, local_bounding_box->polygon.points[0].y, 1e-6);
  EXPECT_NEAR(-1.375, local_bounding_box->polygon.points[0].z, 1e-6);
  EXPECT_NEAR(1.375 - 0.122, local_bounding_box->polygon.points[1].x, 1e-5);
  EXPECT_NEAR(1.375, local_bounding_box->polygon.points[1].y, 1e-6);
  EXPECT_NEAR(1.375, local_bounding_box->polygon.points[1].z, 1e-6);
  EXPECT_EQ("base_link", local_bounding_box_marker->header.frame_id);
  EXPECT_EQ(cloud.header.stamp, local_bounding_box_marker->header.stamp);
  EXPECT_NEAR(2.925, local_bounding_box_marker->scale.x, 1e-5);
  EXPECT_NEAR(2.75, local_bounding_box_marker->scale.y, 1e-5);
  EXPECT_NEAR(2.75, local_bounding_box_marker->scale.z, 1e-5);
  EXPECT_NEAR(-0.0875 - 0.122, local_bounding_box_marker->pose.position.x, 1e-6);
  EXPECT_NEAR(0, local_bounding_box_marker->pose.position.y, 1e-6);
  EXPECT_NEAR(0, local_bounding_box_marker->pose.position.z, 1e-6);
  EXPECT_NEAR(0, local_bounding_box_marker->pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, local_bounding_box_marker->pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, local_bounding_box_marker->pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, local_bounding_box_marker->pose.orientation.w, 1e-6);
  EXPECT_LT(0, local_bounding_box_marker->color.a);
  // make sure the marker has some color
  EXPECT_LT(
    0,
    local_bounding_box_marker->color.r + local_bounding_box_marker->color.g + local_bounding_box_marker->color.b);
  EXPECT_EQ(visualization_msgs::msg::Marker::CUBE, local_bounding_box_marker->type);
  EXPECT_EQ(visualization_msgs::msg::Marker::ADD, local_bounding_box_marker->action);
  EXPECT_EQ("local_bounding_box", local_bounding_box_marker->ns);
  EXPECT_EQ(1, local_bounding_box_marker->frame_locked);

  ASSERT_EQ(8, cras::numPoints(*pcl_no_bounding_sphere));
  cras::CloudConstIter x_it(*pcl_no_bounding_sphere, "x");
  // EXPECT_NEAR(-1.5, *x_it, 1e-6); ++x_it;  // pointSensor
  // EXPECT_NEAR(-1.47, *x_it, 1e-6); ++x_it;  // pointSensor2
  // EXPECT_NEAR(-1.42, *x_it, 1e-6); ++x_it;  // pointClipMin
  EXPECT_NEAR(10, *x_it, 1e-6); ++x_it;  // pointClipMax
  EXPECT_NEAR(5.85, *x_it, 1e-6); ++x_it;  // pointInBox
  EXPECT_NEAR(6.35, *x_it, 1e-6); ++x_it;  // pointInSphere
  EXPECT_NEAR(5, *x_it, 1e-6); ++x_it;  // pointInBoth
  EXPECT_NEAR(5 - 0.25, *x_it, 1e-6); ++x_it;  // pointShadowBox
  EXPECT_NEAR(10 - 0.560762, *x_it, 1e-6); ++x_it;  // pointShadowSphere
  EXPECT_NEAR(11.5, *x_it, 1e-6); ++x_it;  // pointShadowBoth
  EXPECT_NEAR(7, *x_it, 1e-6); ++x_it;  // pointOutside

  ASSERT_EQ(8, cras::numPoints(*pcl_no_bounding_box));
  x_it = cras::CloudConstIter(*pcl_no_bounding_box, "x");
  // EXPECT_NEAR(-1.5, *x_it, 1e-6); ++x_it;  // pointSensor
  // EXPECT_NEAR(-1.47, *x_it, 1e-6); ++x_it;  // pointSensor2
  // EXPECT_NEAR(-1.42, *x_it, 1e-6); ++x_it;  // pointClipMin
  EXPECT_NEAR(10, *x_it, 1e-6); ++x_it;  // pointClipMax
  EXPECT_NEAR(5.85, *x_it, 1e-6); ++x_it;  // pointInBox
  EXPECT_NEAR(6.35, *x_it, 1e-6); ++x_it;  // pointInSphere
  EXPECT_NEAR(5, *x_it, 1e-6); ++x_it;  // pointInBoth
  EXPECT_NEAR(5 - 0.25, *x_it, 1e-6); ++x_it;  // pointShadowBox
  EXPECT_NEAR(10 - 0.560762, *x_it, 1e-6); ++x_it;  // pointShadowSphere
  EXPECT_NEAR(11.5, *x_it, 1e-6); ++x_it;  // pointShadowBoth
  EXPECT_NEAR(7, *x_it, 1e-6); ++x_it;  // pointOutside

  ASSERT_EQ(8, cras::numPoints(*pcl_no_oriented_bounding_box));
  x_it = cras::CloudConstIter(*pcl_no_oriented_bounding_box, "x");
  // EXPECT_NEAR(-1.5, *x_it, 1e-6); ++x_it;  // pointSensor
  // EXPECT_NEAR(-1.47, *x_it, 1e-6); ++x_it;  // pointSensor2
  // EXPECT_NEAR(-1.42, *x_it, 1e-6); ++x_it;  // pointClipMin
  EXPECT_NEAR(10, *x_it, 1e-6); ++x_it;  // pointClipMax
  EXPECT_NEAR(5.85, *x_it, 1e-6); ++x_it;  // pointInBox
  EXPECT_NEAR(6.35, *x_it, 1e-6); ++x_it;  // pointInSphere
  EXPECT_NEAR(5, *x_it, 1e-6); ++x_it;  // pointInBoth
  EXPECT_NEAR(5 - 0.25, *x_it, 1e-6); ++x_it;  // pointShadowBox
  EXPECT_NEAR(10 - 0.560762, *x_it, 1e-6); ++x_it;  // pointShadowSphere
  EXPECT_NEAR(11.5, *x_it, 1e-6); ++x_it;  // pointShadowBoth
  EXPECT_NEAR(7, *x_it, 1e-6); ++x_it;  // pointOutside

  ASSERT_EQ(8, cras::numPoints(*pcl_no_local_bounding_box));
  x_it = cras::CloudConstIter(*pcl_no_local_bounding_box, "x");
  // EXPECT_NEAR(-1.5, *x_it, 1e-6); ++x_it;  // pointSensor
  // EXPECT_NEAR(-1.47, *x_it, 1e-6); ++x_it;  // pointSensor2
  // EXPECT_NEAR(-1.42, *x_it, 1e-6); ++x_it;  // pointClipMin
  EXPECT_NEAR(10, *x_it, 1e-6); ++x_it;  // pointClipMax
  EXPECT_NEAR(5.85, *x_it, 1e-6); ++x_it;  // pointInBox
  EXPECT_NEAR(6.35, *x_it, 1e-6); ++x_it;  // pointInSphere
  EXPECT_NEAR(5, *x_it, 1e-6); ++x_it;  // pointInBoth
  EXPECT_NEAR(5 - 0.25, *x_it, 1e-6); ++x_it;  // pointShadowBox
  EXPECT_NEAR(10 - 0.560762, *x_it, 1e-6); ++x_it;  // pointShadowSphere
  EXPECT_NEAR(11.5, *x_it, 1e-6); ++x_it;  // pointShadowBoth
  EXPECT_NEAR(7, *x_it, 1e-6); ++x_it;  // pointOutside

  ASSERT_EQ(3, cras::numPoints(*pcl_inside));
  x_it = cras::CloudConstIter(*pcl_inside, "x");
  // EXPECT_NEAR(-1.5, *x_it, 1e-6); ++x_it;  // pointSensor
  // EXPECT_NEAR(-1.47, *x_it, 1e-6); ++x_it;  // pointSensor2
  // EXPECT_NEAR(-1.42, *x_it, 1e-6); ++x_it;  // pointClipMin
  // EXPECT_NEAR(10, *x_it, 1e-6); ++x_it;  // pointClipMax
  EXPECT_NEAR(5.85, *x_it, 1e-6); ++x_it;  // pointInBox
  EXPECT_NEAR(6.35, *x_it, 1e-6); ++x_it;  // pointInSphere
  EXPECT_NEAR(5, *x_it, 1e-6); ++x_it;  // pointInBoth
  // EXPECT_NEAR(5 - 0.25, *x_it, 1e-6); ++x_it;  // pointShadowBox
  // EXPECT_NEAR(10 - 0.560762, *x_it, 1e-6); ++x_it;  // pointShadowSphere
  // EXPECT_NEAR(11.5, *x_it, 1e-6); ++x_it;  // pointShadowBoth
  // EXPECT_NEAR(7, *x_it, 1e-6); ++x_it;  // pointOutside

  ASSERT_EQ(4, cras::numPoints(*pcl_clip));
  x_it = cras::CloudConstIter(*pcl_clip, "x");
  EXPECT_NEAR(-1.5, *x_it, 1e-6); ++x_it;  // pointSensor
  EXPECT_NEAR(-1.47, *x_it, 1e-6); ++x_it;  // pointSensor2
  EXPECT_NEAR(-1.42, *x_it, 1e-6); ++x_it;  // pointClipMin
  EXPECT_NEAR(10, *x_it, 1e-6); ++x_it;  // pointClipMax
  // EXPECT_NEAR(5.85, *x_it, 1e-6); ++x_it;  // pointInBox
  // EXPECT_NEAR(6.35, *x_it, 1e-6); ++x_it;  // pointInSphere
  // EXPECT_NEAR(5, *x_it, 1e-6); ++x_it;  // pointInBoth
  // EXPECT_NEAR(5 - 0.25, *x_it, 1e-6); ++x_it;  // pointShadowBox
  // EXPECT_NEAR(10 - 0.560762, *x_it, 1e-6); ++x_it;  // pointShadowSphere
  // EXPECT_NEAR(11.5, *x_it, 1e-6); ++x_it;  // pointShadowBoth
  // EXPECT_NEAR(7, *x_it, 1e-6); ++x_it;  // pointOutside

  ASSERT_EQ(3, cras::numPoints(*pcl_shadow));
  x_it = cras::CloudConstIter(*pcl_shadow, "x");
  // EXPECT_NEAR(-1.5, *x_it, 1e-6); ++x_it;  // pointSensor
  // EXPECT_NEAR(-1.47, *x_it, 1e-6); ++x_it;  // pointSensor2
  // EXPECT_NEAR(-1.42, *x_it, 1e-6); ++x_it;  // pointClipMin
  // EXPECT_NEAR(10, *x_it, 1e-6); ++x_it;  // pointClipMax
  // EXPECT_NEAR(5.85, *x_it, 1e-6); ++x_it;  // pointInBox
  // EXPECT_NEAR(6.35, *x_it, 1e-6); ++x_it;  // pointInSphere
  // EXPECT_NEAR(5, *x_it, 1e-6); ++x_it;  // pointInBoth
  EXPECT_NEAR(5 - 0.25, *x_it, 1e-6); ++x_it;  // pointShadowBox
  EXPECT_NEAR(10 - 0.560762, *x_it, 1e-6); ++x_it;  // pointShadowSphere
  EXPECT_NEAR(11.5, *x_it, 1e-6); ++x_it;  // pointShadowBoth
  // EXPECT_NEAR(7, *x_it, 1e-6); ++x_it;  // pointOutside

  ASSERT_EQ(2, bounding_sphere_debug_marker->markers.size());
  EXPECT_EQ("odom", bounding_sphere_debug_marker->markers[0].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, bounding_sphere_debug_marker->markers[0].header.stamp);
  EXPECT_NEAR(2 * sqrt(3) * 0.92, bounding_sphere_debug_marker->markers[0].scale.x, 1e-6);
  EXPECT_NEAR(2 * sqrt(3) * 0.92, bounding_sphere_debug_marker->markers[0].scale.y, 1e-6);
  EXPECT_NEAR(2 * sqrt(3) * 0.92, bounding_sphere_debug_marker->markers[0].scale.z, 1e-6);
  EXPECT_NEAR(0, bounding_sphere_debug_marker->markers[0].pose.position.x, 1e-6);
  EXPECT_NEAR(0, bounding_sphere_debug_marker->markers[0].pose.position.y, 1e-6);
  EXPECT_NEAR(0, bounding_sphere_debug_marker->markers[0].pose.position.z, 1e-6);
  EXPECT_NEAR(0, bounding_sphere_debug_marker->markers[0].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, bounding_sphere_debug_marker->markers[0].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, bounding_sphere_debug_marker->markers[0].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, bounding_sphere_debug_marker->markers[0].pose.orientation.w, 1e-6);
  EXPECT_LT(0, bounding_sphere_debug_marker->markers[0].color.a);
  // make sure the marker has some color
  EXPECT_LT(
    0,
    bounding_sphere_debug_marker->markers[0].color.r + bounding_sphere_debug_marker->markers[0].color.g +
    bounding_sphere_debug_marker->markers[0].color.b);
  EXPECT_EQ(visualization_msgs::msg::Marker::SPHERE, bounding_sphere_debug_marker->markers[0].type);
  EXPECT_EQ(visualization_msgs::msg::Marker::ADD, bounding_sphere_debug_marker->markers[0].action);
  EXPECT_FALSE(bounding_sphere_debug_marker->markers[0].ns.empty());
  EXPECT_EQ(1, bounding_sphere_debug_marker->markers[0].frame_locked);
  EXPECT_EQ("odom", bounding_sphere_debug_marker->markers[1].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, bounding_sphere_debug_marker->markers[1].header.stamp);
  EXPECT_NEAR(0.1 * sqrt(3), bounding_sphere_debug_marker->markers[1].scale.x, 1e-5);
  EXPECT_NEAR(0.1 * sqrt(3), bounding_sphere_debug_marker->markers[1].scale.y, 1e-5);
  EXPECT_NEAR(0.1 * sqrt(3), bounding_sphere_debug_marker->markers[1].scale.z, 1e-5);
  EXPECT_NEAR(-1.5, bounding_sphere_debug_marker->markers[1].pose.position.x, 1e-5);
  EXPECT_NEAR(0, bounding_sphere_debug_marker->markers[1].pose.position.y, 1e-6);
  EXPECT_NEAR(0, bounding_sphere_debug_marker->markers[1].pose.position.z, 1e-6);
  EXPECT_NEAR(0, bounding_sphere_debug_marker->markers[1].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, bounding_sphere_debug_marker->markers[1].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, bounding_sphere_debug_marker->markers[1].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, bounding_sphere_debug_marker->markers[1].pose.orientation.w, 1e-6);
  EXPECT_LT(0, bounding_sphere_debug_marker->markers[1].color.a);
  // make sure the marker has some color
  EXPECT_LT(
    0,
    bounding_sphere_debug_marker->markers[1].color.r + bounding_sphere_debug_marker->markers[1].color.g +
    bounding_sphere_debug_marker->markers[1].color.b);
  EXPECT_EQ(visualization_msgs::msg::Marker::SPHERE, bounding_sphere_debug_marker->markers[1].type);
  EXPECT_EQ(visualization_msgs::msg::Marker::ADD, bounding_sphere_debug_marker->markers[1].action);
  EXPECT_FALSE(bounding_sphere_debug_marker->markers[1].ns.empty());
  EXPECT_EQ(1, bounding_sphere_debug_marker->markers[1].frame_locked);

  EXPECT_EQ(4, bounding_box_debug_marker->markers.size());
  EXPECT_EQ("odom", bounding_box_debug_marker->markers[0].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, bounding_box_debug_marker->markers[0].header.stamp);
  EXPECT_NEAR(2.75, bounding_box_debug_marker->markers[0].scale.x, 1e-5);
  EXPECT_NEAR(2.75, bounding_box_debug_marker->markers[0].scale.y, 1e-5);
  EXPECT_NEAR(2.75, bounding_box_debug_marker->markers[0].scale.z, 1e-5);
  EXPECT_NEAR(0, bounding_box_debug_marker->markers[0].pose.position.x, 1e-6);
  EXPECT_NEAR(0, bounding_box_debug_marker->markers[0].pose.position.y, 1e-6);
  EXPECT_NEAR(0, bounding_box_debug_marker->markers[0].pose.position.z, 1e-6);
  EXPECT_NEAR(0, bounding_box_debug_marker->markers[0].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, bounding_box_debug_marker->markers[0].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, bounding_box_debug_marker->markers[0].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, bounding_box_debug_marker->markers[0].pose.orientation.w, 1e-6);
  EXPECT_LT(0, bounding_box_debug_marker->markers[0].color.a);
  // make sure the marker has some color
  EXPECT_LT(
    0,
    bounding_box_debug_marker->markers[0].color.r + bounding_box_debug_marker->markers[0].color.g +
    bounding_box_debug_marker->markers[0].color.b);
  EXPECT_EQ(visualization_msgs::msg::Marker::CUBE, bounding_box_debug_marker->markers[0].type);
  EXPECT_EQ(visualization_msgs::msg::Marker::ADD, bounding_box_debug_marker->markers[0].action);
  EXPECT_FALSE(bounding_box_debug_marker->markers[0].ns.empty());
  EXPECT_EQ(1, bounding_box_debug_marker->markers[0].frame_locked);
  EXPECT_EQ("odom", bounding_box_debug_marker->markers[1].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, bounding_box_debug_marker->markers[1].header.stamp);
  EXPECT_NEAR(1.84, bounding_box_debug_marker->markers[1].scale.x, 1e-5);
  EXPECT_NEAR(1.84, bounding_box_debug_marker->markers[1].scale.y, 1e-5);
  EXPECT_NEAR(1.84, bounding_box_debug_marker->markers[1].scale.z, 1e-5);
  EXPECT_NEAR(0, bounding_box_debug_marker->markers[1].pose.position.x, 1e-6);
  EXPECT_NEAR(0, bounding_box_debug_marker->markers[1].pose.position.y, 1e-6);
  EXPECT_NEAR(0, bounding_box_debug_marker->markers[1].pose.position.z, 1e-6);
  EXPECT_NEAR(0, bounding_box_debug_marker->markers[1].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, bounding_box_debug_marker->markers[1].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, bounding_box_debug_marker->markers[1].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, bounding_box_debug_marker->markers[1].pose.orientation.w, 1e-6);
  EXPECT_LT(0, bounding_box_debug_marker->markers[1].color.a);
  // make sure the marker has some color
  EXPECT_LT(
    0,
    bounding_box_debug_marker->markers[1].color.r + bounding_box_debug_marker->markers[1].color.g +
    bounding_box_debug_marker->markers[1].color.b);
  EXPECT_EQ(visualization_msgs::msg::Marker::CUBE, bounding_box_debug_marker->markers[1].type);
  EXPECT_EQ(visualization_msgs::msg::Marker::ADD, bounding_box_debug_marker->markers[1].action);
  EXPECT_FALSE(bounding_box_debug_marker->markers[1].ns.empty());
  EXPECT_EQ(1, bounding_box_debug_marker->markers[1].frame_locked);
  EXPECT_EQ("odom", bounding_box_debug_marker->markers[2].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, bounding_box_debug_marker->markers[2].header.stamp);
  EXPECT_NEAR(2.5, bounding_box_debug_marker->markers[2].scale.x, 1e-4);
  EXPECT_NEAR(2.5, bounding_box_debug_marker->markers[2].scale.y, 1e-4);
  EXPECT_NEAR(2.5, bounding_box_debug_marker->markers[2].scale.z, 1e-4);
  EXPECT_NEAR(0.122 - 0.1, bounding_box_debug_marker->markers[2].pose.position.x, 1e-6);
  EXPECT_NEAR(0, bounding_box_debug_marker->markers[2].pose.position.y, 1e-6);
  EXPECT_NEAR(0, bounding_box_debug_marker->markers[2].pose.position.z, 1e-6);
  EXPECT_NEAR(0, bounding_box_debug_marker->markers[2].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, bounding_box_debug_marker->markers[2].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, bounding_box_debug_marker->markers[2].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, bounding_box_debug_marker->markers[2].pose.orientation.w, 1e-6);
  EXPECT_LT(0, bounding_box_debug_marker->markers[2].color.a);
  // make sure the marker has some color
  EXPECT_LT(
    0,
    bounding_box_debug_marker->markers[2].color.r + bounding_box_debug_marker->markers[2].color.g +
    bounding_box_debug_marker->markers[2].color.b);
  EXPECT_EQ(visualization_msgs::msg::Marker::CUBE, bounding_box_debug_marker->markers[2].type);
  EXPECT_EQ(visualization_msgs::msg::Marker::ADD, bounding_box_debug_marker->markers[2].action);
  EXPECT_FALSE(bounding_box_debug_marker->markers[2].ns.empty());
  EXPECT_EQ(1, bounding_box_debug_marker->markers[2].frame_locked);
  EXPECT_EQ("odom", bounding_box_debug_marker->markers[3].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, bounding_box_debug_marker->markers[3].header.stamp);
  EXPECT_NEAR(0.1, bounding_box_debug_marker->markers[3].scale.x, 1e-5);
  EXPECT_NEAR(0.1, bounding_box_debug_marker->markers[3].scale.y, 1e-5);
  EXPECT_NEAR(0.1, bounding_box_debug_marker->markers[3].scale.z, 1e-5);
  EXPECT_NEAR(-1.5, bounding_box_debug_marker->markers[3].pose.position.x, 1e-6);
  EXPECT_NEAR(0, bounding_box_debug_marker->markers[3].pose.position.y, 1e-6);
  EXPECT_NEAR(0, bounding_box_debug_marker->markers[3].pose.position.z, 1e-6);
  EXPECT_NEAR(0, bounding_box_debug_marker->markers[3].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, bounding_box_debug_marker->markers[3].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, bounding_box_debug_marker->markers[3].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, bounding_box_debug_marker->markers[3].pose.orientation.w, 1e-6);
  EXPECT_LT(0, bounding_box_debug_marker->markers[3].color.a);
  // make sure the marker has some color
  EXPECT_LT(
    0,
    bounding_box_debug_marker->markers[3].color.r + bounding_box_debug_marker->markers[3].color.g +
    bounding_box_debug_marker->markers[3].color.b);
  EXPECT_EQ(visualization_msgs::msg::Marker::CUBE, bounding_box_debug_marker->markers[3].type);
  EXPECT_EQ(visualization_msgs::msg::Marker::ADD, bounding_box_debug_marker->markers[3].action);
  EXPECT_FALSE(bounding_box_debug_marker->markers[3].ns.empty());
  EXPECT_EQ(1, bounding_box_debug_marker->markers[3].frame_locked);

  EXPECT_EQ(4, oriented_bounding_box_debug_marker->markers.size());
  EXPECT_EQ("odom", oriented_bounding_box_debug_marker->markers[0].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, oriented_bounding_box_debug_marker->markers[0].header.stamp);
  EXPECT_NEAR(2.75, oriented_bounding_box_debug_marker->markers[0].scale.x, 1e-5);
  EXPECT_NEAR(2.75, oriented_bounding_box_debug_marker->markers[0].scale.y, 1e-5);
  EXPECT_NEAR(2.75, oriented_bounding_box_debug_marker->markers[0].scale.z, 1e-5);
  EXPECT_NEAR(0, oriented_bounding_box_debug_marker->markers[0].pose.position.x, 1e-6);
  EXPECT_NEAR(0, oriented_bounding_box_debug_marker->markers[0].pose.position.y, 1e-6);
  EXPECT_NEAR(0, oriented_bounding_box_debug_marker->markers[0].pose.position.z, 1e-6);
  EXPECT_NEAR(0, oriented_bounding_box_debug_marker->markers[0].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, oriented_bounding_box_debug_marker->markers[0].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, oriented_bounding_box_debug_marker->markers[0].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, oriented_bounding_box_debug_marker->markers[0].pose.orientation.w, 1e-6);
  EXPECT_LT(0, oriented_bounding_box_debug_marker->markers[0].color.a);
  // make sure the marker has some color
  EXPECT_LT(
    0,
    oriented_bounding_box_debug_marker->markers[0].color.r + oriented_bounding_box_debug_marker->markers[0].color.g +
    oriented_bounding_box_debug_marker->markers[0].color.b);
  EXPECT_EQ(visualization_msgs::msg::Marker::CUBE, oriented_bounding_box_debug_marker->markers[0].type);
  EXPECT_EQ(visualization_msgs::msg::Marker::ADD, oriented_bounding_box_debug_marker->markers[0].action);
  EXPECT_FALSE(oriented_bounding_box_debug_marker->markers[0].ns.empty());
  EXPECT_EQ(1, oriented_bounding_box_debug_marker->markers[0].frame_locked);
  EXPECT_EQ("odom", oriented_bounding_box_debug_marker->markers[1].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, oriented_bounding_box_debug_marker->markers[1].header.stamp);
  EXPECT_NEAR(1.84, oriented_bounding_box_debug_marker->markers[1].scale.x, 1e-5);
  EXPECT_NEAR(1.84, oriented_bounding_box_debug_marker->markers[1].scale.y, 1e-5);
  EXPECT_NEAR(1.84, oriented_bounding_box_debug_marker->markers[1].scale.z, 1e-5);
  EXPECT_NEAR(0, oriented_bounding_box_debug_marker->markers[1].pose.position.x, 1e-6);
  EXPECT_NEAR(0, oriented_bounding_box_debug_marker->markers[1].pose.position.y, 1e-6);
  EXPECT_NEAR(0, oriented_bounding_box_debug_marker->markers[1].pose.position.z, 1e-6);
  EXPECT_NEAR(0, oriented_bounding_box_debug_marker->markers[1].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, oriented_bounding_box_debug_marker->markers[1].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, oriented_bounding_box_debug_marker->markers[1].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, oriented_bounding_box_debug_marker->markers[1].pose.orientation.w, 1e-6);
  EXPECT_LT(0, oriented_bounding_box_debug_marker->markers[1].color.a);
  // make sure the marker has some color
  EXPECT_LT(
    0,
    oriented_bounding_box_debug_marker->markers[1].color.r + oriented_bounding_box_debug_marker->markers[1].color.g +
    oriented_bounding_box_debug_marker->markers[1].color.b);
  EXPECT_EQ(visualization_msgs::msg::Marker::CUBE, oriented_bounding_box_debug_marker->markers[1].type);
  EXPECT_EQ(visualization_msgs::msg::Marker::ADD, oriented_bounding_box_debug_marker->markers[1].action);
  EXPECT_FALSE(oriented_bounding_box_debug_marker->markers[1].ns.empty());
  EXPECT_EQ(1, oriented_bounding_box_debug_marker->markers[1].frame_locked);
  EXPECT_EQ("odom", oriented_bounding_box_debug_marker->markers[2].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, oriented_bounding_box_debug_marker->markers[2].header.stamp);
  EXPECT_NEAR(2.5, oriented_bounding_box_debug_marker->markers[2].scale.x, 1e-4);
  EXPECT_NEAR(2.5, oriented_bounding_box_debug_marker->markers[2].scale.y, 1e-4);
  EXPECT_NEAR(2.5, oriented_bounding_box_debug_marker->markers[2].scale.z, 1e-4);
  EXPECT_NEAR(0.122 - 0.1, oriented_bounding_box_debug_marker->markers[2].pose.position.x, 1e-6);
  EXPECT_NEAR(0, oriented_bounding_box_debug_marker->markers[2].pose.position.y, 1e-6);
  EXPECT_NEAR(0, oriented_bounding_box_debug_marker->markers[2].pose.position.z, 1e-6);
  EXPECT_NEAR(0, oriented_bounding_box_debug_marker->markers[2].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, oriented_bounding_box_debug_marker->markers[2].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, oriented_bounding_box_debug_marker->markers[2].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, oriented_bounding_box_debug_marker->markers[2].pose.orientation.w, 1e-6);
  EXPECT_LT(0, oriented_bounding_box_debug_marker->markers[2].color.a);
  // make sure the marker has some color
  EXPECT_LT(
    0,
    oriented_bounding_box_debug_marker->markers[2].color.r + oriented_bounding_box_debug_marker->markers[2].color.g +
    oriented_bounding_box_debug_marker->markers[2].color.b);
  EXPECT_EQ(visualization_msgs::msg::Marker::CUBE, oriented_bounding_box_debug_marker->markers[2].type);
  EXPECT_EQ(visualization_msgs::msg::Marker::ADD, oriented_bounding_box_debug_marker->markers[2].action);
  EXPECT_FALSE(oriented_bounding_box_debug_marker->markers[2].ns.empty());
  EXPECT_EQ(1, oriented_bounding_box_debug_marker->markers[2].frame_locked);
  EXPECT_EQ("odom", oriented_bounding_box_debug_marker->markers[3].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, oriented_bounding_box_debug_marker->markers[3].header.stamp);
  EXPECT_NEAR(0.1, oriented_bounding_box_debug_marker->markers[3].scale.x, 1e-5);
  EXPECT_NEAR(0.1, oriented_bounding_box_debug_marker->markers[3].scale.y, 1e-5);
  EXPECT_NEAR(0.1, oriented_bounding_box_debug_marker->markers[3].scale.z, 1e-5);
  EXPECT_NEAR(-1.5, oriented_bounding_box_debug_marker->markers[3].pose.position.x, 1e-6);
  EXPECT_NEAR(0, oriented_bounding_box_debug_marker->markers[3].pose.position.y, 1e-6);
  EXPECT_NEAR(0, oriented_bounding_box_debug_marker->markers[3].pose.position.z, 1e-6);
  EXPECT_NEAR(0, oriented_bounding_box_debug_marker->markers[3].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, oriented_bounding_box_debug_marker->markers[3].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, oriented_bounding_box_debug_marker->markers[3].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, oriented_bounding_box_debug_marker->markers[3].pose.orientation.w, 1e-6);
  EXPECT_LT(0, oriented_bounding_box_debug_marker->markers[3].color.a);
  // make sure the marker has some color
  EXPECT_LT(
    0,
    oriented_bounding_box_debug_marker->markers[3].color.r + oriented_bounding_box_debug_marker->markers[3].color.g +
    oriented_bounding_box_debug_marker->markers[3].color.b);
  EXPECT_EQ(visualization_msgs::msg::Marker::CUBE, oriented_bounding_box_debug_marker->markers[3].type);
  EXPECT_EQ(visualization_msgs::msg::Marker::ADD, oriented_bounding_box_debug_marker->markers[3].action);
  EXPECT_FALSE(oriented_bounding_box_debug_marker->markers[3].ns.empty());
  EXPECT_EQ(1, oriented_bounding_box_debug_marker->markers[3].frame_locked);

  EXPECT_EQ(4, local_bounding_box_debug_marker->markers.size());
  EXPECT_EQ("base_link", local_bounding_box_debug_marker->markers[0].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, local_bounding_box_debug_marker->markers[0].header.stamp);
  EXPECT_NEAR(2.75, local_bounding_box_debug_marker->markers[0].scale.x, 1e-5);
  EXPECT_NEAR(2.75, local_bounding_box_debug_marker->markers[0].scale.y, 1e-5);
  EXPECT_NEAR(2.75, local_bounding_box_debug_marker->markers[0].scale.z, 1e-5);
  EXPECT_NEAR(-0.122, local_bounding_box_debug_marker->markers[0].pose.position.x, 1e-6);
  EXPECT_NEAR(0, local_bounding_box_debug_marker->markers[0].pose.position.y, 1e-6);
  EXPECT_NEAR(0, local_bounding_box_debug_marker->markers[0].pose.position.z, 1e-6);
  EXPECT_NEAR(0, local_bounding_box_debug_marker->markers[0].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, local_bounding_box_debug_marker->markers[0].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, local_bounding_box_debug_marker->markers[0].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, local_bounding_box_debug_marker->markers[0].pose.orientation.w, 1e-6);
  EXPECT_LT(0, local_bounding_box_debug_marker->markers[0].color.a);
  // make sure the marker has some color
  EXPECT_LT(
    0,
    local_bounding_box_debug_marker->markers[0].color.r + local_bounding_box_debug_marker->markers[0].color.g +
    local_bounding_box_debug_marker->markers[0].color.b);
  EXPECT_EQ(visualization_msgs::msg::Marker::CUBE, local_bounding_box_debug_marker->markers[0].type);
  EXPECT_EQ(visualization_msgs::msg::Marker::ADD, local_bounding_box_debug_marker->markers[0].action);
  EXPECT_FALSE(local_bounding_box_debug_marker->markers[0].ns.empty());
  EXPECT_EQ(1, local_bounding_box_debug_marker->markers[0].frame_locked);
  EXPECT_EQ("base_link", local_bounding_box_debug_marker->markers[1].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, local_bounding_box_debug_marker->markers[1].header.stamp);
  EXPECT_NEAR(1.84, local_bounding_box_debug_marker->markers[1].scale.x, 1e-5);
  EXPECT_NEAR(1.84, local_bounding_box_debug_marker->markers[1].scale.y, 1e-5);
  EXPECT_NEAR(1.84, local_bounding_box_debug_marker->markers[1].scale.z, 1e-5);
  EXPECT_NEAR(-0.122, local_bounding_box_debug_marker->markers[1].pose.position.x, 1e-6);
  EXPECT_NEAR(0, local_bounding_box_debug_marker->markers[1].pose.position.y, 1e-6);
  EXPECT_NEAR(0, local_bounding_box_debug_marker->markers[1].pose.position.z, 1e-6);
  EXPECT_NEAR(0, local_bounding_box_debug_marker->markers[1].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, local_bounding_box_debug_marker->markers[1].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, local_bounding_box_debug_marker->markers[1].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, local_bounding_box_debug_marker->markers[1].pose.orientation.w, 1e-6);
  EXPECT_LT(0, local_bounding_box_debug_marker->markers[1].color.a);
  // make sure the marker has some color
  EXPECT_LT(
    0,
    local_bounding_box_debug_marker->markers[1].color.r + local_bounding_box_debug_marker->markers[1].color.g +
    local_bounding_box_debug_marker->markers[1].color.b);
  EXPECT_EQ(visualization_msgs::msg::Marker::CUBE, local_bounding_box_debug_marker->markers[1].type);
  EXPECT_EQ(visualization_msgs::msg::Marker::ADD, local_bounding_box_debug_marker->markers[1].action);
  EXPECT_FALSE(local_bounding_box_debug_marker->markers[1].ns.empty());
  EXPECT_EQ(1, local_bounding_box_debug_marker->markers[1].frame_locked);
  EXPECT_EQ("base_link", local_bounding_box_debug_marker->markers[2].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, local_bounding_box_debug_marker->markers[2].header.stamp);
  EXPECT_NEAR(2.5, local_bounding_box_debug_marker->markers[2].scale.x, 1e-4);
  EXPECT_NEAR(2.5, local_bounding_box_debug_marker->markers[2].scale.y, 1e-4);
  EXPECT_NEAR(2.5, local_bounding_box_debug_marker->markers[2].scale.z, 1e-4);
  EXPECT_NEAR(-0.1, local_bounding_box_debug_marker->markers[2].pose.position.x, 1e-6);
  EXPECT_NEAR(0, local_bounding_box_debug_marker->markers[2].pose.position.y, 1e-6);
  EXPECT_NEAR(0, local_bounding_box_debug_marker->markers[2].pose.position.z, 1e-6);
  EXPECT_NEAR(0, local_bounding_box_debug_marker->markers[2].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, local_bounding_box_debug_marker->markers[2].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, local_bounding_box_debug_marker->markers[2].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, local_bounding_box_debug_marker->markers[2].pose.orientation.w, 1e-6);
  EXPECT_LT(0, local_bounding_box_debug_marker->markers[2].color.a);
  // make sure the marker has some color
  EXPECT_LT(
    0,
    local_bounding_box_debug_marker->markers[2].color.r + local_bounding_box_debug_marker->markers[2].color.g +
    local_bounding_box_debug_marker->markers[2].color.b);
  EXPECT_EQ(visualization_msgs::msg::Marker::CUBE, local_bounding_box_debug_marker->markers[2].type);
  EXPECT_EQ(visualization_msgs::msg::Marker::ADD, local_bounding_box_debug_marker->markers[2].action);
  EXPECT_FALSE(local_bounding_box_debug_marker->markers[2].ns.empty());
  EXPECT_EQ(1, local_bounding_box_debug_marker->markers[2].frame_locked);
  EXPECT_EQ("base_link", local_bounding_box_debug_marker->markers[3].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, local_bounding_box_debug_marker->markers[3].header.stamp);
  EXPECT_NEAR(0.1, local_bounding_box_debug_marker->markers[3].scale.x, 1e-5);
  EXPECT_NEAR(0.1, local_bounding_box_debug_marker->markers[3].scale.y, 1e-5);
  EXPECT_NEAR(0.1, local_bounding_box_debug_marker->markers[3].scale.z, 1e-5);
  EXPECT_NEAR(-1.5 - 0.122, local_bounding_box_debug_marker->markers[3].pose.position.x, 1e-6);
  EXPECT_NEAR(0, local_bounding_box_debug_marker->markers[3].pose.position.y, 1e-6);
  EXPECT_NEAR(0, local_bounding_box_debug_marker->markers[3].pose.position.z, 1e-6);
  EXPECT_NEAR(0, local_bounding_box_debug_marker->markers[3].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, local_bounding_box_debug_marker->markers[3].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, local_bounding_box_debug_marker->markers[3].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, local_bounding_box_debug_marker->markers[3].pose.orientation.w, 1e-6);
  EXPECT_LT(0, local_bounding_box_debug_marker->markers[3].color.a);
  // make sure the marker has some color
  EXPECT_LT(
    0,
    local_bounding_box_debug_marker->markers[3].color.r + local_bounding_box_debug_marker->markers[3].color.g +
    local_bounding_box_debug_marker->markers[3].color.b);
  EXPECT_EQ(visualization_msgs::msg::Marker::CUBE, local_bounding_box_debug_marker->markers[3].type);
  EXPECT_EQ(visualization_msgs::msg::Marker::ADD, local_bounding_box_debug_marker->markers[3].action);
  EXPECT_FALSE(local_bounding_box_debug_marker->markers[3].ns.empty());
  EXPECT_EQ(1, local_bounding_box_debug_marker->markers[3].frame_locked);

  EXPECT_EQ(4, robot_model_contains_test->markers.size());
  EXPECT_EQ("odom", robot_model_contains_test->markers[0].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, robot_model_contains_test->markers[0].header.stamp);
  EXPECT_NEAR(2.75, robot_model_contains_test->markers[0].scale.x, 1e-5);
  EXPECT_NEAR(2.75, robot_model_contains_test->markers[0].scale.y, 1e-5);
  EXPECT_NEAR(2.75, robot_model_contains_test->markers[0].scale.z, 1e-5);
  EXPECT_NEAR(0, robot_model_contains_test->markers[0].pose.position.x, 1e-6);
  EXPECT_NEAR(0, robot_model_contains_test->markers[0].pose.position.y, 1e-6);
  EXPECT_NEAR(0, robot_model_contains_test->markers[0].pose.position.z, 1e-6);
  EXPECT_NEAR(0, robot_model_contains_test->markers[0].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, robot_model_contains_test->markers[0].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, robot_model_contains_test->markers[0].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, robot_model_contains_test->markers[0].pose.orientation.w, 1e-6);
  EXPECT_LT(0, robot_model_contains_test->markers[0].color.a);
  // make sure the marker has some color
  EXPECT_LT(
    0,
    robot_model_contains_test->markers[0].color.r + robot_model_contains_test->markers[0].color.g +
    robot_model_contains_test->markers[0].color.b);
  EXPECT_EQ(visualization_msgs::msg::Marker::SPHERE, robot_model_contains_test->markers[0].type);
  EXPECT_EQ(visualization_msgs::msg::Marker::ADD, robot_model_contains_test->markers[0].action);
  EXPECT_EQ("antenna-0", robot_model_contains_test->markers[0].ns);
  EXPECT_EQ(1, robot_model_contains_test->markers[0].frame_locked);
  EXPECT_EQ("odom", robot_model_contains_test->markers[1].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, robot_model_contains_test->markers[1].header.stamp);
  EXPECT_NEAR(1.84, robot_model_contains_test->markers[1].scale.x, 1e-5);
  EXPECT_NEAR(1.84, robot_model_contains_test->markers[1].scale.y, 1e-5);
  EXPECT_NEAR(1.84, robot_model_contains_test->markers[1].scale.z, 1e-5);
  EXPECT_NEAR(0, robot_model_contains_test->markers[1].pose.position.x, 1e-6);
  EXPECT_NEAR(0, robot_model_contains_test->markers[1].pose.position.y, 1e-6);
  EXPECT_NEAR(0, robot_model_contains_test->markers[1].pose.position.z, 1e-6);
  EXPECT_NEAR(0, robot_model_contains_test->markers[1].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, robot_model_contains_test->markers[1].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, robot_model_contains_test->markers[1].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, robot_model_contains_test->markers[1].pose.orientation.w, 1e-6);
  EXPECT_LT(0, robot_model_contains_test->markers[1].color.a);
  // make sure the marker has some color
  EXPECT_LT(
    0,
    robot_model_contains_test->markers[1].color.r + robot_model_contains_test->markers[1].color.g +
    robot_model_contains_test->markers[1].color.b);
  EXPECT_EQ(visualization_msgs::msg::Marker::CUBE, robot_model_contains_test->markers[1].type);
  EXPECT_EQ(visualization_msgs::msg::Marker::ADD, robot_model_contains_test->markers[1].action);
  EXPECT_EQ("base_link-0", robot_model_contains_test->markers[1].ns);
  EXPECT_EQ(1, robot_model_contains_test->markers[1].frame_locked);
  EXPECT_EQ("odom", robot_model_contains_test->markers[2].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, robot_model_contains_test->markers[2].header.stamp);
  EXPECT_NEAR(2.5, robot_model_contains_test->markers[2].scale.x, 1e-4);
  EXPECT_NEAR(2.5, robot_model_contains_test->markers[2].scale.y, 1e-4);
  EXPECT_NEAR(2.5, robot_model_contains_test->markers[2].scale.z, 1e-4);
  EXPECT_NEAR(0.122 - 0.1, robot_model_contains_test->markers[2].pose.position.x, 1e-6);
  EXPECT_NEAR(0, robot_model_contains_test->markers[2].pose.position.y, 1e-6);
  EXPECT_NEAR(0, robot_model_contains_test->markers[2].pose.position.z, 1e-6);
  EXPECT_NEAR(0, robot_model_contains_test->markers[2].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, robot_model_contains_test->markers[2].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, robot_model_contains_test->markers[2].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, robot_model_contains_test->markers[2].pose.orientation.w, 1e-6);
  EXPECT_LT(0, robot_model_contains_test->markers[2].color.a);
  // make sure the marker has some color
  EXPECT_LT(
    0,
    robot_model_contains_test->markers[2].color.r + robot_model_contains_test->markers[2].color.g +
    robot_model_contains_test->markers[2].color.b);
  EXPECT_EQ(visualization_msgs::msg::Marker::CUBE, robot_model_contains_test->markers[2].type);
  EXPECT_EQ(visualization_msgs::msg::Marker::ADD, robot_model_contains_test->markers[2].action);
  EXPECT_EQ("base_link-1", robot_model_contains_test->markers[2].ns);
  EXPECT_EQ(1, robot_model_contains_test->markers[2].frame_locked);
  EXPECT_EQ("odom", robot_model_contains_test->markers[3].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, robot_model_contains_test->markers[3].header.stamp);
  EXPECT_NEAR(0.1, robot_model_contains_test->markers[3].scale.x, 1e-5);
  EXPECT_NEAR(0.1, robot_model_contains_test->markers[3].scale.y, 1e-5);
  EXPECT_NEAR(0.1, robot_model_contains_test->markers[3].scale.z, 1e-5);
  EXPECT_NEAR(-1.5, robot_model_contains_test->markers[3].pose.position.x, 1e-6);
  EXPECT_NEAR(0, robot_model_contains_test->markers[3].pose.position.y, 1e-6);
  EXPECT_NEAR(0, robot_model_contains_test->markers[3].pose.position.z, 1e-6);
  EXPECT_NEAR(0, robot_model_contains_test->markers[3].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, robot_model_contains_test->markers[3].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, robot_model_contains_test->markers[3].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, robot_model_contains_test->markers[3].pose.orientation.w, 1e-6);
  EXPECT_LT(0, robot_model_contains_test->markers[3].color.a);
  // make sure the marker has some color
  EXPECT_LT(
    0,
    robot_model_contains_test->markers[3].color.r + robot_model_contains_test->markers[3].color.g +
    robot_model_contains_test->markers[3].color.b);
  EXPECT_EQ(visualization_msgs::msg::Marker::CUBE, robot_model_contains_test->markers[3].type);
  EXPECT_EQ(visualization_msgs::msg::Marker::ADD, robot_model_contains_test->markers[3].action);
  EXPECT_EQ("laser-0", robot_model_contains_test->markers[3].ns);
  EXPECT_EQ(1, robot_model_contains_test->markers[3].frame_locked);

  EXPECT_EQ(2, robot_model_shadow_test->markers.size());
  EXPECT_EQ("odom", robot_model_shadow_test->markers[0].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, robot_model_shadow_test->markers[0].header.stamp);
  EXPECT_NEAR(2.75, robot_model_shadow_test->markers[0].scale.x, 1e-5);
  EXPECT_NEAR(2.75, robot_model_shadow_test->markers[0].scale.y, 1e-5);
  EXPECT_NEAR(2.75, robot_model_shadow_test->markers[0].scale.z, 1e-5);
  EXPECT_NEAR(0, robot_model_shadow_test->markers[0].pose.position.x, 1e-6);
  EXPECT_NEAR(0, robot_model_shadow_test->markers[0].pose.position.y, 1e-6);
  EXPECT_NEAR(0, robot_model_shadow_test->markers[0].pose.position.z, 1e-6);
  EXPECT_NEAR(0, robot_model_shadow_test->markers[0].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, robot_model_shadow_test->markers[0].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, robot_model_shadow_test->markers[0].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, robot_model_shadow_test->markers[0].pose.orientation.w, 1e-6);
  EXPECT_LT(0, robot_model_shadow_test->markers[0].color.a);
  // make sure the marker has some color
  EXPECT_LT(
    0,
    robot_model_shadow_test->markers[0].color.r + robot_model_shadow_test->markers[0].color.g +
    robot_model_shadow_test->markers[0].color.b);
  EXPECT_EQ(visualization_msgs::msg::Marker::SPHERE, robot_model_shadow_test->markers[0].type);
  EXPECT_EQ(visualization_msgs::msg::Marker::ADD, robot_model_shadow_test->markers[0].action);
  EXPECT_EQ("antenna-0", robot_model_shadow_test->markers[0].ns);
  EXPECT_EQ(1, robot_model_shadow_test->markers[0].frame_locked);
  EXPECT_EQ("odom", robot_model_shadow_test->markers[1].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, robot_model_shadow_test->markers[1].header.stamp);
  EXPECT_NEAR(2, robot_model_shadow_test->markers[1].scale.x, 1e-5);
  EXPECT_NEAR(2, robot_model_shadow_test->markers[1].scale.y, 1e-5);
  EXPECT_NEAR(2, robot_model_shadow_test->markers[1].scale.z, 1e-5);
  EXPECT_NEAR(0, robot_model_shadow_test->markers[1].pose.position.x, 1e-6);
  EXPECT_NEAR(0, robot_model_shadow_test->markers[1].pose.position.y, 1e-6);
  EXPECT_NEAR(0, robot_model_shadow_test->markers[1].pose.position.z, 1e-6);
  EXPECT_NEAR(0, robot_model_shadow_test->markers[1].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, robot_model_shadow_test->markers[1].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, robot_model_shadow_test->markers[1].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, robot_model_shadow_test->markers[1].pose.orientation.w, 1e-6);
  EXPECT_LT(0, robot_model_shadow_test->markers[1].color.a);
  // make sure the marker has some color
  EXPECT_LT(
    0,
    robot_model_shadow_test->markers[1].color.r + robot_model_shadow_test->markers[1].color.g +
    robot_model_shadow_test->markers[1].color.b);
  EXPECT_EQ(visualization_msgs::msg::Marker::CUBE, robot_model_shadow_test->markers[1].type);
  EXPECT_EQ(visualization_msgs::msg::Marker::ADD, robot_model_shadow_test->markers[1].action);
  EXPECT_EQ("base_link-0", robot_model_shadow_test->markers[1].ns);
  EXPECT_EQ(1, robot_model_shadow_test->markers[1].frame_locked);
}  // NOLINT

TEST(RobotBodyFilter, ComputeMaskAllAtOnce) {
  const auto nh = std::make_shared<rclcpp::Node>("compute_mask_config_all_at_once");

  const auto filter = std::make_shared<RobotBodyFilterPointCloud2Test>(nullptr);
  const auto filter_base = std::dynamic_pointer_cast<filters::FilterBase<sensor_msgs::msg::PointCloud2>>(filter);

  filter_base->configure(
    "filter1.params", "compute_mask_config_all_at_once",
    nh->get_node_logging_interface(), nh->get_node_parameters_interface());

  const std_msgs::msg::String::SharedPtr msg(new std_msgs::msg::String);
  msg->data = ROBOT_URDF;
  filter->onRobotModelMsg(msg);

  cras::Cloud cloud;
  cloud.header.frame_id = filter->filtering_frame_;
  cras::CloudModifier mod(cloud);
  mod.setPointCloud2Fields(
    3,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32);
  mod.resize(12);
  cloud.width = 4;
  cloud.height = 3;
  cloud.row_step = cloud.width * cloud.point_step;

  {
    cras::CloudIter x_it(cloud, "x");
    cras::CloudIter y_it(cloud, "y");
    cras::CloudIter z_it(cloud, "z");

    *x_it = 1.5 + -1.5; *y_it = 0; *z_it = 0; ++x_it, ++y_it, ++z_it;  // pointSensor
    *x_it = 1.5 + -1.47; *y_it = 0; *z_it = 0; ++x_it, ++y_it, ++z_it;  // pointSensor2
    *x_it = 1.5 + -1.42; *y_it = 0; *z_it = 0; ++x_it, ++y_it, ++z_it;  // pointClipMin
    *x_it = 1.5 + 10; *y_it = 0; *z_it = 0; ++x_it, ++y_it, ++z_it;  // pointClipMax
    *x_it = 1.5 + 0.85; *y_it = 0.85; *z_it = 0.85; ++x_it, ++y_it, ++z_it;  // pointInBox
    *x_it = 1.5 + 1.35; *y_it = 0; *z_it = 0; ++x_it, ++y_it, ++z_it;  // pointInSphere
    *x_it = 1.5 + 0; *y_it = 0; *z_it = 0; ++x_it, ++y_it, ++z_it;  // pointInBoth
    *x_it = 1.5 + -0.25; *y_it = -2; *z_it = 2; ++x_it, ++y_it, ++z_it;  // pointShadowBox
    *x_it = 1.5 + -0.560762; *y_it = 0; *z_it = 1.83871; ++x_it, ++y_it, ++z_it;  // pointShadowSphere
    *x_it = 1.5 + 1.5; *y_it = 0; *z_it = 0; ++x_it, ++y_it, ++z_it;  // pointShadowBoth
    *x_it = 1.5 + -3; *y_it = 0; *z_it = 0; ++x_it, ++y_it, ++z_it;  // pointOutside
    *x_it = 1.5 + -4; *y_it = 0; *z_it = 0; ++x_it, ++y_it, ++z_it;  // pointOutside2
  }

  {
    rclcpp::Time now = nh->get_clock()->now();
    cloud.header.stamp = now;
    geometry_msgs::msg::TransformStamped tf;
    tf.transform.rotation.w = 1.0;
    for (double d = -5.0; d < 5.0; d += 0.1) {
      tf.header.stamp = now + rclcpp::Duration::from_seconds(d);

      tf.transform.translation.x = 0.122;
      tf.header.frame_id = "odom";
      tf.child_frame_id = "base_link";
      filter->tf_buffer_->setTransform(tf, "test");

      tf.transform.translation.x = -1.5 - 0.122;
      tf.header.frame_id = "base_link";
      tf.child_frame_id = "laser";
      filter->tf_buffer_->setTransform(tf, "test");

      tf.transform.translation.x = 0.01864 - 0.122;
      tf.header.frame_id = "base_link";
      tf.child_frame_id = "antenna";
      filter->tf_buffer_->setTransform(tf, "test");
    }
  }

  while (!filter->tf_frames_watchdog_->isReachable("antenna")) {
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }
  filter->updateTransformCache(cloud.header.stamp, cloud.header.stamp + rclcpp::Duration::from_seconds(1));

  SphereStamped::ConstSharedPtr bounding_sphere;
  geometry_msgs::msg::PolygonStamped::ConstSharedPtr bounding_box;
  OrientedBoundingBoxStamped::ConstSharedPtr oriented_bounding_box;
  geometry_msgs::msg::PolygonStamped::ConstSharedPtr local_bounding_box;
  visualization_msgs::msg::Marker::ConstSharedPtr bounding_sphere_marker;
  visualization_msgs::msg::Marker::ConstSharedPtr bounding_box_marker;
  visualization_msgs::msg::Marker::ConstSharedPtr oriented_bounding_box_marker;
  visualization_msgs::msg::Marker::ConstSharedPtr local_bounding_box_marker;
  visualization_msgs::msg::MarkerArray::ConstSharedPtr bounding_sphere_debug_marker;
  visualization_msgs::msg::MarkerArray::ConstSharedPtr bounding_box_debug_marker;
  visualization_msgs::msg::MarkerArray::ConstSharedPtr oriented_bounding_box_debug_marker;
  visualization_msgs::msg::MarkerArray::ConstSharedPtr local_bounding_box_debug_marker;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr pcl_no_bounding_sphere;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr pcl_no_bounding_box;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr pcl_no_oriented_bounding_box;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr pcl_no_local_bounding_box;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr pcl_inside;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr pcl_clip;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr pcl_shadow;
  visualization_msgs::msg::MarkerArray::ConstSharedPtr robot_model_contains_test;
  visualization_msgs::msg::MarkerArray::ConstSharedPtr robot_model_shadow_test;

  auto bounding_sphereSubscriber = nh->create_subscription<SphereStamped>(
    "/robot_bounding_sphere", 10,
    [&](const SphereStamped::ConstSharedPtr& m){bounding_sphere = m;});
  auto bounding_boxSubscriber = nh->create_subscription<geometry_msgs::msg::PolygonStamped>(
    "/robot_bounding_box", 10,
    [&](const geometry_msgs::msg::PolygonStamped::ConstSharedPtr& m){bounding_box = m;});
  auto oriented_bounding_boxSubscriber = nh->create_subscription<OrientedBoundingBoxStamped>(
    "/robot_oriented_bounding_box", 10,
    [&](const OrientedBoundingBoxStamped::ConstSharedPtr& m){oriented_bounding_box = m;});
  auto local_bounding_boxSubscriber = nh->create_subscription<geometry_msgs::msg::PolygonStamped>(
    "/robot_local_bounding_box", 10,
    [&](const geometry_msgs::msg::PolygonStamped::ConstSharedPtr& m){local_bounding_box = m;});
  auto bounding_sphere_markerSubscriber = nh->create_subscription<visualization_msgs::msg::Marker>(
    "/robot_bounding_sphere_marker", 10,
    [&](const visualization_msgs::msg::Marker::ConstSharedPtr& m){bounding_sphere_marker = m;});
  auto bounding_box_markerSubscriber = nh->create_subscription<visualization_msgs::msg::Marker>(
    "/robot_bounding_box_marker", 10,
    [&](const visualization_msgs::msg::Marker::ConstSharedPtr& m){bounding_box_marker = m;});
  auto oriented_bounding_box_markerSubscriber = nh->create_subscription<visualization_msgs::msg::Marker>(
    "/robot_oriented_bounding_box_marker", 10,
    [&](const visualization_msgs::msg::Marker::ConstSharedPtr& m){oriented_bounding_box_marker = m;});
  auto local_bounding_box_markerSubscriber = nh->create_subscription<visualization_msgs::msg::Marker>(
    "/robot_local_bounding_box_marker", 10,
    [&](const visualization_msgs::msg::Marker::ConstSharedPtr& m){local_bounding_box_marker = m;});
  auto bounding_sphere_debug_markerSubscriber = nh->create_subscription<visualization_msgs::msg::MarkerArray>(
    "/robot_bounding_sphere_debug", 10,
    [&](const visualization_msgs::msg::MarkerArray::ConstSharedPtr& m){bounding_sphere_debug_marker = m;});
  auto bounding_box_debug_markerSubscriber = nh->create_subscription<visualization_msgs::msg::MarkerArray>(
    "/robot_bounding_box_debug", 10,
    [&](const visualization_msgs::msg::MarkerArray::ConstSharedPtr& m){bounding_box_debug_marker = m;});
  auto oriented_bounding_box_debug_markerSubscriber = nh->create_subscription<visualization_msgs::msg::MarkerArray>(
    "/robot_oriented_bounding_box_debug", 10,
    [&](const visualization_msgs::msg::MarkerArray::ConstSharedPtr& m){oriented_bounding_box_debug_marker = m;});
  auto local_bounding_box_debug_markerSubscriber = nh->create_subscription<visualization_msgs::msg::MarkerArray>(
    "/robot_local_bounding_box_debug", 10,
    [&](const visualization_msgs::msg::MarkerArray::ConstSharedPtr& m){local_bounding_box_debug_marker = m;});
  auto scanPointCloudNoBoundingSphereSubscriber = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/scan_point_cloud_no_bsphere", 10,
    [&](const sensor_msgs::msg::PointCloud2::ConstSharedPtr& m){pcl_no_bounding_sphere = m;});
  auto scanPointCloudNoBoundingBoxSubscriber = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/scan_point_cloud_no_bbox", 10,
    [&](const sensor_msgs::msg::PointCloud2::ConstSharedPtr& m){pcl_no_bounding_box = m;});
  auto scanPointCloudNoOrientedBoundingBoxSubscriber = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/scan_point_cloud_no_oriented_bbox", 10,
    [&](const sensor_msgs::msg::PointCloud2::ConstSharedPtr& m){pcl_no_oriented_bounding_box = m;});
  auto scanPointCloudNoLocalBoundingBoxSubscriber = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/scan_point_cloud_no_local_bbox", 10,
    [&](const sensor_msgs::msg::PointCloud2::ConstSharedPtr& m){pcl_no_local_bounding_box = m;});
  auto debugPointCloudInsideSubscriber = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/scan_point_cloud_inside", 10,
    [&](const sensor_msgs::msg::PointCloud2::ConstSharedPtr& m){pcl_inside = m;});
  auto debugPointCloudClipSubscriber = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/scan_point_cloud_clip", 10,
    [&](const sensor_msgs::msg::PointCloud2::ConstSharedPtr& m){pcl_clip = m;});
  auto debugPointCloudShadowSubscriber = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/scan_point_cloud_shadow", 10,
    [&](const sensor_msgs::msg::PointCloud2::ConstSharedPtr& m){pcl_shadow = m;});
  auto debugContainsMarkerSubscriber = nh->create_subscription<visualization_msgs::msg::MarkerArray>(
    "/robot_model_for_contains_test", 10,
    [&](const visualization_msgs::msg::MarkerArray::ConstSharedPtr& m){robot_model_contains_test = m;});
  auto debugShadowMarkerSubscriber = nh->create_subscription<visualization_msgs::msg::MarkerArray>(
    "/robot_model_for_shadow_test", 10,
    [&](const visualization_msgs::msg::MarkerArray::ConstSharedPtr& m){robot_model_shadow_test = m;});

  std::vector<RayCastingShapeMask::MaskValue> mask;
  filter->computeMask(cloud, mask, "laser");

  ASSERT_EQ(cras::numPoints(cloud), mask.size());
  EXPECT_EQ(RayCastingShapeMask::MaskValue::CLIP, mask[0]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::CLIP, mask[1]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::CLIP, mask[2]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::CLIP, mask[3]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::INSIDE, mask[4]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::INSIDE, mask[5]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::INSIDE, mask[6]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, mask[7]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, mask[8]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, mask[9]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, mask[10]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, mask[11]);

  WAIT_FOR_MESSAGE(nh, bounding_sphere)
  WAIT_FOR_MESSAGE(nh, bounding_box)
  WAIT_FOR_MESSAGE(nh, oriented_bounding_box)
  WAIT_FOR_MESSAGE(nh, local_bounding_box)
  WAIT_FOR_MESSAGE(nh, bounding_sphere_marker)
  WAIT_FOR_MESSAGE(nh, bounding_box_marker)
  WAIT_FOR_MESSAGE(nh, oriented_bounding_box_marker)
  WAIT_FOR_MESSAGE(nh, local_bounding_box_marker)
  WAIT_FOR_MESSAGE(nh, bounding_sphere_debug_marker)
  WAIT_FOR_MESSAGE(nh, bounding_box_debug_marker)
  WAIT_FOR_MESSAGE(nh, oriented_bounding_box_debug_marker)
  WAIT_FOR_MESSAGE(nh, local_bounding_box_debug_marker)
  WAIT_FOR_MESSAGE(nh, pcl_no_bounding_sphere)
  WAIT_FOR_MESSAGE(nh, pcl_no_bounding_box)
  WAIT_FOR_MESSAGE(nh, pcl_no_oriented_bounding_box)
  WAIT_FOR_MESSAGE(nh, pcl_no_local_bounding_box)
  WAIT_FOR_MESSAGE(nh, pcl_inside)
  WAIT_FOR_MESSAGE(nh, pcl_clip)
  WAIT_FOR_MESSAGE(nh, pcl_shadow)
  WAIT_FOR_MESSAGE(nh, robot_model_contains_test)
  WAIT_FOR_MESSAGE(nh, robot_model_shadow_test)

  EXPECT_EQ("laser", bounding_sphere->header.frame_id);
  EXPECT_EQ(cloud.header.stamp, bounding_sphere->header.stamp);
  EXPECT_NEAR(sqrt(3) * 0.92, bounding_sphere->sphere.radius, 1e-6);
  EXPECT_NEAR(1.5, bounding_sphere->sphere.center.x, 1e-6);
  EXPECT_DOUBLE_EQ(0, bounding_sphere->sphere.center.y);
  EXPECT_DOUBLE_EQ(0, bounding_sphere->sphere.center.z);
  EXPECT_EQ("laser", bounding_sphere_marker->header.frame_id);
  EXPECT_EQ(cloud.header.stamp, bounding_sphere_marker->header.stamp);
  EXPECT_NEAR(2 * sqrt(3) * 0.92, bounding_sphere_marker->scale.x, 1e-6);
  EXPECT_NEAR(2 * sqrt(3) * 0.92, bounding_sphere_marker->scale.y, 1e-6);
  EXPECT_NEAR(2 * sqrt(3) * 0.92, bounding_sphere_marker->scale.z, 1e-6);
  EXPECT_NEAR(1.5, bounding_sphere_marker->pose.position.x, 1e-6);
  EXPECT_NEAR(0, bounding_sphere_marker->pose.position.y, 1e-6);
  EXPECT_NEAR(0, bounding_sphere_marker->pose.position.z, 1e-6);
  EXPECT_NEAR(0, bounding_sphere_marker->pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, bounding_sphere_marker->pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, bounding_sphere_marker->pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, bounding_sphere_marker->pose.orientation.w, 1e-6);
  EXPECT_LT(0, bounding_sphere_marker->color.a);
  // make sure the marker has some color
  EXPECT_LT(0, bounding_sphere_marker->color.r + bounding_sphere_marker->color.g + bounding_sphere_marker->color.b);
  EXPECT_EQ(visualization_msgs::msg::Marker::SPHERE, bounding_sphere_marker->type);
  EXPECT_EQ(visualization_msgs::msg::Marker::ADD, bounding_sphere_marker->action);
  EXPECT_EQ("bounding_sphere", bounding_sphere_marker->ns);
  EXPECT_EQ(1, bounding_sphere_marker->frame_locked);

  EXPECT_EQ("laser", bounding_box->header.frame_id);
  EXPECT_EQ(cloud.header.stamp, bounding_box->header.stamp);
  ASSERT_EQ(2, bounding_box->polygon.points.size());
  EXPECT_NEAR(1.5 - 1.55, bounding_box->polygon.points[0].x, 1e-5);
  EXPECT_NEAR(-1.375, bounding_box->polygon.points[0].y, 1e-6);
  EXPECT_NEAR(-1.375, bounding_box->polygon.points[0].z, 1e-6);
  EXPECT_NEAR(1.5 + 1.375, bounding_box->polygon.points[1].x, 1e-5);
  EXPECT_NEAR(1.375, bounding_box->polygon.points[1].y, 1e-6);
  EXPECT_NEAR(1.375, bounding_box->polygon.points[1].z, 1e-6);
  EXPECT_EQ("laser", bounding_box_marker->header.frame_id);
  EXPECT_EQ(cloud.header.stamp, bounding_box_marker->header.stamp);
  EXPECT_NEAR(2.925, bounding_box_marker->scale.x, 1e-5);
  EXPECT_NEAR(2.75, bounding_box_marker->scale.y, 1e-5);
  EXPECT_NEAR(2.75, bounding_box_marker->scale.z, 1e-5);
  EXPECT_NEAR(1.5 - 0.0875, bounding_box_marker->pose.position.x, 1e-6);
  EXPECT_NEAR(0, bounding_box_marker->pose.position.y, 1e-6);
  EXPECT_NEAR(0, bounding_box_marker->pose.position.z, 1e-6);
  EXPECT_NEAR(0, bounding_box_marker->pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, bounding_box_marker->pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, bounding_box_marker->pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, bounding_box_marker->pose.orientation.w, 1e-6);
  EXPECT_LT(0, bounding_box_marker->color.a);
  // make sure the marker has some color
  EXPECT_LT(0, bounding_box_marker->color.r + bounding_box_marker->color.g + bounding_box_marker->color.b);
  EXPECT_EQ(visualization_msgs::msg::Marker::CUBE, bounding_box_marker->type);
  EXPECT_EQ(visualization_msgs::msg::Marker::ADD, bounding_box_marker->action);
  EXPECT_EQ("bounding_box", bounding_box_marker->ns);
  EXPECT_EQ(1, bounding_box_marker->frame_locked);

  EXPECT_EQ("laser", oriented_bounding_box->header.frame_id);
  EXPECT_EQ(cloud.header.stamp, oriented_bounding_box->header.stamp);
  EXPECT_NEAR(2.925, oriented_bounding_box->obb.extents.x, 1e-5);
  EXPECT_NEAR(2.75, oriented_bounding_box->obb.extents.y, 1e-5);
  EXPECT_NEAR(2.75, oriented_bounding_box->obb.extents.z, 1e-5);
  EXPECT_NEAR(1.5 - 0.0875, oriented_bounding_box->obb.pose.translation.x, 1e-5);
  EXPECT_NEAR(0, oriented_bounding_box->obb.pose.translation.y, 1e-6);
  EXPECT_NEAR(0, oriented_bounding_box->obb.pose.translation.z, 1e-6);
  EXPECT_NEAR(0, oriented_bounding_box->obb.pose.rotation.x, 1e-6);
  EXPECT_NEAR(0, oriented_bounding_box->obb.pose.rotation.y, 1e-6);
  EXPECT_NEAR(0, oriented_bounding_box->obb.pose.rotation.z, 1e-6);
  EXPECT_NEAR(1, oriented_bounding_box->obb.pose.rotation.w, 1e-6);
  EXPECT_EQ("laser", oriented_bounding_box_marker->header.frame_id);
  EXPECT_EQ(cloud.header.stamp, oriented_bounding_box_marker->header.stamp);
  EXPECT_NEAR(2.925, oriented_bounding_box_marker->scale.x, 1e-5);
  EXPECT_NEAR(2.75, oriented_bounding_box_marker->scale.y, 1e-5);
  EXPECT_NEAR(2.75, oriented_bounding_box_marker->scale.z, 1e-5);
  EXPECT_NEAR(1.5 - 0.0875, oriented_bounding_box_marker->pose.position.x, 1e-6);
  EXPECT_NEAR(0, oriented_bounding_box_marker->pose.position.y, 1e-6);
  EXPECT_NEAR(0, oriented_bounding_box_marker->pose.position.z, 1e-6);
  EXPECT_NEAR(0, oriented_bounding_box_marker->pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, oriented_bounding_box_marker->pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, oriented_bounding_box_marker->pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, oriented_bounding_box_marker->pose.orientation.w, 1e-6);
  EXPECT_LT(0, oriented_bounding_box_marker->color.a);
  // make sure the marker has some color
  EXPECT_LT(
    0, oriented_bounding_box_marker->color.r + oriented_bounding_box_marker->color.g +
    oriented_bounding_box_marker->color.b);
  EXPECT_EQ(visualization_msgs::msg::Marker::CUBE, oriented_bounding_box_marker->type);
  EXPECT_EQ(visualization_msgs::msg::Marker::ADD, oriented_bounding_box_marker->action);
  EXPECT_EQ("oriented_bounding_box", oriented_bounding_box_marker->ns);
  EXPECT_EQ(1, oriented_bounding_box_marker->frame_locked);

  EXPECT_EQ("base_link", local_bounding_box->header.frame_id);
  EXPECT_EQ(cloud.header.stamp, local_bounding_box->header.stamp);
  ASSERT_EQ(2, local_bounding_box->polygon.points.size());
  EXPECT_NEAR(-1.55 - 0.122, local_bounding_box->polygon.points[0].x, 1e-5);
  EXPECT_NEAR(-1.375, local_bounding_box->polygon.points[0].y, 1e-6);
  EXPECT_NEAR(-1.375, local_bounding_box->polygon.points[0].z, 1e-6);
  EXPECT_NEAR(1.375 - 0.122, local_bounding_box->polygon.points[1].x, 1e-5);
  EXPECT_NEAR(1.375, local_bounding_box->polygon.points[1].y, 1e-6);
  EXPECT_NEAR(1.375, local_bounding_box->polygon.points[1].z, 1e-6);
  EXPECT_EQ("base_link", local_bounding_box_marker->header.frame_id);
  EXPECT_EQ(cloud.header.stamp, local_bounding_box_marker->header.stamp);
  EXPECT_NEAR(2.925, local_bounding_box_marker->scale.x, 1e-5);
  EXPECT_NEAR(2.75, local_bounding_box_marker->scale.y, 1e-5);
  EXPECT_NEAR(2.75, local_bounding_box_marker->scale.z, 1e-5);
  EXPECT_NEAR(-0.0875 - 0.122, local_bounding_box_marker->pose.position.x, 1e-6);
  EXPECT_NEAR(0, local_bounding_box_marker->pose.position.y, 1e-6);
  EXPECT_NEAR(0, local_bounding_box_marker->pose.position.z, 1e-6);
  EXPECT_NEAR(0, local_bounding_box_marker->pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, local_bounding_box_marker->pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, local_bounding_box_marker->pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, local_bounding_box_marker->pose.orientation.w, 1e-6);
  EXPECT_LT(0, local_bounding_box_marker->color.a);
  // make sure the marker has some color
  EXPECT_LT(0, local_bounding_box_marker->color.r + local_bounding_box_marker->color.g +
    local_bounding_box_marker->color.b);
  EXPECT_EQ(visualization_msgs::msg::Marker::CUBE, local_bounding_box_marker->type);
  EXPECT_EQ(visualization_msgs::msg::Marker::ADD, local_bounding_box_marker->action);
  EXPECT_EQ("local_bounding_box", local_bounding_box_marker->ns);
  EXPECT_EQ(1, local_bounding_box_marker->frame_locked);

  ASSERT_EQ(12, cras::numPoints(*pcl_no_bounding_sphere));
  EXPECT_EQ(cloud.header.frame_id, pcl_no_bounding_sphere->header.frame_id);
  EXPECT_EQ(cloud.header.stamp, pcl_no_bounding_sphere->header.stamp);
  EXPECT_EQ(cloud.height, pcl_no_bounding_sphere->height);
  EXPECT_EQ(cloud.width, pcl_no_bounding_sphere->width);
  EXPECT_EQ(cloud.fields, pcl_no_bounding_sphere->fields);
  EXPECT_EQ(cloud.point_step, pcl_no_bounding_sphere->point_step);
  EXPECT_EQ(cloud.row_step, pcl_no_bounding_sphere->row_step);
  cras::CloudConstIter x_it(*pcl_no_bounding_sphere, "x");
  EXPECT_NAN(*x_it); ++x_it;  // pointSensor
  EXPECT_NAN(*x_it); ++x_it;  // pointSensor2
  EXPECT_NAN(*x_it); ++x_it;  // pointClipMin
  EXPECT_NEAR(1.5 + 10.0, *x_it, 1e-6); ++x_it;  // pointClipMax
  EXPECT_NAN(*x_it); ++x_it;  // pointInBox
  EXPECT_NAN(*x_it); ++x_it;  // pointInSphere
  EXPECT_NAN(*x_it); ++x_it;  // pointInBoth
  EXPECT_NEAR(1.5 + -0.25, *x_it, 1e-6); ++x_it;  // pointShadowBox
  EXPECT_NEAR(1.5 + -0.560762, *x_it, 1e-6); ++x_it;  // pointShadowSphere
  EXPECT_NAN(*x_it); ++x_it;  // pointShadowBoth
  EXPECT_NEAR(1.5 + -3.0, *x_it, 1e-6); ++x_it;  // pointOutside
  EXPECT_NEAR(1.5 + -4.0, *x_it, 1e-6); ++x_it;  // pointOutside2

  ASSERT_EQ(12, cras::numPoints(*pcl_no_bounding_box));
  EXPECT_EQ(cloud.header.frame_id, pcl_no_bounding_box->header.frame_id);
  EXPECT_EQ(cloud.header.stamp, pcl_no_bounding_box->header.stamp);
  EXPECT_EQ(cloud.height, pcl_no_bounding_box->height);
  EXPECT_EQ(cloud.width, pcl_no_bounding_box->width);
  EXPECT_EQ(cloud.fields, pcl_no_bounding_box->fields);
  EXPECT_EQ(cloud.point_step, pcl_no_bounding_box->point_step);
  EXPECT_EQ(cloud.row_step, pcl_no_bounding_box->row_step);
  x_it = cras::CloudConstIter(*pcl_no_bounding_box, "x");
  EXPECT_NAN(*x_it); ++x_it;  // pointSensor
  EXPECT_NAN(*x_it); ++x_it;  // pointSensor2
  EXPECT_NAN(*x_it); ++x_it;  // pointClipMin
  EXPECT_NEAR(1.5 + 10., *x_it, 1e-6); ++x_it;  // pointClipMax
  EXPECT_NAN(*x_it); ++x_it;  // pointInBox
  EXPECT_NAN(*x_it); ++x_it;  // pointInSphere
  EXPECT_NAN(*x_it); ++x_it;  // pointInBoth
  EXPECT_NEAR(1.5 + -0.25, *x_it, 1e-6); ++x_it;  // pointShadowBox
  EXPECT_NEAR(1.5 + -0.560762, *x_it, 1e-6); ++x_it;  // pointShadowSphere
  EXPECT_NEAR(1.5 + 1.5, *x_it, 1e-6); ++x_it;  // pointShadowBoth
  EXPECT_NEAR(1.5 + -3.0, *x_it, 1e-6); ++x_it;  // pointOutside

  ASSERT_EQ(12, cras::numPoints(*pcl_no_oriented_bounding_box));
  EXPECT_EQ(cloud.header.frame_id, pcl_no_oriented_bounding_box->header.frame_id);
  EXPECT_EQ(cloud.header.stamp, pcl_no_oriented_bounding_box->header.stamp);
  EXPECT_EQ(cloud.height, pcl_no_oriented_bounding_box->height);
  EXPECT_EQ(cloud.width, pcl_no_oriented_bounding_box->width);
  EXPECT_EQ(cloud.fields, pcl_no_oriented_bounding_box->fields);
  EXPECT_EQ(cloud.point_step, pcl_no_oriented_bounding_box->point_step);
  EXPECT_EQ(cloud.row_step, pcl_no_oriented_bounding_box->row_step);
  x_it = cras::CloudConstIter(*pcl_no_oriented_bounding_box, "x");
  EXPECT_NAN(*x_it); ++x_it;  // pointSensor
  EXPECT_NAN(*x_it); ++x_it;  // pointSensor2
  EXPECT_NAN(*x_it); ++x_it;  // pointClipMin
  EXPECT_NEAR(1.5 + 10., *x_it, 1e-6); ++x_it;  // pointClipMax
  EXPECT_NAN(*x_it); ++x_it;  // pointInBox
  EXPECT_NAN(*x_it); ++x_it;  // pointInSphere
  EXPECT_NAN(*x_it); ++x_it;  // pointInBoth
  EXPECT_NEAR(1.5 + -0.25, *x_it, 1e-6); ++x_it;  // pointShadowBox
  EXPECT_NEAR(1.5 + -0.560762, *x_it, 1e-6); ++x_it;  // pointShadowSphere
  EXPECT_NEAR(1.5 + 1.5, *x_it, 1e-6); ++x_it;  // pointShadowBoth
  EXPECT_NEAR(1.5 + -3.0, *x_it, 1e-6); ++x_it;  // pointOutside

  ASSERT_EQ(12, cras::numPoints(*pcl_no_local_bounding_box));
  EXPECT_EQ(cloud.header.frame_id, pcl_no_local_bounding_box->header.frame_id);
  EXPECT_EQ(cloud.header.stamp, pcl_no_local_bounding_box->header.stamp);
  EXPECT_EQ(cloud.height, pcl_no_local_bounding_box->height);
  EXPECT_EQ(cloud.width, pcl_no_local_bounding_box->width);
  EXPECT_EQ(cloud.fields, pcl_no_local_bounding_box->fields);
  EXPECT_EQ(cloud.point_step, pcl_no_local_bounding_box->point_step);
  EXPECT_EQ(cloud.row_step, pcl_no_local_bounding_box->row_step);
  x_it = cras::CloudConstIter(*pcl_no_local_bounding_box, "x");
  EXPECT_NAN(*x_it); ++x_it;  // pointSensor
  EXPECT_NAN(*x_it); ++x_it;  // pointSensor2
  EXPECT_NAN(*x_it); ++x_it;  // pointClipMin
  EXPECT_NEAR(1.5 + 10., *x_it, 1e-6); ++x_it;  // pointClipMax
  EXPECT_NAN(*x_it); ++x_it;  // pointInBox
  EXPECT_NAN(*x_it); ++x_it;  // pointInSphere
  EXPECT_NAN(*x_it); ++x_it;  // pointInBoth
  EXPECT_NEAR(1.5 + -0.25, *x_it, 1e-6); ++x_it;  // pointShadowBox
  EXPECT_NEAR(1.5 + -0.560762, *x_it, 1e-6); ++x_it;  // pointShadowSphere
  EXPECT_NEAR(1.5 + 1.5, *x_it, 1e-6); ++x_it;  // pointShadowBoth
  EXPECT_NEAR(1.5 + -3.0, *x_it, 1e-6); ++x_it;  // pointOutside

  ASSERT_EQ(12, cras::numPoints(*pcl_inside));
  EXPECT_EQ(cloud.header.frame_id, pcl_inside->header.frame_id);
  EXPECT_EQ(cloud.header.stamp, pcl_inside->header.stamp);
  EXPECT_EQ(cloud.height, pcl_inside->height);
  EXPECT_EQ(cloud.width, pcl_inside->width);
  EXPECT_EQ(cloud.fields, pcl_inside->fields);
  EXPECT_EQ(cloud.point_step, pcl_inside->point_step);
  EXPECT_EQ(cloud.row_step, pcl_inside->row_step);
  x_it = cras::CloudConstIter(*pcl_inside, "x");
  EXPECT_NAN(*x_it); ++x_it;  // pointSensor
  EXPECT_NAN(*x_it); ++x_it;  // pointSensor2
  EXPECT_NAN(*x_it); ++x_it;  // pointClipMin
  EXPECT_NAN(*x_it); ++x_it;  // pointClipMax
  EXPECT_NEAR(1.5 + 0.85, *x_it, 1e-6); ++x_it;  // pointInBox
  EXPECT_NEAR(1.5 + 1.35, *x_it, 1e-6); ++x_it;  // pointInSphere
  EXPECT_NEAR(1.5 + 0.0, *x_it, 1e-6); ++x_it;  // pointInBoth
  EXPECT_NAN(*x_it); ++x_it;  // pointShadowBox
  EXPECT_NAN(*x_it); ++x_it;  // pointShadowSphere
  EXPECT_NAN(*x_it); ++x_it;  // pointShadowBoth
  EXPECT_NAN(*x_it); ++x_it;  // pointOutside

  ASSERT_EQ(12, cras::numPoints(*pcl_clip));
  EXPECT_EQ(cloud.header.frame_id, pcl_clip->header.frame_id);
  EXPECT_EQ(cloud.header.stamp, pcl_clip->header.stamp);
  EXPECT_EQ(cloud.height, pcl_clip->height);
  EXPECT_EQ(cloud.width, pcl_clip->width);
  EXPECT_EQ(cloud.fields, pcl_clip->fields);
  EXPECT_EQ(cloud.point_step, pcl_clip->point_step);
  EXPECT_EQ(cloud.row_step, pcl_clip->row_step);
  x_it = cras::CloudConstIter(*pcl_clip, "x");
  EXPECT_NEAR(1.5 + -1.5, *x_it, 1e-6); ++x_it;  // pointSensor
  EXPECT_NEAR(1.5 + -1.47, *x_it, 1e-6); ++x_it;  // pointSensor2
  EXPECT_NEAR(1.5 + -1.42, *x_it, 1e-6); ++x_it;  // pointClipMin
  EXPECT_NEAR(1.5 + 10., *x_it, 1e-6); ++x_it;  // pointClipMax
  EXPECT_NAN(*x_it); ++x_it;  // pointInBox
  EXPECT_NAN(*x_it); ++x_it;  // pointInSphere
  EXPECT_NAN(*x_it); ++x_it;  // pointInBoth
  EXPECT_NAN(*x_it); ++x_it;  // pointShadowBox
  EXPECT_NAN(*x_it); ++x_it;  // pointShadowSphere
  EXPECT_NAN(*x_it); ++x_it;  // pointShadowBoth
  EXPECT_NAN(*x_it); ++x_it;  // pointOutside

  ASSERT_EQ(12, cras::numPoints(*pcl_shadow));
  EXPECT_EQ(cloud.header.frame_id, pcl_shadow->header.frame_id);
  EXPECT_EQ(cloud.header.stamp, pcl_shadow->header.stamp);
  EXPECT_EQ(cloud.height, pcl_shadow->height);
  EXPECT_EQ(cloud.width, pcl_shadow->width);
  EXPECT_EQ(cloud.fields, pcl_shadow->fields);
  EXPECT_EQ(cloud.point_step, pcl_shadow->point_step);
  EXPECT_EQ(cloud.row_step, pcl_shadow->row_step);
  x_it = cras::CloudConstIter(*pcl_shadow, "x");
  EXPECT_NAN(*x_it); ++x_it;  // pointSensor
  EXPECT_NAN(*x_it); ++x_it;  // pointSensor2
  EXPECT_NAN(*x_it); ++x_it;  // pointClipMin
  EXPECT_NAN(*x_it); ++x_it;  // pointClipMax
  EXPECT_NAN(*x_it); ++x_it;  // pointInBox
  EXPECT_NAN(*x_it); ++x_it;  // pointInSphere
  EXPECT_NAN(*x_it); ++x_it;  // pointInBoth
  EXPECT_NEAR(1.5 + -0.25, *x_it, 1e-6); ++x_it;  // pointShadowBox
  EXPECT_NEAR(1.5 + -0.560762, *x_it, 1e-6); ++x_it;  // pointShadowSphere
  EXPECT_NEAR(1.5 + 1.5, *x_it, 1e-6); ++x_it;  // pointShadowBoth
  EXPECT_NAN(*x_it); ++x_it;  // pointOutside

  ASSERT_EQ(2, bounding_sphere_debug_marker->markers.size());
  EXPECT_EQ("laser", bounding_sphere_debug_marker->markers[0].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, bounding_sphere_debug_marker->markers[0].header.stamp);
  EXPECT_NEAR(2 * sqrt(3) * 0.92, bounding_sphere_debug_marker->markers[0].scale.x, 1e-6);
  EXPECT_NEAR(2 * sqrt(3) * 0.92, bounding_sphere_debug_marker->markers[0].scale.y, 1e-6);
  EXPECT_NEAR(2 * sqrt(3) * 0.92, bounding_sphere_debug_marker->markers[0].scale.z, 1e-6);
  EXPECT_NEAR(1.5, bounding_sphere_debug_marker->markers[0].pose.position.x, 1e-6);
  EXPECT_NEAR(0, bounding_sphere_debug_marker->markers[0].pose.position.y, 1e-6);
  EXPECT_NEAR(0, bounding_sphere_debug_marker->markers[0].pose.position.z, 1e-6);
  EXPECT_NEAR(0, bounding_sphere_debug_marker->markers[0].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, bounding_sphere_debug_marker->markers[0].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, bounding_sphere_debug_marker->markers[0].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, bounding_sphere_debug_marker->markers[0].pose.orientation.w, 1e-6);
  EXPECT_LT(0, bounding_sphere_debug_marker->markers[0].color.a);
  // make sure the marker has some color
  EXPECT_LT(
    0,
    bounding_sphere_debug_marker->markers[0].color.r + bounding_sphere_debug_marker->markers[0].color.g +
    bounding_sphere_debug_marker->markers[0].color.b);
  EXPECT_EQ(visualization_msgs::msg::Marker::SPHERE, bounding_sphere_debug_marker->markers[0].type);
  EXPECT_EQ(visualization_msgs::msg::Marker::ADD, bounding_sphere_debug_marker->markers[0].action);
  EXPECT_FALSE(bounding_sphere_debug_marker->markers[0].ns.empty());
  EXPECT_EQ(1, bounding_sphere_debug_marker->markers[0].frame_locked);
  EXPECT_EQ("laser", bounding_sphere_debug_marker->markers[1].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, bounding_sphere_debug_marker->markers[1].header.stamp);
  EXPECT_NEAR(0.1 * sqrt(3), bounding_sphere_debug_marker->markers[1].scale.x, 1e-5);
  EXPECT_NEAR(0.1 * sqrt(3), bounding_sphere_debug_marker->markers[1].scale.y, 1e-5);
  EXPECT_NEAR(0.1 * sqrt(3), bounding_sphere_debug_marker->markers[1].scale.z, 1e-5);
  EXPECT_NEAR(1.5 - 1.5, bounding_sphere_debug_marker->markers[1].pose.position.x, 1e-5);
  EXPECT_NEAR(0, bounding_sphere_debug_marker->markers[1].pose.position.y, 1e-6);
  EXPECT_NEAR(0, bounding_sphere_debug_marker->markers[1].pose.position.z, 1e-6);
  EXPECT_NEAR(0, bounding_sphere_debug_marker->markers[1].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, bounding_sphere_debug_marker->markers[1].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, bounding_sphere_debug_marker->markers[1].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, bounding_sphere_debug_marker->markers[1].pose.orientation.w, 1e-6);
  EXPECT_LT(0, bounding_sphere_debug_marker->markers[1].color.a);
  // make sure the marker has some color
  EXPECT_LT(
    0,
    bounding_sphere_debug_marker->markers[1].color.r + bounding_sphere_debug_marker->markers[1].color.g +
    bounding_sphere_debug_marker->markers[1].color.b);
  EXPECT_EQ(visualization_msgs::msg::Marker::SPHERE, bounding_sphere_debug_marker->markers[1].type);
  EXPECT_EQ(visualization_msgs::msg::Marker::ADD, bounding_sphere_debug_marker->markers[1].action);
  EXPECT_FALSE(bounding_sphere_debug_marker->markers[1].ns.empty());
  EXPECT_EQ(1, bounding_sphere_debug_marker->markers[1].frame_locked);

  EXPECT_EQ(4, bounding_box_debug_marker->markers.size());
  EXPECT_EQ("laser", bounding_box_debug_marker->markers[0].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, bounding_box_debug_marker->markers[0].header.stamp);
  EXPECT_NEAR(2.75, bounding_box_debug_marker->markers[0].scale.x, 1e-5);
  EXPECT_NEAR(2.75, bounding_box_debug_marker->markers[0].scale.y, 1e-5);
  EXPECT_NEAR(2.75, bounding_box_debug_marker->markers[0].scale.z, 1e-5);
  EXPECT_NEAR(1.5, bounding_box_debug_marker->markers[0].pose.position.x, 1e-6);
  EXPECT_NEAR(0, bounding_box_debug_marker->markers[0].pose.position.y, 1e-6);
  EXPECT_NEAR(0, bounding_box_debug_marker->markers[0].pose.position.z, 1e-6);
  EXPECT_NEAR(0, bounding_box_debug_marker->markers[0].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, bounding_box_debug_marker->markers[0].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, bounding_box_debug_marker->markers[0].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, bounding_box_debug_marker->markers[0].pose.orientation.w, 1e-6);
  EXPECT_LT(0, bounding_box_debug_marker->markers[0].color.a);
  // make sure the marker has some color
  EXPECT_LT(
    0,
    bounding_box_debug_marker->markers[0].color.r + bounding_box_debug_marker->markers[0].color.g +
    bounding_box_debug_marker->markers[0].color.b);
  EXPECT_EQ(visualization_msgs::msg::Marker::CUBE, bounding_box_debug_marker->markers[0].type);
  EXPECT_EQ(visualization_msgs::msg::Marker::ADD, bounding_box_debug_marker->markers[0].action);
  EXPECT_FALSE(bounding_box_debug_marker->markers[0].ns.empty());
  EXPECT_EQ(1, bounding_box_debug_marker->markers[0].frame_locked);
  EXPECT_EQ("laser", bounding_box_debug_marker->markers[1].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, bounding_box_debug_marker->markers[1].header.stamp);
  EXPECT_NEAR(1.84, bounding_box_debug_marker->markers[1].scale.x, 1e-5);
  EXPECT_NEAR(1.84, bounding_box_debug_marker->markers[1].scale.y, 1e-5);
  EXPECT_NEAR(1.84, bounding_box_debug_marker->markers[1].scale.z, 1e-5);
  EXPECT_NEAR(1.5, bounding_box_debug_marker->markers[1].pose.position.x, 1e-6);
  EXPECT_NEAR(0, bounding_box_debug_marker->markers[1].pose.position.y, 1e-6);
  EXPECT_NEAR(0, bounding_box_debug_marker->markers[1].pose.position.z, 1e-6);
  EXPECT_NEAR(0, bounding_box_debug_marker->markers[1].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, bounding_box_debug_marker->markers[1].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, bounding_box_debug_marker->markers[1].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, bounding_box_debug_marker->markers[1].pose.orientation.w, 1e-6);
  EXPECT_LT(0, bounding_box_debug_marker->markers[1].color.a);
  // make sure the marker has some color
  EXPECT_LT(
    0,
    bounding_box_debug_marker->markers[1].color.r + bounding_box_debug_marker->markers[1].color.g +
    bounding_box_debug_marker->markers[1].color.b);
  EXPECT_EQ(visualization_msgs::msg::Marker::CUBE, bounding_box_debug_marker->markers[1].type);
  EXPECT_EQ(visualization_msgs::msg::Marker::ADD, bounding_box_debug_marker->markers[1].action);
  EXPECT_FALSE(bounding_box_debug_marker->markers[1].ns.empty());
  EXPECT_EQ(1, bounding_box_debug_marker->markers[1].frame_locked);
  EXPECT_EQ("laser", bounding_box_debug_marker->markers[2].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, bounding_box_debug_marker->markers[2].header.stamp);
  EXPECT_NEAR(2.5, bounding_box_debug_marker->markers[2].scale.x, 1e-4);
  EXPECT_NEAR(2.5, bounding_box_debug_marker->markers[2].scale.y, 1e-4);
  EXPECT_NEAR(2.5, bounding_box_debug_marker->markers[2].scale.z, 1e-4);
  EXPECT_NEAR(1.5 + 0.122 - 0.1, bounding_box_debug_marker->markers[2].pose.position.x, 1e-6);
  EXPECT_NEAR(0, bounding_box_debug_marker->markers[2].pose.position.y, 1e-6);
  EXPECT_NEAR(0, bounding_box_debug_marker->markers[2].pose.position.z, 1e-6);
  EXPECT_NEAR(0, bounding_box_debug_marker->markers[2].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, bounding_box_debug_marker->markers[2].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, bounding_box_debug_marker->markers[2].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, bounding_box_debug_marker->markers[2].pose.orientation.w, 1e-6);
  EXPECT_LT(0, bounding_box_debug_marker->markers[2].color.a);
  // make sure the marker has some color
  EXPECT_LT(
    0,
    bounding_box_debug_marker->markers[2].color.r + bounding_box_debug_marker->markers[2].color.g +
    bounding_box_debug_marker->markers[2].color.b);
  EXPECT_EQ(visualization_msgs::msg::Marker::CUBE, bounding_box_debug_marker->markers[2].type);
  EXPECT_EQ(visualization_msgs::msg::Marker::ADD, bounding_box_debug_marker->markers[2].action);
  EXPECT_FALSE(bounding_box_debug_marker->markers[2].ns.empty());
  EXPECT_EQ(1, bounding_box_debug_marker->markers[2].frame_locked);
  EXPECT_EQ("laser", bounding_box_debug_marker->markers[3].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, bounding_box_debug_marker->markers[3].header.stamp);
  EXPECT_NEAR(0.1, bounding_box_debug_marker->markers[3].scale.x, 1e-5);
  EXPECT_NEAR(0.1, bounding_box_debug_marker->markers[3].scale.y, 1e-5);
  EXPECT_NEAR(0.1, bounding_box_debug_marker->markers[3].scale.z, 1e-5);
  EXPECT_NEAR(1.5 - 1.5, bounding_box_debug_marker->markers[3].pose.position.x, 1e-6);
  EXPECT_NEAR(0, bounding_box_debug_marker->markers[3].pose.position.y, 1e-6);
  EXPECT_NEAR(0, bounding_box_debug_marker->markers[3].pose.position.z, 1e-6);
  EXPECT_NEAR(0, bounding_box_debug_marker->markers[3].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, bounding_box_debug_marker->markers[3].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, bounding_box_debug_marker->markers[3].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, bounding_box_debug_marker->markers[3].pose.orientation.w, 1e-6);
  EXPECT_LT(0, bounding_box_debug_marker->markers[3].color.a);
  // make sure the marker has some color
  EXPECT_LT(
    0,
    bounding_box_debug_marker->markers[3].color.r + bounding_box_debug_marker->markers[3].color.g +
    bounding_box_debug_marker->markers[3].color.b);
  EXPECT_EQ(visualization_msgs::msg::Marker::CUBE, bounding_box_debug_marker->markers[3].type);
  EXPECT_EQ(visualization_msgs::msg::Marker::ADD, bounding_box_debug_marker->markers[3].action);
  EXPECT_FALSE(bounding_box_debug_marker->markers[3].ns.empty());
  EXPECT_EQ(1, bounding_box_debug_marker->markers[3].frame_locked);

  EXPECT_EQ(4, oriented_bounding_box_debug_marker->markers.size());
  EXPECT_EQ("laser", oriented_bounding_box_debug_marker->markers[0].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, oriented_bounding_box_debug_marker->markers[0].header.stamp);
  EXPECT_NEAR(2.75, oriented_bounding_box_debug_marker->markers[0].scale.x, 1e-5);
  EXPECT_NEAR(2.75, oriented_bounding_box_debug_marker->markers[0].scale.y, 1e-5);
  EXPECT_NEAR(2.75, oriented_bounding_box_debug_marker->markers[0].scale.z, 1e-5);
  EXPECT_NEAR(1.5, oriented_bounding_box_debug_marker->markers[0].pose.position.x, 1e-6);
  EXPECT_NEAR(0, oriented_bounding_box_debug_marker->markers[0].pose.position.y, 1e-6);
  EXPECT_NEAR(0, oriented_bounding_box_debug_marker->markers[0].pose.position.z, 1e-6);
  EXPECT_NEAR(0, oriented_bounding_box_debug_marker->markers[0].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, oriented_bounding_box_debug_marker->markers[0].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, oriented_bounding_box_debug_marker->markers[0].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, oriented_bounding_box_debug_marker->markers[0].pose.orientation.w, 1e-6);
  EXPECT_LT(0, oriented_bounding_box_debug_marker->markers[0].color.a);
  // make sure the marker has some color
  EXPECT_LT(
    0,
    oriented_bounding_box_debug_marker->markers[0].color.r + oriented_bounding_box_debug_marker->markers[0].color.g +
    oriented_bounding_box_debug_marker->markers[0].color.b);
  EXPECT_EQ(visualization_msgs::msg::Marker::CUBE, oriented_bounding_box_debug_marker->markers[0].type);
  EXPECT_EQ(visualization_msgs::msg::Marker::ADD, oriented_bounding_box_debug_marker->markers[0].action);
  EXPECT_FALSE(oriented_bounding_box_debug_marker->markers[0].ns.empty());
  EXPECT_EQ(1, oriented_bounding_box_debug_marker->markers[0].frame_locked);
  EXPECT_EQ("laser", oriented_bounding_box_debug_marker->markers[1].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, oriented_bounding_box_debug_marker->markers[1].header.stamp);
  EXPECT_NEAR(1.84, oriented_bounding_box_debug_marker->markers[1].scale.x, 1e-5);
  EXPECT_NEAR(1.84, oriented_bounding_box_debug_marker->markers[1].scale.y, 1e-5);
  EXPECT_NEAR(1.84, oriented_bounding_box_debug_marker->markers[1].scale.z, 1e-5);
  EXPECT_NEAR(1.5, oriented_bounding_box_debug_marker->markers[1].pose.position.x, 1e-6);
  EXPECT_NEAR(0, oriented_bounding_box_debug_marker->markers[1].pose.position.y, 1e-6);
  EXPECT_NEAR(0, oriented_bounding_box_debug_marker->markers[1].pose.position.z, 1e-6);
  EXPECT_NEAR(0, oriented_bounding_box_debug_marker->markers[1].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, oriented_bounding_box_debug_marker->markers[1].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, oriented_bounding_box_debug_marker->markers[1].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, oriented_bounding_box_debug_marker->markers[1].pose.orientation.w, 1e-6);
  EXPECT_LT(0, oriented_bounding_box_debug_marker->markers[1].color.a);
  // make sure the marker has some color
  EXPECT_LT(
    0,
    oriented_bounding_box_debug_marker->markers[1].color.r + oriented_bounding_box_debug_marker->markers[1].color.g +
    oriented_bounding_box_debug_marker->markers[1].color.b);
  EXPECT_EQ(visualization_msgs::msg::Marker::CUBE, oriented_bounding_box_debug_marker->markers[1].type);
  EXPECT_EQ(visualization_msgs::msg::Marker::ADD, oriented_bounding_box_debug_marker->markers[1].action);
  EXPECT_FALSE(oriented_bounding_box_debug_marker->markers[1].ns.empty());
  EXPECT_EQ(1, oriented_bounding_box_debug_marker->markers[1].frame_locked);
  EXPECT_EQ("laser", oriented_bounding_box_debug_marker->markers[2].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, oriented_bounding_box_debug_marker->markers[2].header.stamp);
  EXPECT_NEAR(2.5, oriented_bounding_box_debug_marker->markers[2].scale.x, 1e-4);
  EXPECT_NEAR(2.5, oriented_bounding_box_debug_marker->markers[2].scale.y, 1e-4);
  EXPECT_NEAR(2.5, oriented_bounding_box_debug_marker->markers[2].scale.z, 1e-4);
  EXPECT_NEAR(1.5 + 0.122 - 0.1, oriented_bounding_box_debug_marker->markers[2].pose.position.x, 1e-6);
  EXPECT_NEAR(0, oriented_bounding_box_debug_marker->markers[2].pose.position.y, 1e-6);
  EXPECT_NEAR(0, oriented_bounding_box_debug_marker->markers[2].pose.position.z, 1e-6);
  EXPECT_NEAR(0, oriented_bounding_box_debug_marker->markers[2].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, oriented_bounding_box_debug_marker->markers[2].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, oriented_bounding_box_debug_marker->markers[2].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, oriented_bounding_box_debug_marker->markers[2].pose.orientation.w, 1e-6);
  EXPECT_LT(0, oriented_bounding_box_debug_marker->markers[2].color.a);
  // make sure the marker has some color
  EXPECT_LT(
    0,
    oriented_bounding_box_debug_marker->markers[2].color.r + oriented_bounding_box_debug_marker->markers[2].color.g +
    oriented_bounding_box_debug_marker->markers[2].color.b);
  EXPECT_EQ(visualization_msgs::msg::Marker::CUBE, oriented_bounding_box_debug_marker->markers[2].type);
  EXPECT_EQ(visualization_msgs::msg::Marker::ADD, oriented_bounding_box_debug_marker->markers[2].action);
  EXPECT_FALSE(oriented_bounding_box_debug_marker->markers[2].ns.empty());
  EXPECT_EQ(1, oriented_bounding_box_debug_marker->markers[2].frame_locked);
  EXPECT_EQ("laser", oriented_bounding_box_debug_marker->markers[3].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, oriented_bounding_box_debug_marker->markers[3].header.stamp);
  EXPECT_NEAR(0.1, oriented_bounding_box_debug_marker->markers[3].scale.x, 1e-5);
  EXPECT_NEAR(0.1, oriented_bounding_box_debug_marker->markers[3].scale.y, 1e-5);
  EXPECT_NEAR(0.1, oriented_bounding_box_debug_marker->markers[3].scale.z, 1e-5);
  EXPECT_NEAR(1.5 - 1.5, oriented_bounding_box_debug_marker->markers[3].pose.position.x, 1e-6);
  EXPECT_NEAR(0, oriented_bounding_box_debug_marker->markers[3].pose.position.y, 1e-6);
  EXPECT_NEAR(0, oriented_bounding_box_debug_marker->markers[3].pose.position.z, 1e-6);
  EXPECT_NEAR(0, oriented_bounding_box_debug_marker->markers[3].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, oriented_bounding_box_debug_marker->markers[3].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, oriented_bounding_box_debug_marker->markers[3].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, oriented_bounding_box_debug_marker->markers[3].pose.orientation.w, 1e-6);
  EXPECT_LT(0, oriented_bounding_box_debug_marker->markers[3].color.a);
  // make sure the marker has some color
  EXPECT_LT(
    0,
    oriented_bounding_box_debug_marker->markers[3].color.r + oriented_bounding_box_debug_marker->markers[3].color.g +
    oriented_bounding_box_debug_marker->markers[3].color.b);
  EXPECT_EQ(visualization_msgs::msg::Marker::CUBE, oriented_bounding_box_debug_marker->markers[3].type);
  EXPECT_EQ(visualization_msgs::msg::Marker::ADD, oriented_bounding_box_debug_marker->markers[3].action);
  EXPECT_FALSE(oriented_bounding_box_debug_marker->markers[3].ns.empty());
  EXPECT_EQ(1, oriented_bounding_box_debug_marker->markers[3].frame_locked);

  EXPECT_EQ(4, local_bounding_box_debug_marker->markers.size());
  EXPECT_EQ("base_link", local_bounding_box_debug_marker->markers[0].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, local_bounding_box_debug_marker->markers[0].header.stamp);
  EXPECT_NEAR(2.75, local_bounding_box_debug_marker->markers[0].scale.x, 1e-5);
  EXPECT_NEAR(2.75, local_bounding_box_debug_marker->markers[0].scale.y, 1e-5);
  EXPECT_NEAR(2.75, local_bounding_box_debug_marker->markers[0].scale.z, 1e-5);
  EXPECT_NEAR(-0.122, local_bounding_box_debug_marker->markers[0].pose.position.x, 1e-6);
  EXPECT_NEAR(0, local_bounding_box_debug_marker->markers[0].pose.position.y, 1e-6);
  EXPECT_NEAR(0, local_bounding_box_debug_marker->markers[0].pose.position.z, 1e-6);
  EXPECT_NEAR(0, local_bounding_box_debug_marker->markers[0].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, local_bounding_box_debug_marker->markers[0].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, local_bounding_box_debug_marker->markers[0].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, local_bounding_box_debug_marker->markers[0].pose.orientation.w, 1e-6);
  EXPECT_LT(0, local_bounding_box_debug_marker->markers[0].color.a);
  // make sure the marker has some color
  EXPECT_LT(
    0,
    local_bounding_box_debug_marker->markers[0].color.r + local_bounding_box_debug_marker->markers[0].color.g +
    local_bounding_box_debug_marker->markers[0].color.b);
  EXPECT_EQ(visualization_msgs::msg::Marker::CUBE, local_bounding_box_debug_marker->markers[0].type);
  EXPECT_EQ(visualization_msgs::msg::Marker::ADD, local_bounding_box_debug_marker->markers[0].action);
  EXPECT_FALSE(local_bounding_box_debug_marker->markers[0].ns.empty());
  EXPECT_EQ(1, local_bounding_box_debug_marker->markers[0].frame_locked);
  EXPECT_EQ("base_link", local_bounding_box_debug_marker->markers[1].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, local_bounding_box_debug_marker->markers[1].header.stamp);
  EXPECT_NEAR(1.84, local_bounding_box_debug_marker->markers[1].scale.x, 1e-5);
  EXPECT_NEAR(1.84, local_bounding_box_debug_marker->markers[1].scale.y, 1e-5);
  EXPECT_NEAR(1.84, local_bounding_box_debug_marker->markers[1].scale.z, 1e-5);
  EXPECT_NEAR(-0.122, local_bounding_box_debug_marker->markers[1].pose.position.x, 1e-6);
  EXPECT_NEAR(0, local_bounding_box_debug_marker->markers[1].pose.position.y, 1e-6);
  EXPECT_NEAR(0, local_bounding_box_debug_marker->markers[1].pose.position.z, 1e-6);
  EXPECT_NEAR(0, local_bounding_box_debug_marker->markers[1].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, local_bounding_box_debug_marker->markers[1].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, local_bounding_box_debug_marker->markers[1].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, local_bounding_box_debug_marker->markers[1].pose.orientation.w, 1e-6);
  EXPECT_LT(0, local_bounding_box_debug_marker->markers[1].color.a);
  // make sure the marker has some color
  EXPECT_LT(
    0,
    local_bounding_box_debug_marker->markers[1].color.r + local_bounding_box_debug_marker->markers[1].color.g +
    local_bounding_box_debug_marker->markers[1].color.b);
  EXPECT_EQ(visualization_msgs::msg::Marker::CUBE, local_bounding_box_debug_marker->markers[1].type);
  EXPECT_EQ(visualization_msgs::msg::Marker::ADD, local_bounding_box_debug_marker->markers[1].action);
  EXPECT_FALSE(local_bounding_box_debug_marker->markers[1].ns.empty());
  EXPECT_EQ(1, local_bounding_box_debug_marker->markers[1].frame_locked);
  EXPECT_EQ("base_link", local_bounding_box_debug_marker->markers[2].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, local_bounding_box_debug_marker->markers[2].header.stamp);
  EXPECT_NEAR(2.5, local_bounding_box_debug_marker->markers[2].scale.x, 1e-4);
  EXPECT_NEAR(2.5, local_bounding_box_debug_marker->markers[2].scale.y, 1e-4);
  EXPECT_NEAR(2.5, local_bounding_box_debug_marker->markers[2].scale.z, 1e-4);
  EXPECT_NEAR(-0.1, local_bounding_box_debug_marker->markers[2].pose.position.x, 1e-6);
  EXPECT_NEAR(0, local_bounding_box_debug_marker->markers[2].pose.position.y, 1e-6);
  EXPECT_NEAR(0, local_bounding_box_debug_marker->markers[2].pose.position.z, 1e-6);
  EXPECT_NEAR(0, local_bounding_box_debug_marker->markers[2].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, local_bounding_box_debug_marker->markers[2].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, local_bounding_box_debug_marker->markers[2].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, local_bounding_box_debug_marker->markers[2].pose.orientation.w, 1e-6);
  EXPECT_LT(0, local_bounding_box_debug_marker->markers[2].color.a);
  // make sure the marker has some color
  EXPECT_LT(
    0,
    local_bounding_box_debug_marker->markers[2].color.r + local_bounding_box_debug_marker->markers[2].color.g +
    local_bounding_box_debug_marker->markers[2].color.b);
  EXPECT_EQ(visualization_msgs::msg::Marker::CUBE, local_bounding_box_debug_marker->markers[2].type);
  EXPECT_EQ(visualization_msgs::msg::Marker::ADD, local_bounding_box_debug_marker->markers[2].action);
  EXPECT_FALSE(local_bounding_box_debug_marker->markers[2].ns.empty());
  EXPECT_EQ(1, local_bounding_box_debug_marker->markers[2].frame_locked);
  EXPECT_EQ("base_link", local_bounding_box_debug_marker->markers[3].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, local_bounding_box_debug_marker->markers[3].header.stamp);
  EXPECT_NEAR(0.1, local_bounding_box_debug_marker->markers[3].scale.x, 1e-5);
  EXPECT_NEAR(0.1, local_bounding_box_debug_marker->markers[3].scale.y, 1e-5);
  EXPECT_NEAR(0.1, local_bounding_box_debug_marker->markers[3].scale.z, 1e-5);
  EXPECT_NEAR(-1.5 - 0.122, local_bounding_box_debug_marker->markers[3].pose.position.x, 1e-6);
  EXPECT_NEAR(0, local_bounding_box_debug_marker->markers[3].pose.position.y, 1e-6);
  EXPECT_NEAR(0, local_bounding_box_debug_marker->markers[3].pose.position.z, 1e-6);
  EXPECT_NEAR(0, local_bounding_box_debug_marker->markers[3].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, local_bounding_box_debug_marker->markers[3].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, local_bounding_box_debug_marker->markers[3].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, local_bounding_box_debug_marker->markers[3].pose.orientation.w, 1e-6);
  EXPECT_LT(0, local_bounding_box_debug_marker->markers[3].color.a);
  // make sure the marker has some color
  EXPECT_LT(
    0,
    local_bounding_box_debug_marker->markers[3].color.r + local_bounding_box_debug_marker->markers[3].color.g +
    local_bounding_box_debug_marker->markers[3].color.b);
  EXPECT_EQ(visualization_msgs::msg::Marker::CUBE, local_bounding_box_debug_marker->markers[3].type);
  EXPECT_EQ(visualization_msgs::msg::Marker::ADD, local_bounding_box_debug_marker->markers[3].action);
  EXPECT_FALSE(local_bounding_box_debug_marker->markers[3].ns.empty());
  EXPECT_EQ(1, local_bounding_box_debug_marker->markers[3].frame_locked);

  EXPECT_EQ(4, robot_model_contains_test->markers.size());
  EXPECT_EQ("laser", robot_model_contains_test->markers[0].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, robot_model_contains_test->markers[0].header.stamp);
  EXPECT_NEAR(2.75, robot_model_contains_test->markers[0].scale.x, 1e-5);
  EXPECT_NEAR(2.75, robot_model_contains_test->markers[0].scale.y, 1e-5);
  EXPECT_NEAR(2.75, robot_model_contains_test->markers[0].scale.z, 1e-5);
  EXPECT_NEAR(1.5, robot_model_contains_test->markers[0].pose.position.x, 1e-6);
  EXPECT_NEAR(0, robot_model_contains_test->markers[0].pose.position.y, 1e-6);
  EXPECT_NEAR(0, robot_model_contains_test->markers[0].pose.position.z, 1e-6);
  EXPECT_NEAR(0, robot_model_contains_test->markers[0].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, robot_model_contains_test->markers[0].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, robot_model_contains_test->markers[0].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, robot_model_contains_test->markers[0].pose.orientation.w, 1e-6);
  EXPECT_LT(0, robot_model_contains_test->markers[0].color.a);
  // make sure the marker has some color
  EXPECT_LT(
    0,
    robot_model_contains_test->markers[0].color.r + robot_model_contains_test->markers[0].color.g +
    robot_model_contains_test->markers[0].color.b);
  EXPECT_EQ(visualization_msgs::msg::Marker::SPHERE, robot_model_contains_test->markers[0].type);
  EXPECT_EQ(visualization_msgs::msg::Marker::ADD, robot_model_contains_test->markers[0].action);
  EXPECT_EQ("antenna-0", robot_model_contains_test->markers[0].ns);
  EXPECT_EQ(1, robot_model_contains_test->markers[0].frame_locked);
  EXPECT_EQ("laser", robot_model_contains_test->markers[1].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, robot_model_contains_test->markers[1].header.stamp);
  EXPECT_NEAR(1.84, robot_model_contains_test->markers[1].scale.x, 1e-5);
  EXPECT_NEAR(1.84, robot_model_contains_test->markers[1].scale.y, 1e-5);
  EXPECT_NEAR(1.84, robot_model_contains_test->markers[1].scale.z, 1e-5);
  EXPECT_NEAR(1.5, robot_model_contains_test->markers[1].pose.position.x, 1e-6);
  EXPECT_NEAR(0, robot_model_contains_test->markers[1].pose.position.y, 1e-6);
  EXPECT_NEAR(0, robot_model_contains_test->markers[1].pose.position.z, 1e-6);
  EXPECT_NEAR(0, robot_model_contains_test->markers[1].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, robot_model_contains_test->markers[1].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, robot_model_contains_test->markers[1].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, robot_model_contains_test->markers[1].pose.orientation.w, 1e-6);
  EXPECT_LT(0, robot_model_contains_test->markers[1].color.a);
  // make sure the marker has some color
  EXPECT_LT(
    0,
    robot_model_contains_test->markers[1].color.r + robot_model_contains_test->markers[1].color.g +
    robot_model_contains_test->markers[1].color.b);
  EXPECT_EQ(visualization_msgs::msg::Marker::CUBE, robot_model_contains_test->markers[1].type);
  EXPECT_EQ(visualization_msgs::msg::Marker::ADD, robot_model_contains_test->markers[1].action);
  EXPECT_EQ("base_link-0", robot_model_contains_test->markers[1].ns);
  EXPECT_EQ(1, robot_model_contains_test->markers[1].frame_locked);
  EXPECT_EQ("laser", robot_model_contains_test->markers[2].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, robot_model_contains_test->markers[2].header.stamp);
  EXPECT_NEAR(2.5, robot_model_contains_test->markers[2].scale.x, 1e-4);
  EXPECT_NEAR(2.5, robot_model_contains_test->markers[2].scale.y, 1e-4);
  EXPECT_NEAR(2.5, robot_model_contains_test->markers[2].scale.z, 1e-4);
  EXPECT_NEAR(1.5 + 0.122 - 0.1, robot_model_contains_test->markers[2].pose.position.x, 1e-6);
  EXPECT_NEAR(0, robot_model_contains_test->markers[2].pose.position.y, 1e-6);
  EXPECT_NEAR(0, robot_model_contains_test->markers[2].pose.position.z, 1e-6);
  EXPECT_NEAR(0, robot_model_contains_test->markers[2].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, robot_model_contains_test->markers[2].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, robot_model_contains_test->markers[2].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, robot_model_contains_test->markers[2].pose.orientation.w, 1e-6);
  EXPECT_LT(0, robot_model_contains_test->markers[2].color.a);
  // make sure the marker has some color
  EXPECT_LT(
    0,
    robot_model_contains_test->markers[2].color.r + robot_model_contains_test->markers[2].color.g +
    robot_model_contains_test->markers[2].color.b);
  EXPECT_EQ(visualization_msgs::msg::Marker::CUBE, robot_model_contains_test->markers[2].type);
  EXPECT_EQ(visualization_msgs::msg::Marker::ADD, robot_model_contains_test->markers[2].action);
  EXPECT_EQ("base_link-1", robot_model_contains_test->markers[2].ns);
  EXPECT_EQ(1, robot_model_contains_test->markers[2].frame_locked);
  EXPECT_EQ("laser", robot_model_contains_test->markers[3].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, robot_model_contains_test->markers[3].header.stamp);
  EXPECT_NEAR(0.1, robot_model_contains_test->markers[3].scale.x, 1e-5);
  EXPECT_NEAR(0.1, robot_model_contains_test->markers[3].scale.y, 1e-5);
  EXPECT_NEAR(0.1, robot_model_contains_test->markers[3].scale.z, 1e-5);
  EXPECT_NEAR(1.5 - 1.5, robot_model_contains_test->markers[3].pose.position.x, 1e-6);
  EXPECT_NEAR(0, robot_model_contains_test->markers[3].pose.position.y, 1e-6);
  EXPECT_NEAR(0, robot_model_contains_test->markers[3].pose.position.z, 1e-6);
  EXPECT_NEAR(0, robot_model_contains_test->markers[3].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, robot_model_contains_test->markers[3].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, robot_model_contains_test->markers[3].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, robot_model_contains_test->markers[3].pose.orientation.w, 1e-6);
  EXPECT_LT(0, robot_model_contains_test->markers[3].color.a);
  // make sure the marker has some color
  EXPECT_LT(
    0,
    robot_model_contains_test->markers[3].color.r + robot_model_contains_test->markers[3].color.g +
    robot_model_contains_test->markers[3].color.b);
  EXPECT_EQ(visualization_msgs::msg::Marker::CUBE, robot_model_contains_test->markers[3].type);
  EXPECT_EQ(visualization_msgs::msg::Marker::ADD, robot_model_contains_test->markers[3].action);
  EXPECT_EQ("laser-0", robot_model_contains_test->markers[3].ns);
  EXPECT_EQ(1, robot_model_contains_test->markers[3].frame_locked);

  EXPECT_EQ(2, robot_model_shadow_test->markers.size());
  EXPECT_EQ("laser", robot_model_shadow_test->markers[0].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, robot_model_shadow_test->markers[0].header.stamp);
  EXPECT_NEAR(2.75, robot_model_shadow_test->markers[0].scale.x, 1e-5);
  EXPECT_NEAR(2.75, robot_model_shadow_test->markers[0].scale.y, 1e-5);
  EXPECT_NEAR(2.75, robot_model_shadow_test->markers[0].scale.z, 1e-5);
  EXPECT_NEAR(1.5, robot_model_shadow_test->markers[0].pose.position.x, 1e-6);
  EXPECT_NEAR(0, robot_model_shadow_test->markers[0].pose.position.y, 1e-6);
  EXPECT_NEAR(0, robot_model_shadow_test->markers[0].pose.position.z, 1e-6);
  EXPECT_NEAR(0, robot_model_shadow_test->markers[0].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, robot_model_shadow_test->markers[0].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, robot_model_shadow_test->markers[0].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, robot_model_shadow_test->markers[0].pose.orientation.w, 1e-6);
  EXPECT_LT(0, robot_model_shadow_test->markers[0].color.a);
  // make sure the marker has some color
  EXPECT_LT(
    0,
    robot_model_shadow_test->markers[0].color.r + robot_model_shadow_test->markers[0].color.g +
    robot_model_shadow_test->markers[0].color.b);
  EXPECT_EQ(visualization_msgs::msg::Marker::SPHERE, robot_model_shadow_test->markers[0].type);
  EXPECT_EQ(visualization_msgs::msg::Marker::ADD, robot_model_shadow_test->markers[0].action);
  EXPECT_EQ("antenna-0", robot_model_shadow_test->markers[0].ns);
  EXPECT_EQ(1, robot_model_shadow_test->markers[0].frame_locked);
  EXPECT_EQ("laser", robot_model_shadow_test->markers[1].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, robot_model_shadow_test->markers[1].header.stamp);
  EXPECT_NEAR(2, robot_model_shadow_test->markers[1].scale.x, 1e-5);
  EXPECT_NEAR(2, robot_model_shadow_test->markers[1].scale.y, 1e-5);
  EXPECT_NEAR(2, robot_model_shadow_test->markers[1].scale.z, 1e-5);
  EXPECT_NEAR(1.5, robot_model_shadow_test->markers[1].pose.position.x, 1e-6);
  EXPECT_NEAR(0, robot_model_shadow_test->markers[1].pose.position.y, 1e-6);
  EXPECT_NEAR(0, robot_model_shadow_test->markers[1].pose.position.z, 1e-6);
  EXPECT_NEAR(0, robot_model_shadow_test->markers[1].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, robot_model_shadow_test->markers[1].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, robot_model_shadow_test->markers[1].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, robot_model_shadow_test->markers[1].pose.orientation.w, 1e-6);
  EXPECT_LT(0, robot_model_shadow_test->markers[1].color.a);
  // make sure the marker has some color
  EXPECT_LT(
    0,
    robot_model_shadow_test->markers[1].color.r + robot_model_shadow_test->markers[1].color.g +
    robot_model_shadow_test->markers[1].color.b);
  EXPECT_EQ(visualization_msgs::msg::Marker::CUBE, robot_model_shadow_test->markers[1].type);
  EXPECT_EQ(visualization_msgs::msg::Marker::ADD, robot_model_shadow_test->markers[1].action);
  EXPECT_EQ("base_link-0", robot_model_shadow_test->markers[1].ns);
  EXPECT_EQ(1, robot_model_shadow_test->markers[1].frame_locked);
}  // NOLINT

TEST(RobotBodyFilter, UpdateLaserScan) {
  const auto nh = std::make_shared<rclcpp::Node>("compute_mask_config_point_by_point");

  const auto filter = std::make_shared<RobotBodyFilterLaserScanTest>(nh);
  const auto filter_base = std::dynamic_pointer_cast<filters::FilterBase<sensor_msgs::msg::LaserScan>>(filter);

  filter_base->configure(
    "filter1.params", "compute_mask_config_point_by_point",
    nh->get_node_logging_interface(), nh->get_node_parameters_interface());

  const std_msgs::msg::String::SharedPtr msg(new std_msgs::msg::String);
  msg->data = ROBOT_URDF;
  filter->onRobotModelMsg(msg);

  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "laser";
  scan.range_min = 0.1;
  scan.range_max = 10;
  scan.angle_min = -M_PI_2;
  scan.angle_max = M_PI_2;
  scan.angle_increment = M_PI / 18;
  scan.time_increment = 1.0f;
  scan.scan_time = 20.0f;
  for (size_t i = 0; i < 19; ++i) {
    scan.intensities.push_back(i);
    scan.ranges.push_back(1.5);
  }

  {
    geometry_msgs::msg::TransformStamped tf;
    rclcpp::Time now = nh->get_clock()->now();
    scan.header.stamp = now;
    tf.transform.rotation.w = 1;
    for (double d = -5.0; d < 5.0; d += 0.1) {
      tf.header.stamp = now + rclcpp::Duration::from_seconds(d);

      tf.transform.translation.x = 0.122;
      tf.header.frame_id = "odom";
      tf.child_frame_id = "base_link";
      ASSERT_TRUE(filter->tf_buffer_->setTransform(tf, "test"));

      tf.transform.translation.x = -1.5 - 0.122;
      tf.header.frame_id = "base_link";
      tf.child_frame_id = "laser";
      ASSERT_TRUE(filter->tf_buffer_->setTransform(tf, "test"));

      tf.transform.translation.x = 0.01864 - 0.122;
      tf.header.frame_id = "base_link";
      tf.child_frame_id = "antenna";
      ASSERT_TRUE(filter->tf_buffer_->setTransform(tf, "test"));

      tf.header.stamp = rclcpp::Time(tf.header.stamp) + rclcpp::Duration::from_seconds(20);

      tf.transform.translation.x = 10.122;
      tf.header.frame_id = "odom";
      tf.child_frame_id = "base_link";
      ASSERT_TRUE(filter->tf_buffer_->setTransform(tf, "test"));

      tf.transform.translation.x = -1.5 - 0.122;
      tf.header.frame_id = "base_link";
      tf.child_frame_id = "laser";
      ASSERT_TRUE(filter->tf_buffer_->setTransform(tf, "test"));

      tf.transform.translation.x = 0.01864 - 0.122;
      tf.header.frame_id = "base_link";
      tf.child_frame_id = "antenna";
      ASSERT_TRUE(filter->tf_buffer_->setTransform(tf, "test"));
    }
  }

  // give TF frames watchdog some time to catch up
  while (!filter->tf_frames_watchdog_->isReachable("laser")) {
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }
  while (!filter->tf_frames_watchdog_->isReachable("base_link")) {
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }

  sensor_msgs::msg::LaserScan out_scan;
  ASSERT_TRUE(filter->update(scan, out_scan));

  ASSERT_EQ(19, out_scan.ranges.size());
  EXPECT_EQ(1.5, out_scan.ranges[0]); EXPECT_EQ(0, out_scan.intensities[0]);
  EXPECT_EQ(1.5, out_scan.ranges[1]); EXPECT_EQ(1, out_scan.intensities[1]);
  EXPECT_EQ(1.5, out_scan.ranges[2]); EXPECT_EQ(2, out_scan.intensities[2]);
  EXPECT_NAN(out_scan.ranges[3]); EXPECT_EQ(3, out_scan.intensities[3]);
  EXPECT_NAN(out_scan.ranges[4]); EXPECT_EQ(4, out_scan.intensities[4]);
  EXPECT_NAN(out_scan.ranges[5]); EXPECT_EQ(5, out_scan.intensities[5]);
  EXPECT_NAN(out_scan.ranges[6]); EXPECT_EQ(6, out_scan.intensities[6]);
  EXPECT_NAN(out_scan.ranges[7]); EXPECT_EQ(7, out_scan.intensities[7]);
  EXPECT_NAN(out_scan.ranges[8]); EXPECT_EQ(8, out_scan.intensities[8]);
  EXPECT_NAN(out_scan.ranges[9]); EXPECT_EQ(9, out_scan.intensities[9]);
  EXPECT_NAN(out_scan.ranges[10]); EXPECT_EQ(10, out_scan.intensities[10]);
  EXPECT_NAN(out_scan.ranges[11]); EXPECT_EQ(11, out_scan.intensities[11]);
  EXPECT_NAN(out_scan.ranges[12]); EXPECT_EQ(12, out_scan.intensities[12]);
  EXPECT_NAN(out_scan.ranges[13]); EXPECT_EQ(13, out_scan.intensities[13]);
  EXPECT_NAN(out_scan.ranges[14]); EXPECT_EQ(14, out_scan.intensities[14]);
  EXPECT_NAN(out_scan.ranges[15]); EXPECT_EQ(15, out_scan.intensities[15]);
  EXPECT_EQ(1.5, out_scan.ranges[16]); EXPECT_EQ(16, out_scan.intensities[16]);
  EXPECT_EQ(1.5, out_scan.ranges[17]); EXPECT_EQ(17, out_scan.intensities[17]);
  EXPECT_EQ(1.5, out_scan.ranges[18]); EXPECT_EQ(18, out_scan.intensities[18]);
}

TEST(RobotBodyFilter, UpdatePointCloud2) {
  const auto nh = std::make_shared<rclcpp::Node>("compute_mask_config_all_at_once");

  const auto filter = std::make_shared<RobotBodyFilterPointCloud2Test>(nh);
  const auto filter_base = std::dynamic_pointer_cast<filters::FilterBase<sensor_msgs::msg::PointCloud2>>(filter);

  filter_base->configure(
    "filter1.params", "compute_mask_config_all_at_once",
    nh->get_node_logging_interface(), nh->get_node_parameters_interface());

  const std_msgs::msg::String::SharedPtr msg(new std_msgs::msg::String);
  msg->data = ROBOT_URDF;
  filter->onRobotModelMsg(msg);

  cras::Cloud cloud;
  cloud.header.frame_id = "laser";
  cras::CloudModifier mod(cloud);
  mod.setPointCloud2Fields(
    3,
    "x", 1, sensor_msgs::msg::PointField::FLOAT32,
    "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32);
  mod.resize(12);
  cloud.width = 4;
  cloud.height = 3;
  cloud.row_step = cloud.width * cloud.point_step;

  {
    cras::CloudIter x_it(cloud, "x");
    cras::CloudIter y_it(cloud, "y");
    cras::CloudIter z_it(cloud, "z");

    *x_it = 1.5 + -1.5; *y_it = 0; *z_it = 0; ++x_it, ++y_it, ++z_it;  // pointSensor
    *x_it = 1.5 + -1.47; *y_it = 0; *z_it = 0; ++x_it, ++y_it, ++z_it;  // pointSensor2
    *x_it = 1.5 + -1.42; *y_it = 0; *z_it = 0; ++x_it, ++y_it, ++z_it;  // pointClipMin
    *x_it = 1.5 + 10; *y_it = 0; *z_it = 0; ++x_it, ++y_it, ++z_it;  // pointClipMax
    *x_it = 1.5 + 0.95; *y_it = 0.95; *z_it = 0.95; ++x_it, ++y_it, ++z_it;  // pointInBox
    *x_it = 1.5 + 1.35; *y_it = 0; *z_it = 0; ++x_it, ++y_it, ++z_it;  // pointInSphere
    *x_it = 1.5 + 0; *y_it = 0; *z_it = 0; ++x_it, ++y_it, ++z_it;  // pointInBoth
    *x_it = 1.5 + -0.25; *y_it = -2; *z_it = 2; ++x_it, ++y_it, ++z_it;  // pointShadowBox
    *x_it = 1.5 + -0.560762; *y_it = 0; *z_it = 1.83871; ++x_it, ++y_it, ++z_it;  // pointShadowSphere
    *x_it = 1.5 + 1.5; *y_it = 0; *z_it = 0; ++x_it, ++y_it, ++z_it;  // pointShadowBoth
    *x_it = 1.5 + -3; *y_it = 0; *z_it = 0; ++x_it, ++y_it, ++z_it;  // pointOutside
    *x_it = 1.5 + -4; *y_it = 0; *z_it = 0; ++x_it, ++y_it, ++z_it;  // pointOutside2
  }

  {
    rclcpp::Time now = nh->get_clock()->now();
    cloud.header.stamp = now;
    geometry_msgs::msg::TransformStamped tf;
    tf.transform.rotation.w = 1.0;
    for (double d = -5.0; d < 5.0; d += 0.1) {
      tf.header.stamp = now + rclcpp::Duration::from_seconds(d);

      tf.transform.translation.x = 0.122;
      tf.header.frame_id = "odom";
      tf.child_frame_id = "base_link";
      filter->tf_buffer_->setTransform(tf, "test");

      tf.transform.translation.x = -1.5 - 0.122;
      tf.header.frame_id = "base_link";
      tf.child_frame_id = "laser";
      filter->tf_buffer_->setTransform(tf, "test");

      tf.transform.translation.x = 0.01864 - 0.122;
      tf.header.frame_id = "base_link";
      tf.child_frame_id = "antenna";
      filter->tf_buffer_->setTransform(tf, "test");
    }
  }

  while (!filter->tf_frames_watchdog_->isReachable("laser")) {
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }
  while (!filter->tf_frames_watchdog_->isReachable("base_link")) {
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }

  sensor_msgs::msg::PointCloud2 out_cloud;
  filter->update(cloud, out_cloud);

  ASSERT_EQ(cras::numPoints(cloud), cras::numPoints(out_cloud));
  EXPECT_EQ("base_link", out_cloud.header.frame_id);
  EXPECT_EQ(cloud.header.stamp, out_cloud.header.stamp);
  EXPECT_EQ(cloud.height, out_cloud.height);
  EXPECT_EQ(cloud.width, out_cloud.width);
  EXPECT_EQ(cloud.fields, out_cloud.fields);
  EXPECT_EQ(cloud.point_step, out_cloud.point_step);
  EXPECT_EQ(cloud.row_step, out_cloud.row_step);

  cras::CloudConstIter x_it(out_cloud, "x");
  cras::CloudConstIter y_it(out_cloud, "y");
  cras::CloudConstIter z_it(out_cloud, "z");

  // PointSensor
  EXPECT_NAN(*x_it); EXPECT_NAN(*y_it); EXPECT_NAN(*z_it);
  ++x_it, ++y_it, ++z_it;
  // PointSensor2
  EXPECT_NAN(*x_it); EXPECT_NAN(*y_it); EXPECT_NAN(*z_it);
  ++x_it, ++y_it, ++z_it;
  // PointClipMin
  EXPECT_NAN(*x_it); EXPECT_NAN(*y_it); EXPECT_NAN(*z_it);
  ++x_it, ++y_it, ++z_it;
  // PointClipMax
  EXPECT_NAN(*x_it); EXPECT_NAN(*y_it); EXPECT_NAN(*z_it);
  ++x_it, ++y_it, ++z_it;
  // PointInBox
  EXPECT_NAN(*x_it); EXPECT_NAN(*y_it); EXPECT_NAN(*z_it);
  ++x_it, ++y_it, ++z_it;
  // PointInSphere
  EXPECT_NAN(*x_it); EXPECT_NAN(*y_it); EXPECT_NAN(*z_it);
  ++x_it, ++y_it, ++z_it;
  // PointInBoth
  EXPECT_NAN(*x_it); EXPECT_NAN(*y_it); EXPECT_NAN(*z_it);
  ++x_it, ++y_it, ++z_it;
  // PointShadowBox
  EXPECT_NAN(*x_it); EXPECT_NAN(*y_it); EXPECT_NAN(*z_it);
  ++x_it, ++y_it, ++z_it;
  // PointShadowSphere
  EXPECT_NAN(*x_it); EXPECT_NAN(*y_it); EXPECT_NAN(*z_it);
  ++x_it, ++y_it, ++z_it;
  // PointShadowBoth
  EXPECT_NAN(*x_it); EXPECT_NAN(*y_it); EXPECT_NAN(*z_it);
  ++x_it, ++y_it, ++z_it;
  // PointOutside
  EXPECT_NEAR(-3.122, *x_it, 1e-5); EXPECT_NEAR(0, *y_it, 1e-6); EXPECT_NEAR(0, *z_it, 1e-6);
  ++x_it, ++y_it, ++z_it;
  // PointOutside2
  EXPECT_NEAR(-4.122, *x_it, 1e-5); EXPECT_NEAR(0, *y_it, 1e-6); EXPECT_NEAR(0, *z_it, 1e-6);
  ++x_it, ++y_it, ++z_it;

  // test with unorganized clouds

  filter->keep_clouds_organized_ = false;

  filter->update(cloud, out_cloud);

  ASSERT_EQ(2, cras::numPoints(out_cloud));
  EXPECT_EQ("base_link", out_cloud.header.frame_id);
  EXPECT_EQ(cloud.header.stamp, out_cloud.header.stamp);
  EXPECT_EQ(1, out_cloud.height);
  EXPECT_EQ(2, out_cloud.width);
  EXPECT_EQ(cloud.fields, out_cloud.fields);
  EXPECT_EQ(cloud.point_step, out_cloud.point_step);

  x_it = cras::CloudConstIter(out_cloud, "x");
  y_it = cras::CloudConstIter(out_cloud, "y");
  z_it = cras::CloudConstIter(out_cloud, "z");

  // PointOutside
  EXPECT_NEAR(-3.122, *x_it, 1e-5); EXPECT_NEAR(0, *y_it, 1e-6); EXPECT_NEAR(0, *z_it, 1e-6);
  ++x_it, ++y_it, ++z_it;
  // PointOutside2
  EXPECT_NEAR(-4.122, *x_it, 1e-5); EXPECT_NEAR(0, *y_it, 1e-6); EXPECT_NEAR(0, *z_it, 1e-6);
  ++x_it, ++y_it, ++z_it;
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
