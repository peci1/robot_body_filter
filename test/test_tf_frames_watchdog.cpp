// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include "gtest/gtest.h"

#include <memory>
#include <set>
#include <string>

#include <rclcpp/duration.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/utilities.hpp>
#include <robot_body_filter/TfFramesWatchdog.hpp>
#include <tf2_ros/buffer.hpp>

class TestWatchdog : public robot_body_filter::TFFramesWatchdog {
public:
  TestWatchdog(
    rclcpp::Node::SharedPtr node, const std::string& robotFrame, const std::set<std::string>& monitoredFrames,
    const std::shared_ptr<tf2_ros::Buffer>& tfBuffer, const rclcpp::Duration& unreachableTfLookupTimeout,
    const rclcpp::Rate::SharedPtr& unreachableFramesCheckRate)
    : TFFramesWatchdog(node->get_logger(), node->get_clock(), robotFrame, monitoredFrames, tfBuffer,
      unreachableTfLookupTimeout, unreachableFramesCheckRate) {
  }

  friend class TfFramesWatchdog_Basic_Test;
  friend class TfFramesWatchdog_ThreadControl_Test;
  friend class TfFramesWatchdog_SearchForReachableFrames_Test;
  friend class TfFramesWatchdog_LookupTransform_Test;
};

TEST(TfFramesWatchdog, Basic) {
  const auto nh = std::make_shared<rclcpp::Node>("test_ray_casting_shape_mask");
  const auto tfBuffer = std::make_shared<tf2_ros::Buffer>(nh->get_clock(), tf2::BUFFER_CORE_DEFAULT_CACHE_TIME, nh);
  TestWatchdog watchdog(
    nh, "base_link", {"left_track", "front_left_flipper"}, tfBuffer,
    rclcpp::Duration::from_seconds(0.1), std::make_shared<rclcpp::Rate>(1.0));

  EXPECT_TRUE(watchdog.isMonitored("left_track"));
  EXPECT_TRUE(watchdog.isMonitored("front_left_flipper"));
  EXPECT_FALSE(watchdog.isMonitored("base_link"));

  EXPECT_FALSE(watchdog.isMonitored("rear_left_flipper"));
  watchdog.addMonitoredFrame("rear_left_flipper");
  EXPECT_TRUE(watchdog.isMonitored("rear_left_flipper"));

  watchdog.setMonitoredFrames({"test"});
  EXPECT_FALSE(watchdog.isMonitored("left_track"));
  EXPECT_FALSE(watchdog.isMonitored("front_left_flipper"));
  EXPECT_FALSE(watchdog.isMonitored("base_link"));
  EXPECT_FALSE(watchdog.isMonitored("rear_left_flipper"));
  EXPECT_TRUE(watchdog.isMonitored("test"));
  EXPECT_FALSE(watchdog.isReachable("left_track"));
  EXPECT_FALSE(watchdog.isReachable("front_left_flipper"));
  EXPECT_FALSE(watchdog.isReachable("rear_left_flipper"));
  EXPECT_FALSE(watchdog.isReachable("base_link"));

  watchdog.markReachable("left_track");
  EXPECT_TRUE(watchdog.isReachable("left_track"));
  watchdog.markUnreachable("left_track");
  EXPECT_FALSE(watchdog.isReachable("left_track"));

  watchdog.clear();
  EXPECT_FALSE(watchdog.isMonitored("left_track"));
  EXPECT_FALSE(watchdog.isMonitored("front_left_flipper"));
  EXPECT_FALSE(watchdog.isMonitored("base_link"));
  EXPECT_FALSE(watchdog.isMonitored("rear_left_flipper"));
  EXPECT_FALSE(watchdog.isMonitored("test"));
  EXPECT_FALSE(watchdog.isReachable("left_track"));
  EXPECT_FALSE(watchdog.isReachable("front_left_flipper"));
  EXPECT_FALSE(watchdog.isReachable("rear_left_flipper"));
  EXPECT_FALSE(watchdog.isReachable("base_link"));
  EXPECT_FALSE(watchdog.isReachable("test"));
}

TEST(TfFramesWatchdog, ThreadControl) {
  const auto nh = std::make_shared<rclcpp::Node>("test_ray_casting_shape_mask");
  const auto tfBuffer = std::make_shared<tf2_ros::Buffer>(nh->get_clock(), tf2::BUFFER_CORE_DEFAULT_CACHE_TIME, nh);
  TestWatchdog watchdog(
    nh, "base_link", {"left_track", "front_left_flipper"}, tfBuffer,
    rclcpp::Duration::from_seconds(0.1), std::make_shared<rclcpp::Rate>(1.0));

  EXPECT_FALSE(watchdog.started_);
  EXPECT_TRUE(watchdog.paused_);
  EXPECT_FALSE(watchdog.should_stop_);

  watchdog.unpause();
  EXPECT_FALSE(watchdog.started_);
  EXPECT_FALSE(watchdog.paused_);
  EXPECT_FALSE(watchdog.should_stop_);

  watchdog.pause();
  EXPECT_FALSE(watchdog.started_);
  EXPECT_TRUE(watchdog.paused_);
  EXPECT_FALSE(watchdog.should_stop_);

  watchdog.stop();
  EXPECT_FALSE(watchdog.started_);
  EXPECT_TRUE(watchdog.paused_);
  EXPECT_TRUE(watchdog.should_stop_);

  watchdog.start();
  for (size_t i = 0; i < 100; ++i) {
    if (watchdog.started_) {
      break;
    }
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_TRUE(watchdog.started_);
  EXPECT_FALSE(watchdog.paused_);
  EXPECT_FALSE(watchdog.should_stop_);

  watchdog.stop();
  EXPECT_TRUE(watchdog.started_);
  EXPECT_TRUE(watchdog.paused_);
  EXPECT_TRUE(watchdog.should_stop_);

  // test that the watchdog can be re-run
  watchdog.start();
  for (size_t i = 0; i < 100; ++i) {
    if (watchdog.started_) {
      break;
    }
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_TRUE(watchdog.started_);
  EXPECT_FALSE(watchdog.paused_);
  EXPECT_FALSE(watchdog.should_stop_);

  watchdog.stop();
  EXPECT_TRUE(watchdog.started_);
  EXPECT_TRUE(watchdog.paused_);
  EXPECT_TRUE(watchdog.should_stop_);
}

TEST(TfFramesWatchdog, SearchForReachableFrames) {
  const auto nh = std::make_shared<rclcpp::Node>("test_ray_casting_shape_mask");
  const auto clock_ptr = nh->get_clock();
  const auto tfBuffer = std::make_shared<tf2_ros::Buffer>(nh->get_clock(), tf2::BUFFER_CORE_DEFAULT_CACHE_TIME, nh);
  tfBuffer->setUsingDedicatedThread(true);
  TestWatchdog watchdog(
    nh, "base_link", {"left_track", "front_left_flipper"}, tfBuffer,
    rclcpp::Duration::from_seconds(0.1), std::make_shared<rclcpp::Rate>(1.0));

  watchdog.unpause();  // searchForReachableFrames checks paused_

  rclcpp::Time start = clock_ptr->now();
  watchdog.searchForReachableFrames();
  rclcpp::Time end = clock_ptr->now();

  EXPECT_FALSE(watchdog.isReachable("left_track"));
  EXPECT_FALSE(watchdog.isReachable("front_left_flipper"));
  // we're searching for 2 frames; check that the search took at least 2x lookup timeout time, but
  // it did not take much more
  EXPECT_LE(2.0 * watchdog.unreachable_tf_lookup_timeout_.seconds(), (end - start).seconds());
  EXPECT_GE(2.5 * watchdog.unreachable_tf_lookup_timeout_.seconds(), (end - start).seconds());

  geometry_msgs::msg::TransformStamped tf;
  tf.header.frame_id = "base_link";
  tf.child_frame_id = "left_track";
  tf.transform.rotation.w = 1.0;
  for (double d = -5.0; d < 5.0; d += 0.1) {
    tf.header.stamp = clock_ptr->now() + rclcpp::Duration::from_seconds(d);
    tfBuffer->setTransform(tf, "test");
  }

  start = clock_ptr->now();
  watchdog.searchForReachableFrames();
  end = clock_ptr->now();

  EXPECT_TRUE(watchdog.isReachable("left_track"));
  EXPECT_FALSE(watchdog.isReachable("front_left_flipper"));
  // we're searching for 2 frames, one of which should be found immediately; check that the search
  // took at least 1x lookup timeout time, but it did not take much more
  EXPECT_LE(1.0 * watchdog.unreachable_tf_lookup_timeout_.seconds(), (end - start).seconds());
  EXPECT_GE(1.5 * watchdog.unreachable_tf_lookup_timeout_.seconds(), (end - start).seconds());

  tf.header.frame_id = "left_track";
  tf.child_frame_id = "front_left_flipper";
  tf.transform.rotation.w = 1.0;
  for (double d = -5.0; d < 5.0; d += 0.1) {
    tf.header.stamp = clock_ptr->now() + rclcpp::Duration::from_seconds(d);
    tfBuffer->setTransform(tf, "test");
  }

  watchdog.searchForReachableFrames();
  EXPECT_TRUE(watchdog.isReachable("left_track"));
  EXPECT_TRUE(watchdog.isReachable("front_left_flipper"));
}

TEST(TfFramesWatchdog, LookupTransform) {
  const auto nh = std::make_shared<rclcpp::Node>("test_ray_casting_shape_mask");
  const auto clock_ptr = nh->get_clock();
  const auto tfBuffer = std::make_shared<tf2_ros::Buffer>(nh->get_clock(), tf2::BUFFER_CORE_DEFAULT_CACHE_TIME, nh);
  tfBuffer->setUsingDedicatedThread(true);
  TestWatchdog watchdog(
    nh, "base_link", {"left_track", "front_left_flipper"}, tfBuffer,
    rclcpp::Duration::from_seconds(0.1), std::make_shared<rclcpp::Rate>(1.0));

  EXPECT_THROW(
    watchdog.lookupTransform("left_track", clock_ptr->now(), rclcpp::Duration::from_seconds(1)),
    std::runtime_error);

  watchdog.started_ = true;  // fake the running thread

  EXPECT_FALSE(watchdog.isReachable("left_track"));
  auto resTf = watchdog.lookupTransform("left_track", clock_ptr->now(), rclcpp::Duration::from_seconds(1));
  EXPECT_FALSE(resTf.has_value());

  // if the frame is marked reachable and canTransform fails, it is marked unreachable
  watchdog.markReachable("left_track");
  rclcpp::Time start = clock_ptr->now();
  resTf = watchdog.lookupTransform("left_track", clock_ptr->now(), rclcpp::Duration::from_seconds(1));
  rclcpp::Time end = clock_ptr->now();
  EXPECT_FALSE(watchdog.isReachable("left_track"));
  EXPECT_FALSE(resTf.has_value());
  // check that the lookup took at least the amount of time specified by timeout, but not much more
  EXPECT_LE(1.0, (end - start).seconds());
  EXPECT_GE(1.5, (end - start).seconds());

  geometry_msgs::msg::TransformStamped tf;
  tf.header.frame_id = "base_link";
  tf.child_frame_id = "left_track";
  tf.transform.rotation.w = 1.0;
  for (double d = -5.0; d < 5.0; d += 0.1) {
    tf.header.stamp = clock_ptr->now() + rclcpp::Duration::from_seconds(d);
    tfBuffer->setTransform(tf, "test");
  }

  // if the transform is there but the frame is marked unreachable, lookupTransform should fail

  resTf = watchdog.lookupTransform("left_track", clock_ptr->now(), rclcpp::Duration::from_seconds(1));
  EXPECT_FALSE(watchdog.isReachable("left_track"));
  EXPECT_FALSE(resTf.has_value());

  // if looking up an unmonitored frame, the first lookup fails, but sets the frame as monitored
  EXPECT_FALSE(watchdog.isMonitored("rear_left_flipper"));
  EXPECT_FALSE(watchdog.isReachable("rear_left_flipper"));
  resTf = watchdog.lookupTransform("rear_left_flipper", clock_ptr->now(), rclcpp::Duration::from_seconds(1));
  EXPECT_TRUE(watchdog.isMonitored("rear_left_flipper"));
  EXPECT_FALSE(watchdog.isReachable("rear_left_flipper"));
  EXPECT_FALSE(resTf.has_value());

  // look up a transform that is monitored, reachable and available in the buffer

  tf.child_frame_id = "rear_left_flipper";
  tf.transform.translation.x = 1.0;
  tf.transform.translation.y = 2.0;
  tf.transform.translation.z = 3.0;
  tf.transform.rotation.w = 1.0;
  for (double d = -5.0; d < 5.0; d += 0.1) {
    tf.header.stamp = clock_ptr->now() + rclcpp::Duration::from_seconds(d);
    tfBuffer->setTransform(tf, "test");
  }

  EXPECT_TRUE(watchdog.isMonitored("rear_left_flipper"));
  EXPECT_FALSE(watchdog.isReachable("rear_left_flipper"));
  watchdog.markReachable("rear_left_flipper");
  rclcpp::Time time = clock_ptr->now();
  resTf = watchdog.lookupTransform("rear_left_flipper", time, rclcpp::Duration::from_seconds(1));
  EXPECT_TRUE(watchdog.isMonitored("rear_left_flipper"));
  EXPECT_TRUE(watchdog.isReachable("rear_left_flipper"));
  ASSERT_TRUE(resTf.has_value());
  EXPECT_EQ("base_link", resTf.value().header.frame_id);
  EXPECT_EQ("rear_left_flipper", resTf.value().child_frame_id);
  EXPECT_EQ(time, resTf.value().header.stamp);
  EXPECT_DOUBLE_EQ(1.0, resTf.value().transform.translation.x);
  EXPECT_DOUBLE_EQ(2.0, resTf.value().transform.translation.y);
  EXPECT_DOUBLE_EQ(3.0, resTf.value().transform.translation.z);
  EXPECT_DOUBLE_EQ(0.0, resTf.value().transform.rotation.x);
  EXPECT_DOUBLE_EQ(0.0, resTf.value().transform.rotation.y);
  EXPECT_DOUBLE_EQ(0.0, resTf.value().transform.rotation.z);
  EXPECT_DOUBLE_EQ(1.0, resTf.value().transform.rotation.w);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
