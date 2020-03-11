#include "gtest/gtest.h"
#include <robot_body_filter/TfFramesWatchdog.h>

using namespace robot_body_filter;

class TestWatchdog : public TFFramesWatchdog
{
  public:
  TestWatchdog(const std::string &robotFrame,
               const std::set<std::string> &monitoredFrames,
               const std::shared_ptr<tf2_ros::Buffer> &tfBuffer,
               const ros::Duration &unreachableTfLookupTimeout,
               const ros::Rate &unreachableFramesCheckRate) :
    TFFramesWatchdog(robotFrame, monitoredFrames, tfBuffer, unreachableTfLookupTimeout,
        unreachableFramesCheckRate)
  {

  }

  friend class TfFramesWatchdog_Basic_Test;
  friend class TfFramesWatchdog_ThreadControl_Test;
  friend class TfFramesWatchdog_SearchForReachableFrames_Test;
  friend class TfFramesWatchdog_LookupTransform_Test;
};

TEST(TfFramesWatchdog, Basic)
{
  ros::Time::init();
  ros::Time::setNow(ros::Time(1));

  const std::shared_ptr<tf2_ros::Buffer> tfBuffer(new tf2_ros::Buffer());
  TestWatchdog watchdog("base_link", {"left_track", "front_left_flipper"}, tfBuffer,
      ros::Duration(0.1), ros::Rate(1.0));

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

TEST(TfFramesWatchdog, ThreadControl)
{
  ros::Time::init();
  ros::Time::setNow(ros::Time(1));

  const std::shared_ptr<tf2_ros::Buffer> tfBuffer(new tf2_ros::Buffer());
  TestWatchdog watchdog("base_link", {"left_track", "front_left_flipper"}, tfBuffer,
                        ros::Duration(0.1), ros::Rate(1.0));

  EXPECT_FALSE(watchdog.started);
  EXPECT_TRUE(watchdog.paused);
  EXPECT_FALSE(watchdog.shouldStop);

  watchdog.unpause();
  EXPECT_FALSE(watchdog.started);
  EXPECT_FALSE(watchdog.paused);
  EXPECT_FALSE(watchdog.shouldStop);

  watchdog.pause();
  EXPECT_FALSE(watchdog.started);
  EXPECT_TRUE(watchdog.paused);
  EXPECT_FALSE(watchdog.shouldStop);

  watchdog.stop();
  EXPECT_FALSE(watchdog.started);
  EXPECT_TRUE(watchdog.paused);
  EXPECT_TRUE(watchdog.shouldStop);

  watchdog.start();
  for (size_t i = 0; i < 100; ++i)
  {
    if (watchdog.started)
      break;
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_TRUE(watchdog.started);
  EXPECT_FALSE(watchdog.paused);
  EXPECT_FALSE(watchdog.shouldStop);

  watchdog.stop();
  EXPECT_TRUE(watchdog.started);
  EXPECT_TRUE(watchdog.paused);
  EXPECT_TRUE(watchdog.shouldStop);

  // test that the watchdog can be re-run
  watchdog.start();
  for (size_t i = 0; i < 100; ++i)
  {
    if (watchdog.started)
      break;
    ros::WallDuration(0.01).sleep();
  }
  EXPECT_TRUE(watchdog.started);
  EXPECT_FALSE(watchdog.paused);
  EXPECT_FALSE(watchdog.shouldStop);

  watchdog.stop();
  EXPECT_TRUE(watchdog.started);
  EXPECT_TRUE(watchdog.paused);
  EXPECT_TRUE(watchdog.shouldStop);
}

TEST(TfFramesWatchdog, SearchForReachableFrames)
{
  ros::Time::init(); // use system time for this test so that we don't have problems with timeouts

  const std::shared_ptr<tf2_ros::Buffer> tfBuffer(new tf2_ros::Buffer());
  tfBuffer->setUsingDedicatedThread(true);
  TestWatchdog watchdog("base_link", {"left_track", "front_left_flipper"}, tfBuffer,
                        ros::Duration(0.1), ros::Rate(1.0));

  watchdog.unpause(); // searchForReachableFrames checks this->paused

  ros::Time start = ros::Time::now();
  watchdog.searchForReachableFrames();
  ros::Time end = ros::Time::now();

  EXPECT_FALSE(watchdog.isReachable("left_track"));
  EXPECT_FALSE(watchdog.isReachable("front_left_flipper"));
  // we're searching for 2 frames; check that the search took at least 2x lookup timeout time, but
  // it did not take much more
  EXPECT_LE(2.0 * watchdog.unreachableTfLookupTimeout.toSec(), (end - start).toSec());
  EXPECT_GE(2.5 * watchdog.unreachableTfLookupTimeout.toSec(), (end - start).toSec());

  geometry_msgs::TransformStamped tf;
  tf.header.frame_id = "base_link";
  tf.child_frame_id = "left_track";
  tf.transform.rotation.w = 1.0;
  for (double d = -5.0; d < 5.0; d += 0.1)
  {
    tf.header.stamp = ros::Time::now() + ros::Duration(d);
    tfBuffer->setTransform(tf, "test");
  }

  start = ros::Time::now();
  watchdog.searchForReachableFrames();
  end = ros::Time::now();

  EXPECT_TRUE(watchdog.isReachable("left_track"));
  EXPECT_FALSE(watchdog.isReachable("front_left_flipper"));
  // we're searching for 2 frames, one of which should be found immediately; check that the search
  // took at least 1x lookup timeout time, but it did not take much more
  EXPECT_LE(1.0 * watchdog.unreachableTfLookupTimeout.toSec(), (end - start).toSec());
  EXPECT_GE(1.5 * watchdog.unreachableTfLookupTimeout.toSec(), (end - start).toSec());

  tf.header.frame_id = "left_track";
  tf.child_frame_id = "front_left_flipper";
  tf.transform.rotation.w = 1.0;
  for (double d = -5.0; d < 5.0; d += 0.1)
  {
    tf.header.stamp = ros::Time::now() + ros::Duration(d);
    tfBuffer->setTransform(tf, "test");
  }

  watchdog.searchForReachableFrames();
  EXPECT_TRUE(watchdog.isReachable("left_track"));
  EXPECT_TRUE(watchdog.isReachable("front_left_flipper"));
}

TEST(TfFramesWatchdog, LookupTransform)
{
  ros::Time::init(); // use system time for this test so that we don't have problems with timeouts

  const std::shared_ptr<tf2_ros::Buffer> tfBuffer(new tf2_ros::Buffer());
  tfBuffer->setUsingDedicatedThread(true);
  TestWatchdog watchdog("base_link", {"left_track", "front_left_flipper"}, tfBuffer,
                        ros::Duration(0.1), ros::Rate(1.0));

  EXPECT_THROW(watchdog.lookupTransform("left_track", ros::Time::now(), ros::Duration(1)), std::runtime_error);

  watchdog.started = true; // fake the running thread

  EXPECT_FALSE(watchdog.isReachable("left_track"));
  auto resTf = watchdog.lookupTransform("left_track", ros::Time::now(), ros::Duration(1));
  EXPECT_FALSE(resTf.has_value());

  // if the frame is marked reachable and canTransform fails, it is marked unreachable
  watchdog.markReachable("left_track");
  ros::Time start = ros::Time::now();
  resTf = watchdog.lookupTransform("left_track", ros::Time::now(), ros::Duration(1));
  ros::Time end = ros::Time::now();
  EXPECT_FALSE(watchdog.isReachable("left_track"));
  EXPECT_FALSE(resTf.has_value());
  // check that the lookup took at least the amount of time specified by timeout, but not much more
  EXPECT_LE(1.0, (end - start).toSec());
  EXPECT_GE(1.5, (end - start).toSec());

  geometry_msgs::TransformStamped tf;
  tf.header.frame_id = "base_link";
  tf.child_frame_id = "left_track";
  tf.transform.rotation.w = 1.0;
  for (double d = -5.0; d < 5.0; d += 0.1)
  {
    tf.header.stamp = ros::Time::now() + ros::Duration(d);
    tfBuffer->setTransform(tf, "test");
  }

  // if the transform is there but the frame is marked unreachable, lookupTransform should fail

  resTf = watchdog.lookupTransform("left_track", ros::Time::now(), ros::Duration(1));
  EXPECT_FALSE(watchdog.isReachable("left_track"));
  EXPECT_FALSE(resTf.has_value());

  // if looking up an unmonitored frame, the first lookup fails, but sets the frame as monitored
  EXPECT_FALSE(watchdog.isMonitored("rear_left_flipper"));
  EXPECT_FALSE(watchdog.isReachable("rear_left_flipper"));
  resTf = watchdog.lookupTransform("rear_left_flipper", ros::Time::now(), ros::Duration(1));
  EXPECT_TRUE(watchdog.isMonitored("rear_left_flipper"));
  EXPECT_FALSE(watchdog.isReachable("rear_left_flipper"));
  EXPECT_FALSE(resTf.has_value());

  // look up a transform that is monitored, reachable and available in the buffer

  tf.child_frame_id = "rear_left_flipper";
  tf.transform.translation.x = 1.0;
  tf.transform.translation.y = 2.0;
  tf.transform.translation.z = 3.0;
  tf.transform.rotation.w = 1.0;
  for (double d = -5.0; d < 5.0; d += 0.1)
  {
    tf.header.stamp = ros::Time::now() + ros::Duration(d);
    tfBuffer->setTransform(tf, "test");
  }

  EXPECT_TRUE(watchdog.isMonitored("rear_left_flipper"));
  EXPECT_FALSE(watchdog.isReachable("rear_left_flipper"));
  watchdog.markReachable("rear_left_flipper");
  ros::Time time = ros::Time::now();
  resTf = watchdog.lookupTransform("rear_left_flipper", time, ros::Duration(1));
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

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}