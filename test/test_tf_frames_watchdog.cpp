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

  watchdog.searchForReachableFrames();
  EXPECT_FALSE(watchdog.isReachable("left_track"));
  EXPECT_FALSE(watchdog.isReachable("front_left_flipper"));

  geometry_msgs::TransformStamped tf;
  tf.header.frame_id = "base_link";
  tf.child_frame_id = "left_track";
  tf.transform.rotation.w = 1.0;
  for (double d = -5.0; d < 5.0; d += 0.1)
  {
    tf.header.stamp = ros::Time::now() + ros::Duration(d);
    tfBuffer->setTransform(tf, "test");
  }

  watchdog.searchForReachableFrames();
  EXPECT_TRUE(watchdog.isReachable("left_track"));
  EXPECT_FALSE(watchdog.isReachable("front_left_flipper"));

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

}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}