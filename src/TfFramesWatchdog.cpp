// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <algorithm>
#include <iterator>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <utility>

#include <cras_cpp_common/time_utils.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>
#include <robot_body_filter/TfFramesWatchdog.hpp>
#include <tf2/exceptions.hpp>
#include <tf2_ros/buffer.hpp>

namespace robot_body_filter {
TFFramesWatchdog::TFFramesWatchdog(
  rclcpp::Logger logger, const rclcpp::Clock::SharedPtr& clock,
  std::string robot_frame, std::set<std::string> monitored_frames,
  std::shared_ptr<tf2_ros::Buffer> tf_buffer, rclcpp::Duration unreachable_tf_lookup_timeout,
  rclcpp::Rate::SharedPtr unreachable_frames_check_rate)
  : robot_frame_(std::move(robot_frame)), monitored_frames_(std::move(monitored_frames)),
    tf_buffer_(std::move(tf_buffer)), unreachable_tf_lookup_timeout_(std::move(unreachable_tf_lookup_timeout)),
    unreachable_frames_check_rate_(std::move(unreachable_frames_check_rate)),
    logger_(logger.get_child("tf_frames_watchdog")), clock_(clock) {
  if (unreachable_frames_check_rate_ == nullptr) {
    unreachable_frames_check_rate_ = std::make_shared<rclcpp::Rate>(1.0, clock_);
  }
}

void TFFramesWatchdog::start() {
  should_stop_ = false;
  this_thread_ = std::thread(&TFFramesWatchdog::run, this);
  unpause();
}

void TFFramesWatchdog::run() {
  started_ = true;

  while (!should_stop_ && rclcpp::ok()) {
    if (!paused_) {  // the thread is paused_ when we want to change the stuff protected by frames_mutex_.
      searchForReachableFrames();
    }
    unreachable_frames_check_rate_->sleep();
  }
}

bool TFFramesWatchdog::isRunning() const {
  return started_;
}

void TFFramesWatchdog::searchForReachableFrames() {
  const rclcpp::Time time = clock_->now();

  // detect all unreachable frames
  // we can't join this loop with the following one, because we need to lock the frames_mutex_, but
  // canTransform can hang for pretty long, which would result in deadlocks

  std::set<std::string> unreachable_frames;
  {
    std::lock_guard<std::mutex> guard(frames_mutex_);
    std::set_difference(
      monitored_frames_.begin(), monitored_frames_.end(),
      reachable_frames_.begin(), reachable_frames_.end(),
      std::inserter(unreachable_frames, unreachable_frames.end()));
  }

  // now, the mutex is unlocked and we try to get transforms to all of the unreachable links... that
  // could take a while
  for (auto& frame : unreachable_frames) {
    if (paused_) {
      break;
    }
    std::string err;
    if (tf_buffer_->canTransform(robot_frame_, frame, time, unreachable_tf_lookup_timeout_, &err)) {
      markReachable(frame);
      RCLCPP_DEBUG(
        logger_, "TFFramesWatchdog (%s): Frame %s became reachable at %f.%li",
        robot_frame_.c_str(), frame.c_str(), time.seconds(), time.nanoseconds());
    } else {
      // TODO: Originally delayed-throttle
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 3, "TFFramesWatchdog (%s): Frame %s is not reachable! Cause: %s",
        robot_frame_.c_str(), frame.c_str(), err.c_str());
    }
  }
}

void TFFramesWatchdog::pause() {
  paused_ = true;
}

void TFFramesWatchdog::unpause() {
  paused_ = false;
}

void TFFramesWatchdog::stop() {
  RCLCPP_INFO(logger_, "Stopping TF watchdog.");
  should_stop_ = true;
  paused_ = true;

  if (started_ && this_thread_.joinable()) {
    this_thread_.join();  // segfaults without this line
  }
  RCLCPP_INFO(logger_, "TF watchdog stopped.");
}

void TFFramesWatchdog::clear() {
  std::lock_guard<std::mutex> guard(frames_mutex_);
  monitored_frames_.clear();
  tf_buffer_->clear();
  reachable_frames_.clear();
}

std::optional<geometry_msgs::msg::TransformStamped> TFFramesWatchdog::lookupTransform(
  const std::string& frame, const rclcpp::Time& time, const rclcpp::Duration& timeout, std::string* errstr) {
  if (!started_) {
    throw std::runtime_error("TFFramesWatchdog has not been started.");
  }

  {
    std::lock_guard<std::mutex> guard(frames_mutex_);
    if (!isMonitoredNoLock(frame)) {
      RCLCPP_WARN(
        logger_, "TFFramesWatchdog (%s): Frame %s is not yet monitored, starting monitoring it.",
        robot_frame_.c_str(), frame.c_str());
      addMonitoredFrameNoLock(frame);
      // this lookup is lost, same as if the frame is unreachable
      return {};
    }

    // Return immediately for unreachable frames
    if (!isReachableNoLock(frame)) {
      return {};
    }
  }

  std::string tmp_errstr;
  if (errstr == nullptr) {
    errstr = &tmp_errstr;
  }

  if (!tf_buffer_->canTransform(
    robot_frame_, frame, time, cras::remainingTime(time, timeout, clock_), errstr)) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 3, "TFFramesWatchdog (%s): Frame %s became unreachable. Cause: %s",
      robot_frame_.c_str(), frame.c_str(), errstr->c_str());

    // if we couldn't get TF for this reachable frame, mark it unreachable
    markUnreachable(frame);
    return {};
  }

  try {
    return tf_buffer_->lookupTransform(
      robot_frame_, frame, time, cras::remainingTime(time, timeout, clock_));
  } catch (tf2::LookupException&) {
    RCLCPP_WARN_THROTTLE(
      logger_, *clock_, 3, "TFFramesWatchdog (%s): Frame %s is not reachable. Cause: %s",
      robot_frame_.c_str(), frame.c_str(), errstr->c_str());

    // if we couldn't get TF for this reachable frame, mark it unreachable
    markUnreachable(frame);
    return {};
  }
}

void TFFramesWatchdog::setMonitoredFrames(std::set<std::string> monitored_frames) {
  std::lock_guard<std::mutex> guard(frames_mutex_);
  monitored_frames_ = std::move(monitored_frames);

  // if some monitored frames disappeared, delete them also from reachable_frames_
  for (auto& frame : reachable_frames_) {
    if (monitored_frames_.find(frame) == monitored_frames_.end()) {
      reachable_frames_.erase(frame);
    }
  }
}

void TFFramesWatchdog::addMonitoredFrame(const std::string& monitored_frame) {
  std::lock_guard<std::mutex> guard(frames_mutex_);
  addMonitoredFrameNoLock(monitored_frame);
}

void TFFramesWatchdog::addMonitoredFrameNoLock(const std::string& monitored_frame) {
  monitored_frames_.insert(monitored_frame);
}

bool TFFramesWatchdog::isReachable(const std::string& frame) const {
  std::lock_guard<std::mutex> guard(frames_mutex_);
  return isReachableNoLock(frame);
}

bool TFFramesWatchdog::isReachableNoLock(const std::string& frame) const {
  return reachable_frames_.find(frame) != reachable_frames_.end();
}

void TFFramesWatchdog::markReachable(const std::string& frame) {
  std::lock_guard<std::mutex> guard(frames_mutex_);
  reachable_frames_.insert(frame);
}

void TFFramesWatchdog::markUnreachable(const std::string& frame) {
  std::lock_guard<std::mutex> guard(frames_mutex_);
  reachable_frames_.erase(frame);
}

bool TFFramesWatchdog::areAllFramesReachable() const {
  std::lock_guard<std::mutex> guard(frames_mutex_);
  return reachable_frames_.size() == monitored_frames_.size();
}

bool TFFramesWatchdog::isMonitored(const std::string& frame) const {
  std::lock_guard<std::mutex> guard(frames_mutex_);
  return isMonitoredNoLock(frame);
}

bool TFFramesWatchdog::isMonitoredNoLock(const std::string& frame) const {
  return monitored_frames_.find(frame) != monitored_frames_.end();
}

TFFramesWatchdog::~TFFramesWatchdog() {
  stop();
}

}  // namespace robot_body_filter
