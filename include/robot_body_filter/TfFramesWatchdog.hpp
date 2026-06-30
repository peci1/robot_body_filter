#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <string>
#include <thread>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/rate.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

namespace robot_body_filter {

/**
 * \brief Provide quick access to TFs while simultaneously monitoring if some
 *        frames haven't got unreachable (in which case the node tries to get
 *        the transforms with longer timeouts but doesn't block other queries).
 *
 * Runs a separate thread.
 *
 * \author Martin Pecka
 */
class TFFramesWatchdog {
public:
  TFFramesWatchdog(
    rclcpp::Logger logger, const rclcpp::Clock::SharedPtr& clock,
    std::string robot_frame, std::set<std::string> monitored_frames, std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    rclcpp::Duration unreachable_tf_lookup_timeout = rclcpp::Duration(0, 100000000),  // 0.1 sec
    rclcpp::Rate::SharedPtr unreachable_frames_check_rate = nullptr);

  virtual ~TFFramesWatchdog();

  /** Start the updater using a thread.
   */
  void start();

  /** Run the updater (should be run in a separate thread).
   */
  void run();

  /**
   * \brief Return true if the watchdog is running.
   * \return Whether the watchdog is running or not.
   */
  bool isRunning() const;

  /**
   * \brief Pause thread execution.
   */
  void pause();

  /**
   * \brief Unpause thread execution.
   */
  void unpause();

  /**
   * \brief Stop the watchdog for good. Can only be called once.
   */
  void stop();

  /**
   * \brief Clear shapes_to_links, reachable_frames and tf_buffer.
   */
  void clear();

  /**
   * \brief TF frames to be monitored by this watchdog.
   * \param[in] monitored_frames Set of frames to be monitored.
   */
  void setMonitoredFrames(std::set<std::string> monitored_frames);

  /**
   * \brief Add the given frame to the set of monitored frames (if it is not
   * already there).
   * \param[in] monitored_frame Name of the frame.
   */
  void addMonitoredFrame(const std::string& monitored_frame);

  /**
   * \brief Return whether the given frame is monitored by this watchdog.
   * \param[in] frame TF frame.
   * \return Whether the frame is monitored.
   */
  bool isMonitored(const std::string& frame) const;

  /**
   * \brief Return whether the given frame is reachable.
   * \param[in] frame TF frame.
   * \return Whether the frame is reachable.
   */
  bool isReachable(const std::string& frame) const;

  /**
   * \brief Return whether all monitored frames are reachable.
   * \return Whether all monitored frames are reachable.
   */
  bool areAllFramesReachable() const;

  /**
   * \brief Looks for a transform if it is marked reachable. Returns immediately
   *        for transforms marked unreachable.
   * \param[in] frame Source frame.
   * \param[in] time Time of transform.
   * \param[in] timeout Timeout for waiting for reachable transforms.
   * \param[out] errstr Optional error string.
   * \return If the lookup succeeded, returns the transform.
   * \throws tf2::TransformException If a normal canTransform or lookupTransform
   *         would throw except for transform not found exceptions, which just
   *         mark the frame as unreachable.
   * \throws std::runtime_exception If you call this function before a call to
   *         start().
   */
  std::optional<geometry_msgs::msg::TransformStamped> lookupTransform(
    const std::string& frame, const rclcpp::Time& time, const rclcpp::Duration& timeout, std::string* errstr = nullptr);

protected:
  /**
   * \brief Return whether the given frame is reachable.
   * \param[in] frame TF frame.
   * \return Whether the frame is reachable.
   * \note The caller has to hold a lock to framesMutex.
   */
  bool isReachableNoLock(const std::string& frame) const;

  /**
   * \brief Return whether the given frame is monitored by this watchdog.
   * \param[in] frame TF frame.
   * \return Whether the frame is monitored.
   * \note The caller has to hold a lock to framesMutex.
   */
  bool isMonitoredNoLock(const std::string& frame) const;

  /**
   * \brief Add the given frame to the set of monitored frames (if it is not already there).
   * \param[in] monitored_frame Name of the frame.
   * \note The caller has to hold a lock to framesMutex.
   */
  void addMonitoredFrameNoLock(const std::string& monitored_frame);

  /**
   * \brief Mark the given frame as reachable.
   * \param[in] frame The frame to mark as reachable.
   */
  void markReachable(const std::string& frame);

  /**
   * \brief Mark the given frame as unreachable.
   * \param[in] frame The frame to mark as unreachable.
   */
  void markUnreachable(const std::string& frame);

  /**
   * \brief Perform the search for reachable frames.
   */
  void searchForReachableFrames();

  //! The target frame of all watched transforms.
  std::string robot_frame_;
  //! List of source frames for which TFs to robot_frame are available.
  std::set<std::string> reachable_frames_;
  //! Set of frames to be watched
  std::set<std::string> monitored_frames_;

  //! If true, this thread is paused_.
  volatile bool paused_ = true;
  //! True if the watchdog thread has been started.
  bool started_ = false;
  //! If true, the watchdog should stop its execution. Blocks until the
  //! execution thread exits.
  volatile bool should_stop_ = false;

  //! TF buffer
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  //! Timeout for canTransform() for figuring out if an unreachable frame became reachable.
  rclcpp::Duration unreachable_tf_lookup_timeout_;
  //! Rate at which checking for unreachable frames will be done.
  rclcpp::Rate::SharedPtr unreachable_frames_check_rate_;

  //! Lock this mutex any time you want to work with monitored_frames_ or reachable_frames_.
  mutable std::mutex frames_mutex_;

private:
  std::thread this_thread_;

  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;
};

}  // namespace robot_body_filter
