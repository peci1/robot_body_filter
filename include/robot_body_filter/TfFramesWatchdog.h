#ifndef ROBOT_BODY_FILTER_TFFRAMESWATCHDOG_H
#define ROBOT_BODY_FILTER_TFFRAMESWATCHDOG_H

#include <mutex>
#include <set>
#include <string>
#include <thread>

#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <robot_body_filter/utils/optional.hpp>

namespace robot_body_filter {

/**
 * \brief Provide quick access to TFs while simultaneousely monitoring if some
 *        frames haven't got unreachable (in which case the node tries to get
 *        the transforms with longer timeouts but doesn't block other queries).
 *
 * Runs a separate thread.
 *
 * \author Martin Pecka
 */
class TFFramesWatchdog {
public:
  TFFramesWatchdog(std::string robotFrame,
                   std::set<std::string>  monitoredFrames,
                   std::shared_ptr<tf2_ros::Buffer> tfBuffer,
                   ros::Duration unreachableTfLookupTimeout = ros::Duration(0, 100000000),  // 0.1 sec
                   ros::Rate unreachableFramesCheckRate = ros::Rate(1.0));

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
   * \param monitored_frames Set of frames to be monitored.
   */
  void setMonitoredFrames(std::set<std::string> monitoredFrames);

  /**
   * \brief Add the given frame to the set of monitored frames (if it is not
   * already there).
   * \param monitoredFrame Name of the frame.
   */
  void addMonitoredFrame(const std::string& monitoredFrame);

  /**
   * \brief Return whether the given frame is monitored by this watchdog.
   * \param frame TF frame.
   * \return Whether the frame is monitored.
   */
  bool isMonitored(const std::string& frame) const;

  /**
   * \brief Return whether the given frame is reachable.
   * \param frame TF frame.
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
   * \param frame Source frame.
   * \param time Time of transform.
   * \param timeout Timeout for waiting for reachable transforms.
   * \param errstr Optional error string.
   * \return If the lookup succeeded, returns the transform.
   * \throws tf2::TransformException If a normal canTransform or lookupTransform
   *         would throw except for transform not found exceptions, which just
   *         mark the frame as unreachable.
   * \throws std::runtime_exception If you call this function before a call to
   *         start().
   */
  optional<geometry_msgs::TransformStamped> lookupTransform(
      const std::string& frame,
      const ros::Time& time,
      const ros::Duration& timeout,
      std::string* errstr = nullptr);

protected:
  /**
   * \brief Return whether the given frame is reachable.
   * \param frame TF frame.
   * \return Whether the frame is reachable.
   * \note The caller has to hold a lock to framesMutex.
   */
  bool isReachableNoLock(const std::string& frame) const;

  /**
   * \brief Return whether the given frame is monitored by this watchdog.
   * \param frame TF frame.
   * \return Whether the frame is monitored.
   * \note The caller has to hold a lock to framesMutex.
   */
  bool isMonitoredNoLock(const std::string& frame) const;

  /**
   * \brief Add the given frame to the set of monitored frames (if it is not
   * already there).
   * \param monitoredFrame Name of the frame.
   * \note The caller has to hold a lock to framesMutex.
   */
  void addMonitoredFrameNoLock(const std::string& monitoredFrame);

  /**
   * \brief Mark the given frame as reachable.
   * \param frame The frame to mark as reachable.
   */
  void markReachable(const std::string& frame);

  /**
   * \brief Mark the given frame as unreachable.
   * \param frame The frame to mark as unreachable.
   */
  void markUnreachable(const std::string& frame);

  /**
   * \brief Perform the search for reachable frames.
   */
  void searchForReachableFrames();

  //! The target frame of all watched transforms.
  std::string robotFrame;
  //! List of source frames for which TFs to robot_frame are available.
  std::set<std::string> reachableFrames;
  //! Set of frames to be watched
  std::set<std::string> monitoredFrames;

  //! If true, this thread is paused.
  volatile bool paused = true;
  //! True if the watchdog thread has been started.
  bool started = false;
  //! If true, the watchdog should stop its execution. Blocks until the
  //! execution thread exits.
  volatile bool shouldStop = false;

  //! TF buffer
  std::shared_ptr<tf2_ros::Buffer> tfBuffer;

  //! Timeout for canTransform() for figuring out if an unreachable frame became reachable.
  ros::Duration unreachableTfLookupTimeout;
  //! Rate at which checking for unreachable frames will be done.
  ros::Rate unreachableFramesCheckRate;

  //! Lock this mutex any time you want to work with monitoredFrames or reachableFrames.
  mutable std::mutex framesMutex;

private:
  std::thread thisThread;
};

}

#endif //ROBOT_BODY_FILTER_TFFRAMESWATCHDOG_H
