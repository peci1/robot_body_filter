#include <utility>

#include <robot_body_filter/TfFramesWatchdog.h>

#include <robot_body_filter/utils/time_utils.hpp>

namespace robot_body_filter
{
TFFramesWatchdog::TFFramesWatchdog(std::string robotFrame,
                                   std::set<std::string>  monitoredFrames,
                                   std::shared_ptr<tf2_ros::Buffer> tfBuffer,
                                   ros::Duration unreachableTfLookupTimeout,
                                   ros::Rate unreachableFramesCheckRate):
    robotFrame(std::move(robotFrame)),
    monitoredFrames(std::move(monitoredFrames)),
    tfBuffer(tfBuffer),
    unreachableTfLookupTimeout(std::move(unreachableTfLookupTimeout)),
    unreachableFramesCheckRate(std::move(unreachableFramesCheckRate)) {
}

void TFFramesWatchdog::start() {
  this->thisThread = std::thread(&TFFramesWatchdog::run, this);
  this->unpause();
}

void TFFramesWatchdog::run() {
  this->started = true;

  while (!this->shouldStop && ros::ok()) {
    if (!this->paused) { // the thread is paused when we want to change the stuff protected by framesMutex.
      this->searchForReachableFrames();
    }
    this->unreachableFramesCheckRate.sleep();
  }
}

void TFFramesWatchdog::searchForReachableFrames()
{

  const ros::Time time = ros::Time::now();

  // detect all unreachable frames
  // we can't join this loop with the following one, because we need to lock the framesMutex, but
  // canTransform can hang for pretty long, which would result in deadlocks

  std::set<std::string> unreachableFrames;
  {
    std::lock_guard<std::mutex> guard(this->framesMutex);
    std::set_difference(
        this->monitoredFrames.begin(), this->monitoredFrames.end(),
        this->reachableFrames.begin(), this->reachableFrames.end(),
        std::inserter(unreachableFrames, unreachableFrames.end()));
  }

  // now, the mutex is unlocked and we try to get transforms to all of the unreachable links... that
  // could take a while
  for (auto &frame : unreachableFrames) {
    if (this->paused) {
      break;
    }
    std::string err;
    if (this->tfBuffer->canTransform(this->robotFrame, frame, time, this->unreachableTfLookupTimeout, &err)) {
      this->markReachable(frame);
      ROS_DEBUG("TFFramesWatchdog (%s): Frame %s became reachable at %i.%i",
          this->robotFrame.c_str(), frame.c_str(), time.sec, time.nsec);
    } else {
      ROS_WARN_THROTTLE(3,
          "TFFramesWatchdog (%s): Frame %s is not reachable! Cause: %s",
          this->robotFrame.c_str(), frame.c_str(), err.c_str());
    }
  }
}

void TFFramesWatchdog::pause() {
  this->paused = true;
}

void TFFramesWatchdog::unpause() {
  this->paused = false;
}

void TFFramesWatchdog::stop() {
  ROS_INFO("Stopping TF watchdog.");
  this->shouldStop = true;
  this->thisThread.join(); // segfaults without this line
  ROS_INFO("TF watchdog stopped.");
}

void TFFramesWatchdog::clear() {
  std::lock_guard<std::mutex> guard(this->framesMutex);
  monitoredFrames.clear();
  tfBuffer->clear();
  reachableFrames.clear();
}

std::optional<geometry_msgs::TransformStamped> TFFramesWatchdog::lookupTransform(
    const std::string &frame,
    const ros::Time &time,
    const ros::Duration &timeout,
    std::string *errstr)
{
  if (!this->started)
    throw std::runtime_error("TFFramesWatchdog has not been started.");

  // Return immediately for unreachable frames
  if (!this->isReachable(frame))
    return std::nullopt;

  if (!this->tfBuffer->canTransform(this->robotFrame, frame, time,
      remainingTime(time, timeout), errstr)) {
    ROS_WARN_THROTTLE(3,
        "TFFramesWatchdog (%s): Frame %s became unreachable. Cause: %s",
        this->robotFrame.c_str(), frame.c_str(), errstr->c_str());

    // if we couldn't get TF for this reachable frame, mark it unreachable
    this->markUnreachable(frame);
    return std::nullopt;
  }

  try
  {
    return this->tfBuffer->lookupTransform(
        this->robotFrame, frame, time, remainingTime(time, timeout));
  } catch (tf2::LookupException&) {
    ROS_WARN_THROTTLE(3,
        "TFFramesWatchdog (%s): Frame %s is not reachable. Cause: %s",
        this->robotFrame.c_str(), frame.c_str(), errstr->c_str());

    // if we couldn't get TF for this reachable frame, mark it unreachable
    this->markUnreachable(frame);
    return std::nullopt;
  }
}

void TFFramesWatchdog::setMonitoredFrames(std::set<std::string> monitoredFrames)
{
  std::lock_guard<std::mutex> guard(this->framesMutex);
  this->monitoredFrames = std::move(monitoredFrames);

  // if some monitored frames disappeared, delete them also from reachableFrames
  for (auto& frame: this->reachableFrames) {
    if (this->monitoredFrames.find(frame) == this->monitoredFrames.end())
      this->reachableFrames.erase(frame);
  }
}
bool TFFramesWatchdog::isReachable(const std::string &frame) const
{
  std::lock_guard<std::mutex> guard(this->framesMutex);
  return this->isReachableNoLock(frame);
}
bool TFFramesWatchdog::isReachableNoLock(const std::string &frame) const
{
  return this->reachableFrames.find(frame) != this->reachableFrames.end();
}
void TFFramesWatchdog::markReachable(const std::string &frame)
{
  std::lock_guard<std::mutex> guard(this->framesMutex);
  this->reachableFrames.insert(frame);
}
void TFFramesWatchdog::markUnreachable(const std::string &frame)
{
  std::lock_guard<std::mutex> guard(this->framesMutex);
  this->reachableFrames.erase(frame);
}

}