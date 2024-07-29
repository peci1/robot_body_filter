#include <utility>

#include <robot_body_filter/TfFramesWatchdog.h>

#include <robot_body_filter/utils/time_utils.hpp>

namespace robot_body_filter
{
TFFramesWatchdog::TFFramesWatchdog(
                                   rclcpp::Node::SharedPtr nodeHandle,
                                   std::string robotFrame,
                                   std::set<std::string>  monitoredFrames,
                                   std::shared_ptr<tf2_ros::Buffer> tfBuffer,
                                   rclcpp::Duration unreachableTfLookupTimeout,
                                   rclcpp::Rate::SharedPtr unreachableFramesCheckRate):
    nodeHandle(nodeHandle),                               
    robotFrame(std::move(robotFrame)),
    monitoredFrames(std::move(monitoredFrames)),
    tfBuffer(tfBuffer),
    unreachableTfLookupTimeout(std::move(unreachableTfLookupTimeout)),
    unreachableFramesCheckRate(std::move(unreachableFramesCheckRate)) {
}

void TFFramesWatchdog::start() {
  this->shouldStop = false;
  this->thisThread = std::thread(&TFFramesWatchdog::run, this);
  this->unpause();
}

void TFFramesWatchdog::run() {
  this->started = true;

  while (!this->shouldStop && rclcpp::ok()) {
    if (!this->paused) { // the thread is paused when we want to change the stuff protected by framesMutex.
      this->searchForReachableFrames();
    }
    this->unreachableFramesCheckRate->sleep();
  }
}

bool TFFramesWatchdog::isRunning() const {
  return this->started;
}

void TFFramesWatchdog::searchForReachableFrames()
{

  const rclcpp::Time time = nodeHandle->now();

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
    RCLCPP_DEBUG(this->nodeHandle->get_logger(), "robot frame: %s, checking against: %s" , this->robotFrame.c_str(), frame.c_str());
    if (this->tfBuffer->canTransform(this->robotFrame, frame, time, this->unreachableTfLookupTimeout, &err)) {
      this->markReachable(frame);
      RCLCPP_DEBUG(this->nodeHandle->get_logger(),"TFFramesWatchdog (%s): Frame %s became reachable at %f.%li",
          this->robotFrame.c_str(), frame.c_str(), time.seconds(), time.nanoseconds());
    } else {
      auto &clk = *nodeHandle->get_clock();
      RCLCPP_WARN_THROTTLE(this->nodeHandle->get_logger(), clk, 3,
                           "TFFramesWatchdog (%s): Frame %s is not reachable! Cause: %s", this->robotFrame.c_str(),
                           frame.c_str(), err.c_str());
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
  RCLCPP_INFO(this->nodeHandle->get_logger(), "Stopping TF watchdog.");
  this->shouldStop = true;
  this->paused = true;

  if (this->started && this->thisThread.joinable())
    this->thisThread.join(); // segfaults without this line
  RCLCPP_INFO(this->nodeHandle->get_logger(), "TF watchdog stopped.");
}

void TFFramesWatchdog::clear() {
  std::lock_guard<std::mutex> guard(this->framesMutex);
  monitoredFrames.clear();
  tfBuffer->clear();
  reachableFrames.clear();
}

optional<geometry_msgs::msg::TransformStamped> TFFramesWatchdog::lookupTransform(
    const std::string &frame,
    const rclcpp::Time &time,
    const rclcpp::Duration &timeout,
    std::string *errstr)
{
  if (!this->started)
    throw std::runtime_error("TFFramesWatchdog has not been started.");

  {
    std::lock_guard<std::mutex> guard(this->framesMutex);
    if (!this->isMonitoredNoLock(frame))
    {
      RCLCPP_WARN(this->nodeHandle->get_logger(), "TFFramesWatchdog (%s): Frame %s is not yet monitored, starting "
          "monitoring it.", this->robotFrame.c_str(), frame.c_str());
      this->addMonitoredFrameNoLock(frame);
      // this lookup is lost, same as if the frame is unreachable
      return nullopt;
    }

    // Return immediately for unreachable frames
    if (!this->isReachableNoLock(frame))
      return nullopt;
  }

  std::string tmpErrstr;
  if (errstr == nullptr) {
    errstr = &tmpErrstr;
  }

  if (!this->tfBuffer->canTransform(this->robotFrame, frame, time,
      remainingTime(*this->nodeHandle->get_clock(), time, timeout), errstr)) {
    auto &clk = *nodeHandle->get_clock();
    RCLCPP_WARN_THROTTLE(this->nodeHandle->get_logger(), clk, 3,
                         "TFFramesWatchdog (%s): Frame %s became unreachable. Cause: %s", this->robotFrame.c_str(),
                         frame.c_str(), errstr->c_str());
    // if we couldn't get TF for this reachable frame, mark it unreachable
    this->markUnreachable(frame);
    return nullopt;
  }

  try
  {
    return this->tfBuffer->lookupTransform(
        this->robotFrame, frame, time, remainingTime(*this->nodeHandle->get_clock(), time, timeout));
  } catch (tf2::LookupException&) {
    auto &clk = *nodeHandle->get_clock();
    RCLCPP_WARN_THROTTLE(this->nodeHandle->get_logger(), clk, 3,
        "TFFramesWatchdog (%s): Frame %s is not reachable. Cause: %s",
        this->robotFrame.c_str(), frame.c_str(), errstr->c_str());

    // if we couldn't get TF for this reachable frame, mark it unreachable
    this->markUnreachable(frame);
    return nullopt;
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

void TFFramesWatchdog::addMonitoredFrame(const std::string& monitoredFrame)
{
  std::lock_guard<std::mutex> guard(this->framesMutex);
  this->addMonitoredFrameNoLock(monitoredFrame);
}

void TFFramesWatchdog::addMonitoredFrameNoLock(const std::string& monitoredFrame)
{
  this->monitoredFrames.insert(monitoredFrame);
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

bool TFFramesWatchdog::areAllFramesReachable() const
{
  std::lock_guard<std::mutex> guard(this->framesMutex);
  return this->reachableFrames.size() == this->monitoredFrames.size();
}
bool TFFramesWatchdog::isMonitored(const std::string &frame) const
{
  std::lock_guard<std::mutex> guard(this->framesMutex);
  return this->isMonitoredNoLock(frame);
}
bool TFFramesWatchdog::isMonitoredNoLock(const std::string &frame) const
{
  return this->monitoredFrames.find(frame) != this->monitoredFrames.end();
}

TFFramesWatchdog::~TFFramesWatchdog()
{
  this->stop();
}

}