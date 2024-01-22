#include <utility>

#include <functional>
#include <memory>

#include "robot_body_filter/RobotBodyFilter.h"

#include "pluginlib/class_list_macros.h"

#include <geometric_shapes/bodies.h>
#include <geometric_shapes/body_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shape_to_marker.h>

#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

#include <robot_body_filter/utils/bodies.h>
#include <robot_body_filter/utils/crop_box.h>
#include <robot_body_filter/utils/set_utils.hpp>
#include <robot_body_filter/utils/shapes.h>
#include <robot_body_filter/utils/string_utils.hpp>
#include <robot_body_filter/utils/tf2_eigen.h>
#include <robot_body_filter/utils/tf2_sensor_msgs.h>
#include <robot_body_filter/utils/time_utils.hpp>
#include <robot_body_filter/utils/urdf_eigen.hpp>

using namespace std;
using namespace sensor_msgs;
using namespace filters;

namespace robot_body_filter {

template<typename T>
RobotBodyFilter<T>::RobotBodyFilter() : privateNodeHandle("~") {
  this->modelMutex.reset(new std::mutex());
}

template<typename T>
bool RobotBodyFilter<T>::configure() {
  this->tfBufferLength = this->getParamVerbose("transforms/buffer_length", ros::Duration(60.0), "s");

  if (this->tfBuffer == nullptr)
  {
    this->tfBuffer = std::make_shared<tf2_ros::Buffer>(this->tfBufferLength);
    this->tfListener = std::make_unique<tf2_ros::TransformListener>(*this->tfBuffer);
  } else {
    // clear the TF buffer (useful if calling configure() after receiving old TF data)
    this->tfBuffer->clear();
  }

  this->fixedFrame = this->getParamVerbose("frames/fixed", "base_link");
  stripLeadingSlash(this->fixedFrame, true);
  this->sensorFrame = this->getParamVerbose("frames/sensor", "");
  stripLeadingSlash(this->sensorFrame, true);
  this->filteringFrame = this->getParamVerbose("frames/filtering", this->fixedFrame);
  stripLeadingSlash(this->filteringFrame, true);
  this->minDistance = this->getParamVerbose("sensor/min_distance", 0.0, "m");
  this->maxDistance = this->getParamVerbose("sensor/max_distance", 0.0, "m");
  this->robotDescriptionParam = this->getParamVerbose("body_model/robot_description_param", "robot_description");
  this->keepCloudsOrganized = this->getParamVerbose("filter/keep_clouds_organized", true);
  this->modelPoseUpdateInterval = this->getParamVerbose("filter/model_pose_update_interval", ros::Duration(0, 0), "s");
  const bool doClipping = this->getParamVerbose("filter/do_clipping", true);
  const bool doContainsTest = this->getParamVerbose("filter/do_contains_test", true);
  const bool doShadowTest = this->getParamVerbose("filter/do_shadow_test", true);
  const double maxShadowDistance = this->getParamVerbose("filter/max_shadow_distance", this->maxDistance, "m");
  this->reachableTransformTimeout = this->getParamVerbose("transforms/timeout/reachable", ros::Duration(0.1), "s");
  this->unreachableTransformTimeout = this->getParamVerbose("transforms/timeout/unreachable", ros::Duration(0.2), "s");
  this->requireAllFramesReachable = this->getParamVerbose("transforms/require_all_reachable", false);
  this->publishNoBoundingSpherePointcloud = this->getParamVerbose("bounding_sphere/publish_cut_out_pointcloud", false);
  this->publishNoBoundingBoxPointcloud = this->getParamVerbose("bounding_box/publish_cut_out_pointcloud", false);
  this->publishNoOrientedBoundingBoxPointcloud = this->getParamVerbose("oriented_bounding_box/publish_cut_out_pointcloud", false);
  this->publishNoLocalBoundingBoxPointcloud = this->getParamVerbose("local_bounding_box/publish_cut_out_pointcloud", false);
  this->computeBoundingSphere = this->getParamVerbose("bounding_sphere/compute", false) || this->publishNoBoundingSpherePointcloud;
  this->computeBoundingBox = this->getParamVerbose("bounding_box/compute", false) || this->publishNoBoundingBoxPointcloud;
  this->computeOrientedBoundingBox = this->getParamVerbose("oriented_bounding_box/compute", false) || this->publishNoOrientedBoundingBoxPointcloud;
  this->computeLocalBoundingBox = this->getParamVerbose("local_bounding_box/compute", false) || this->publishNoLocalBoundingBoxPointcloud;
  this->computeDebugBoundingSphere = this->getParamVerbose("bounding_sphere/debug", false);
  this->computeDebugBoundingBox = this->getParamVerbose("bounding_box/debug", false);
  this->computeDebugOrientedBoundingBox = this->getParamVerbose("oriented_bounding_box/debug", false);
  this->computeDebugLocalBoundingBox = this->getParamVerbose("local_bounding_box/debug", false);
  this->publishBoundingSphereMarker = this->getParamVerbose("bounding_sphere/marker", false);
  this->publishBoundingBoxMarker = this->getParamVerbose("bounding_box/marker", false);
  this->publishOrientedBoundingBoxMarker = this->getParamVerbose("oriented_bounding_box/marker", false);
  this->publishLocalBoundingBoxMarker = this->getParamVerbose("local_bounding_box/marker", false);
  this->localBoundingBoxFrame = this->getParamVerbose("local_bounding_box/frame_id", this->fixedFrame);
  this->publishDebugPclInside = this->getParamVerbose("debug/pcl/inside", false);
  this->publishDebugPclClip = this->getParamVerbose("debug/pcl/clip", false);
  this->publishDebugPclShadow = this->getParamVerbose("debug/pcl/shadow", false);
  this->publishDebugContainsMarker = this->getParamVerbose("debug/marker/contains", false);
  this->publishDebugShadowMarker = this->getParamVerbose("debug/marker/shadow", false);
  this->publishDebugBsphereMarker = this->getParamVerbose("debug/marker/bounding_sphere", false);
  this->publishDebugBboxMarker = this->getParamVerbose("debug/marker/bounding_box", false);

  const auto inflationPadding = this->getParamVerbose("body_model/inflation/padding", 0.0, "m");
  const auto inflationScale = this->getParamVerbose("body_model/inflation/scale", 1.0);
  this->defaultContainsInflation.padding = this->getParamVerbose("body_model/inflation/contains_test/padding", inflationPadding, "m");
  this->defaultContainsInflation.scale = this->getParamVerbose("body_model/inflation/contains_test/scale", inflationScale);
  this->defaultShadowInflation.padding = this->getParamVerbose("body_model/inflation/shadow_test/padding", inflationPadding, "m");
  this->defaultShadowInflation.scale = this->getParamVerbose("body_model/inflation/shadow_test/scale", inflationScale);
  this->defaultBsphereInflation.padding = this->getParamVerbose("body_model/inflation/bounding_sphere/padding", inflationPadding, "m");
  this->defaultBsphereInflation.scale = this->getParamVerbose("body_model/inflation/bounding_sphere/scale", inflationScale);
  this->defaultBboxInflation.padding = this->getParamVerbose("body_model/inflation/bounding_box/padding", inflationPadding, "m");
  this->defaultBboxInflation.scale = this->getParamVerbose("body_model/inflation/bounding_box/scale", inflationScale);

  // read per-link padding
  const auto perLinkInflationPadding = this->getParamVerboseMap("body_model/inflation/per_link/padding", std::map<std::string, double>(), "m");
  for (const auto& inflationPair : perLinkInflationPadding)
  {
    bool containsOnly;
    bool shadowOnly;
    bool bsphereOnly;
    bool bboxOnly;

    auto linkName = inflationPair.first;
    linkName = removeSuffix(linkName, CONTAINS_SUFFIX, &containsOnly);
    linkName = removeSuffix(linkName, SHADOW_SUFFIX, &shadowOnly);
    linkName = removeSuffix(linkName, BSPHERE_SUFFIX, &bsphereOnly);
    linkName = removeSuffix(linkName, BBOX_SUFFIX, &bboxOnly);

    if (!shadowOnly && !bsphereOnly && !bboxOnly)
      this->perLinkContainsInflation[linkName] =
          ScaleAndPadding(this->defaultContainsInflation.scale, inflationPair.second);
    if (!containsOnly && !bsphereOnly && !bboxOnly)
      this->perLinkShadowInflation[linkName] =
          ScaleAndPadding(this->defaultShadowInflation.scale, inflationPair.second);
    if (!containsOnly && !shadowOnly && !bboxOnly)
      this->perLinkBsphereInflation[linkName] =
          ScaleAndPadding(this->defaultBsphereInflation.scale, inflationPair.second);
    if (!containsOnly && !shadowOnly && !bsphereOnly)
      this->perLinkBboxInflation[linkName] =
          ScaleAndPadding(this->defaultBboxInflation.scale, inflationPair.second);
  }

  // read per-link scale
  const auto perLinkInflationScale = this->getParamVerboseMap("body_model/inflation/per_link/scale", std::map<std::string, double>());
  for (const auto& inflationPair : perLinkInflationScale)
  {
    bool containsOnly;
    bool shadowOnly;
    bool bsphereOnly;
    bool bboxOnly;

    auto linkName = inflationPair.first;
    linkName = removeSuffix(linkName, CONTAINS_SUFFIX, &containsOnly);
    linkName = removeSuffix(linkName, SHADOW_SUFFIX, &shadowOnly);
    linkName = removeSuffix(linkName, BSPHERE_SUFFIX, &bsphereOnly);
    linkName = removeSuffix(linkName, BBOX_SUFFIX, &bboxOnly);

    if (!shadowOnly && !bsphereOnly && !bboxOnly)
    {
      if (this->perLinkContainsInflation.find(linkName) == this->perLinkContainsInflation.end())
        this->perLinkContainsInflation[linkName] = ScaleAndPadding(inflationPair.second, this->defaultContainsInflation.padding);
      else
        this->perLinkContainsInflation[linkName].scale = inflationPair.second;
    }

    if (!containsOnly && !bsphereOnly && !bboxOnly)
    {
      if (this->perLinkShadowInflation.find(linkName) == this->perLinkShadowInflation.end())
        this->perLinkShadowInflation[linkName] = ScaleAndPadding(inflationPair.second, this->defaultShadowInflation.padding);
      else
        this->perLinkShadowInflation[linkName].scale = inflationPair.second;
    }

    if (!containsOnly && !shadowOnly && !bboxOnly)
    {
      if (this->perLinkBsphereInflation.find(linkName) == this->perLinkBsphereInflation.end())
        this->perLinkBsphereInflation[linkName] = ScaleAndPadding(inflationPair.second, this->defaultBsphereInflation.padding);
      else
        this->perLinkBsphereInflation[linkName].scale = inflationPair.second;
    }

    if (!containsOnly && !shadowOnly && !bsphereOnly)
    {
      if (this->perLinkBboxInflation.find(linkName) == this->perLinkBboxInflation.end())
        this->perLinkBboxInflation[linkName] = ScaleAndPadding(inflationPair.second, this->defaultBboxInflation.padding);
      else
        this->perLinkBboxInflation[linkName].scale = inflationPair.second;
    }
  }

  // can contain either whole link names, or scoped names of their collisions (i.e. "link::collision_1" or "link::my_collision")
  this->linksIgnoredInBoundingSphere = this->template getParamVerboseSet<string>("ignored_links/bounding_sphere");
  this->linksIgnoredInBoundingBox = this->template getParamVerboseSet<string>("ignored_links/bounding_box");
  this->linksIgnoredInContainsTest = this->template getParamVerboseSet<string>("ignored_links/contains_test");
  this->linksIgnoredInShadowTest = this->template getParamVerboseSet<string>("ignored_links/shadow_test", { "laser" });
  this->linksIgnoredEverywhere = this->template getParamVerboseSet<string>("ignored_links/everywhere");
  this->onlyLinks = this->template getParamVerboseSet<string>("only_links");

  this->robotDescriptionUpdatesFieldName = this->getParamVerbose("body_model/dynamic_robot_description/field_name", "robot_model");
  // subscribe for robot_description param changes
  this->robotDescriptionUpdatesListener = this->nodeHandle.subscribe(
    "dynamic_robot_model_server/parameter_updates", 10, &RobotBodyFilter::robotDescriptionUpdated, this);

  this->reloadRobotModelServiceServer = this->privateNodeHandle.advertiseService(
      "reload_model", &RobotBodyFilter::triggerModelReload, this);

  if (this->computeBoundingSphere) {
    this->boundingSpherePublisher = this->nodeHandle.template advertise<SphereStamped>("robot_bounding_sphere", 100);
  }

  if (this->computeBoundingBox) {
    this->boundingBoxPublisher = this->nodeHandle.template advertise<geometry_msgs::PolygonStamped>("robot_bounding_box", 100);
  }

  if (this->computeOrientedBoundingBox) {
    this->orientedBoundingBoxPublisher = this->nodeHandle.template advertise<OrientedBoundingBoxStamped>("robot_oriented_bounding_box", 100);
  }

  if (this->computeLocalBoundingBox) {
    this->localBoundingBoxPublisher = this->nodeHandle.template advertise<geometry_msgs::PolygonStamped>("robot_local_bounding_box", 100);
  }

  if (this->publishBoundingSphereMarker && this->computeBoundingSphere) {
    this->boundingSphereMarkerPublisher = this->nodeHandle.template advertise<visualization_msgs::Marker>("robot_bounding_sphere_marker", 100);
  }

  if (this->publishBoundingBoxMarker && this->computeBoundingBox) {
    this->boundingBoxMarkerPublisher = this->nodeHandle.template advertise<visualization_msgs::Marker>("robot_bounding_box_marker", 100);
  }

  if (this->publishOrientedBoundingBoxMarker && this->computeOrientedBoundingBox) {
    this->orientedBoundingBoxMarkerPublisher = this->nodeHandle.template advertise<visualization_msgs::Marker>("robot_oriented_bounding_box_marker", 100);
  }

  if (this->publishLocalBoundingBoxMarker && this->computeLocalBoundingBox) {
    this->localBoundingBoxMarkerPublisher = this->nodeHandle.template advertise<visualization_msgs::Marker>("robot_local_bounding_box_marker", 100);
  }

  if (this->publishNoBoundingBoxPointcloud)
  {
    this->scanPointCloudNoBoundingBoxPublisher = this->nodeHandle.template advertise<sensor_msgs::PointCloud2>("scan_point_cloud_no_bbox", 100);
  }

  if (this->publishNoOrientedBoundingBoxPointcloud)
  {
    this->scanPointCloudNoOrientedBoundingBoxPublisher = this->nodeHandle.template advertise<sensor_msgs::PointCloud2>("scan_point_cloud_no_oriented_bbox", 100);
  }

  if (this->publishNoLocalBoundingBoxPointcloud)
  {
    this->scanPointCloudNoLocalBoundingBoxPublisher = this->nodeHandle.template advertise<sensor_msgs::PointCloud2>("scan_point_cloud_no_local_bbox", 100);
  }

  if (this->publishNoBoundingSpherePointcloud)
  {
    this->scanPointCloudNoBoundingSpherePublisher = this->nodeHandle.template advertise<sensor_msgs::PointCloud2>("scan_point_cloud_no_bsphere", 100);
  }

  if (this->publishDebugPclInside)
  {
    this->debugPointCloudInsidePublisher = this->nodeHandle.template advertise<sensor_msgs::PointCloud2>("scan_point_cloud_inside", 100);
  }

  if (this->publishDebugPclClip)
  {
    this->debugPointCloudClipPublisher = this->nodeHandle.template advertise<sensor_msgs::PointCloud2>("scan_point_cloud_clip", 100);
  }

  if (this->publishDebugPclShadow)
  {
    this->debugPointCloudShadowPublisher = this->nodeHandle.template advertise<sensor_msgs::PointCloud2>("scan_point_cloud_shadow", 100);
  }

  if (this->publishDebugContainsMarker)
  {
    this->debugContainsMarkerPublisher = this->nodeHandle.template advertise<visualization_msgs::MarkerArray>("robot_model_for_contains_test", 100);
  }

  if (this->publishDebugShadowMarker)
  {
    this->debugShadowMarkerPublisher = this->nodeHandle.template advertise<visualization_msgs::MarkerArray>("robot_model_for_shadow_test", 100);
  }

  if (this->publishDebugBsphereMarker)
  {
    this->debugBsphereMarkerPublisher = this->nodeHandle.template advertise<visualization_msgs::MarkerArray>("robot_model_for_bounding_sphere", 100);
  }

  if (this->publishDebugBboxMarker)
  {
    this->debugBboxMarkerPublisher = this->nodeHandle.template advertise<visualization_msgs::MarkerArray>("robot_model_for_bounding_box", 100);
  }

  if (this->computeDebugBoundingBox) {
    this->boundingBoxDebugMarkerPublisher = this->nodeHandle.template advertise<visualization_msgs::MarkerArray>(
      "robot_bounding_box_debug", 100);
  }

  if (this->computeDebugOrientedBoundingBox) {
    this->orientedBoundingBoxDebugMarkerPublisher = this->nodeHandle.template advertise<visualization_msgs::MarkerArray>(
      "robot_oriented_bounding_box_debug", 100);
  }

  if (this->computeDebugLocalBoundingBox) {
    this->localBoundingBoxDebugMarkerPublisher = this->nodeHandle.template advertise<visualization_msgs::MarkerArray>(
      "robot_local_bounding_box_debug", 100);
  }

  if (this->computeDebugBoundingSphere) {
    this->boundingSphereDebugMarkerPublisher = this->nodeHandle.template advertise<visualization_msgs::MarkerArray>(
      "robot_bounding_sphere_debug", 100);
  }

  // initialize the 3D body masking tool
  auto getShapeTransformCallback = std::bind(&RobotBodyFilter::getShapeTransform, this, std::placeholders::_1, std::placeholders::_2);
  shapeMask = std::make_unique<RayCastingShapeMask>(getShapeTransformCallback,
      this->minDistance, this->maxDistance,
      doClipping, doContainsTest, doShadowTest, maxShadowDistance);

  // the other case happens when configure() is called again from update() (e.g. when a new bag file
  // started playing)
  if (this->tfFramesWatchdog == nullptr) {
    std::set<std::string> initialMonitoredFrames;
    if (!this->sensorFrame.empty())
    {
      initialMonitoredFrames.insert(this->sensorFrame);
    }

    this->tfFramesWatchdog = std::make_shared<TFFramesWatchdog>(this->filteringFrame,
        initialMonitoredFrames, this->tfBuffer,
        this->unreachableTransformTimeout, ros::Rate(ros::Duration(1.0)));
    this->tfFramesWatchdog->start();
  }

  { // initialize the robot body to be masked out

    string robotUrdf;
    while (!this->nodeHandle.getParam(this->robotDescriptionParam, robotUrdf) || robotUrdf.length() == 0) {
      if (this->failWithoutRobotDescription)
      {
        throw std::runtime_error(
            "RobotBodyFilter: " + this->robotDescriptionParam + " is empty or not set.");
      }
      if (!ros::ok())
        return false;

      ROS_ERROR("RobotBodyFilter: %s is empty or not set. Please, provide the robot model. Waiting 1s.",
                robotDescriptionParam.c_str());
      ros::Duration(1.0).sleep();
    }

    // happens when configure() is called again from update() (e.g. when a new bag file started
    // playing)
    if (!this->shapesToLinks.empty())
      this->clearRobotMask();
    this->addRobotMaskFromUrdf(robotUrdf);
  }

  ROS_INFO("RobotBodyFilter: Successfully configured.");
  ROS_INFO("Filtering data in frame %s", this->filteringFrame.c_str());
  ROS_INFO("RobotBodyFilter: Filtering into the following categories:");
  ROS_INFO("RobotBodyFilter: \tOUTSIDE");
  if (doClipping) ROS_INFO("RobotBodyFilter: \tCLIP");
  if (doContainsTest) ROS_INFO("RobotBodyFilter: \tINSIDE");
  if (doShadowTest) ROS_INFO("RobotBodyFilter: \tSHADOW");

  if (this->onlyLinks.empty()) {
    if (this->linksIgnoredEverywhere.empty()) {
      ROS_INFO("RobotBodyFilter: Filtering applied to all links.");
    } else {
      ROS_INFO("RobotBodyFilter: Filtering applied to all links except %s.", to_string(this->linksIgnoredEverywhere).c_str());
    }
  } else {
    if (this->linksIgnoredEverywhere.empty()) {
      ROS_INFO("RobotBodyFilter: Filtering applied to links %s.", to_string(this->onlyLinks).c_str());
    } else {
      ROS_INFO("RobotBodyFilter: Filtering applied to links %s with these links excluded: %s.", to_string(this->onlyLinks).c_str(), to_string(this->linksIgnoredEverywhere).c_str());
    }
  }

  this->timeConfigured = ros::Time::now();

  return true;
}

bool RobotBodyFilterLaserScan::configure() {
  this->pointByPointScan = this->getParamVerbose("sensor/point_by_point", true);

  bool success = RobotBodyFilter::configure();
  return success;
}

bool RobotBodyFilterPointCloud2::configure() {
  this->pointByPointScan = this->getParamVerbose("sensor/point_by_point", false);

  bool success = RobotBodyFilter::configure();
  if (!success)
    return false;

  this->outputFrame = this->getParamVerbose("frames/output", this->filteringFrame);

  const auto pointChannels = this->getParamVerbose("cloud/point_channels", std::vector<std::string>{"vp_"});
  const auto directionChannels = this->getParamVerbose("cloud/direction_channels", std::vector<std::string>{"normal_"});

  for (const auto& channel : pointChannels)
    this->channelsToTransform[channel] = CloudChannelType::POINT;
  for (const auto& channel : directionChannels)
    this->channelsToTransform[channel] = CloudChannelType::DIRECTION;

  stripLeadingSlash(this->outputFrame, true);

  return true;
}

template <typename T>
bool RobotBodyFilter<T>::computeMask(
    const sensor_msgs::PointCloud2 &projectedPointCloud,
    std::vector<RayCastingShapeMask::MaskValue> &pointMask,
    const std::string &sensorFrame) {

  // this->modelMutex has to be already locked!

  const clock_t stopwatchOverall = clock();
  const auto& scanTime = projectedPointCloud.header.stamp;

  // compute a mask of point indices for points from projectedPointCloud
  // that tells if they are inside or outside robot, or shadow points

  if (!this->pointByPointScan)
  {
    Eigen::Vector3d sensorPosition;
    try {
      const auto sensorTf = this->tfBuffer->lookupTransform(
          this->filteringFrame, sensorFrame, scanTime,
          remainingTime(scanTime, this->reachableTransformTimeout));
      tf2::fromMsg(sensorTf.transform.translation, sensorPosition);
    } catch (tf2::TransformException& e) {
      ROS_ERROR("RobotBodyFilter: Could not compute filtering mask due to this "
                "TF exception: %s", e.what());
      return false;
    }

    // update transforms cache, which is then used in body masking
    this->updateTransformCache(scanTime);

    // updates shapes according to tf cache (by calling getShapeTransform
    // for each shape) and masks contained points
    this->shapeMask->maskContainmentAndShadows(projectedPointCloud, pointMask, sensorPosition);
  } else {
    CloudConstIter x_it(projectedPointCloud, "x");
    CloudConstIter y_it(projectedPointCloud, "y");
    CloudConstIter z_it(projectedPointCloud, "z");
    CloudConstIter vp_x_it(projectedPointCloud, "vp_x");
    CloudConstIter vp_y_it(projectedPointCloud, "vp_y");
    CloudConstIter vp_z_it(projectedPointCloud, "vp_z");
    CloudConstIter stamps_it(projectedPointCloud, "stamps");

    pointMask.resize(num_points(projectedPointCloud));

    double scanDuration = 0.0;
    for (CloudConstIter stamps_end_it(projectedPointCloud, "stamps"); stamps_end_it != stamps_end_it.end(); ++stamps_end_it)
    {
      if ((*stamps_end_it) > static_cast<float>(scanDuration))
        scanDuration = static_cast<double>(*stamps_end_it);
    }
    const ros::Time afterScanTime(scanTime + ros::Duration().fromSec(scanDuration));

    size_t updateBodyPosesEvery;
    if (this->modelPoseUpdateInterval.sec == 0 && this->modelPoseUpdateInterval.nsec == 0) {
      updateBodyPosesEvery = 1;
    } else {
      updateBodyPosesEvery = static_cast<size_t>(ceil(this->modelPoseUpdateInterval.toSec() / scanDuration * num_points(projectedPointCloud)));
      // prevent division by zero
      if (updateBodyPosesEvery == 0)
        updateBodyPosesEvery = 1;
    }

    // prevent division by zero in ratio computation in case the pointcloud
    // isn't really taken point by point with different timestamps
    if (scanDuration == 0.0) {
      updateBodyPosesEvery = num_points(projectedPointCloud) + 1;
      ROS_WARN_ONCE("RobotBodyFilter: sensor/point_by_point is set to true but "
                    "all points in the cloud have the same timestamp. You should"
                    " change the parameter to false to gain performance.");
    }

    // update transforms cache, which is then used in body masking
    this->updateTransformCache(scanTime, afterScanTime);

    Eigen::Vector3f point;
    Eigen::Vector3d viewPoint;
    RayCastingShapeMask::MaskValue mask;

    this->cacheLookupBetweenScansRatio = 0.0;
    for (size_t i = 0; i < num_points(projectedPointCloud); ++i, ++x_it, ++y_it, ++z_it, ++vp_x_it, ++vp_y_it, ++vp_z_it, ++stamps_it)
    {
      point.x() = *x_it;
      point.y() = *y_it;
      point.z() = *z_it;

      // TODO viewpoint can be autocomputed from stamps
      viewPoint.x() = static_cast<double>(*vp_x_it);
      viewPoint.y() = static_cast<double>(*vp_y_it);
      viewPoint.z() = static_cast<double>(*vp_z_it);

      const auto updateBodyPoses = i % updateBodyPosesEvery == 0;

      if (updateBodyPoses && scanDuration > 0.0)
        this->cacheLookupBetweenScansRatio = static_cast<double>(*stamps_it) / scanDuration;

      // updates shapes according to tf cache (by calling getShapeTransform
      // for each shape) and masks contained points
      this->shapeMask->maskContainmentAndShadows(point, mask, viewPoint, updateBodyPoses);
      pointMask[i] = mask;
    }
  }

  ROS_DEBUG("RobotBodyFilter: Mask computed in %.5f secs.", double(clock()-stopwatchOverall) / CLOCKS_PER_SEC);

  this->publishDebugPointClouds(projectedPointCloud, pointMask);
  this->publishDebugMarkers(scanTime);
  this->computeAndPublishBoundingSphere(projectedPointCloud);
  this->computeAndPublishBoundingBox(projectedPointCloud);
  this->computeAndPublishOrientedBoundingBox(projectedPointCloud);
  this->computeAndPublishLocalBoundingBox(projectedPointCloud);

  ROS_DEBUG("RobotBodyFilter: Filtering run time is %.5f secs.", double(clock()-stopwatchOverall) / CLOCKS_PER_SEC);
  return true;
}

bool RobotBodyFilterLaserScan::update(const LaserScan &inputScan, LaserScan &filteredScan) {
  const auto& scanTime = inputScan.header.stamp;

  if (!this->configured_) {
    ROS_DEBUG("RobotBodyFilter: Ignore scan from time %u.%u - filter not yet initialized.",
              scanTime.sec, scanTime.nsec);
    return false;
  }

  if ((scanTime < timeConfigured) && ((scanTime + tfBufferLength) >= timeConfigured)) {
    ROS_DEBUG("RobotBodyFilter: Ignore scan from time %u.%u - filter not yet initialized.",
              scanTime.sec, scanTime.nsec);
    return false;
  }

  if ((scanTime < timeConfigured) && ((scanTime + tfBufferLength) < timeConfigured)) {
    ROS_WARN("RobotBodyFilter: Old TF data received. Clearing TF buffer and reconfiguring laser"
             "filter. If you're replaying a bag file, make sure rosparam /use_sim_time is set to "
             "true");
    this->configure();
    return false;
  }

  // tf2 doesn't like frames starting with slash
  const auto scanFrame = stripLeadingSlash(inputScan.header.frame_id, true);

  // Passing a sensorFrame does not make sense. Scan messages can't be transformed to other frames.
  if (!this->sensorFrame.empty() && this->sensorFrame != scanFrame) {
    ROS_WARN_ONCE("RobotBodyFilter: frames/sensor is set to frame_id '%s' different than "
                  "the frame_id of the incoming message '%s'. This is an invalid configuration: "
                  "the frames/sensor parameter will be neglected.",
                  this->sensorFrame.c_str(), scanFrame.c_str());
  }

  if (!this->tfFramesWatchdog->isReachable(scanFrame))
  {
    ROS_DEBUG("RobotBodyFilter: Throwing away scan since sensor frame is unreachable.");
    // if this->sensorFrame is empty, it can happen that we're not actually monitoring the sensor
    // frame, so start monitoring it
    if (!this->tfFramesWatchdog->isMonitored(scanFrame))
      this->tfFramesWatchdog->addMonitoredFrame(scanFrame);
    return false;
  }

  if (this->requireAllFramesReachable && !this->tfFramesWatchdog->areAllFramesReachable())
  {
    ROS_DEBUG("RobotBodyFilter: Throwing away scan since not all frames are reachable.");
    return false;
  }

  const clock_t stopwatchOverall = clock();

  // create the output copy of the input scan
  filteredScan = inputScan;
  filteredScan.header.frame_id = scanFrame;
  filteredScan.range_min = max(inputScan.range_min, (float) this->minDistance);
  if (this->maxDistance > 0.0)
    filteredScan.range_max = min(inputScan.range_max, (float) this->maxDistance);

  { // acquire the lock here, because we work with the tfBuffer all the time
    std::lock_guard<std::mutex> guard(*this->modelMutex);

    if (this->pointByPointScan)
    { // make sure we have all the tfs between sensor frame and fixedFrame during the time of scan acquisition
      const auto scanDuration = inputScan.ranges.size() * inputScan.time_increment;
      const auto afterScanTime = scanTime + ros::Duration().fromSec(scanDuration);

      string err;
      if (!this->tfBuffer->canTransform(this->fixedFrame, scanFrame, scanTime,
            remainingTime(scanTime, this->reachableTransformTimeout), &err) ||
            !this->tfBuffer->canTransform(this->fixedFrame, scanFrame, afterScanTime,
                remainingTime(afterScanTime, this->reachableTransformTimeout), &err)) {
        if (err.find("future") != string::npos) {
          const auto delay = ros::Time::now() - scanTime;
          ROS_ERROR_THROTTLE(3, "RobotBodyFilter: Cannot transform laser scan to "
            "fixed frame. The scan is too much delayed (%s s). TF error: %s",
            to_string(delay).c_str(), err.c_str());
        } else {
          ROS_ERROR_DELAYED_THROTTLE(3, "RobotBodyFilter: Cannot transform laser scan to "
            "fixed frame. Something's wrong with TFs: %s", err.c_str());
        }
        return false;
      }
    }

    // The point cloud will have fields x, y, z, intensity (float32) and index (int32)
    // and for point-by-point scans also timestamp and viewpoint
    sensor_msgs::PointCloud2 projectedPointCloud;
    { // project the scan measurements to a point cloud in the filteringFrame

      sensor_msgs::PointCloud2 tmpPointCloud;

      // the projected point cloud can omit some measurements if they are out of the defined scan's range;
      // for this case, the second channel ("index") contains indices of the point cloud's points into the scan
      auto channelOptions =
          laser_geometry::channel_option::Intensity |
          laser_geometry::channel_option::Index;

      if (this->pointByPointScan) {
        ROS_INFO_ONCE("RobotBodyFilter: Applying complex laser scan projection.");
        // perform the complex laser scan projection
        channelOptions |=
            laser_geometry::channel_option::Timestamp |
            laser_geometry::channel_option::Viewpoint;

        laserProjector.transformLaserScanToPointCloud(
            this->fixedFrame, inputScan, tmpPointCloud, *this->tfBuffer,
            -1, channelOptions);
      } else {
        ROS_INFO_ONCE("RobotBodyFilter: Applying simple laser scan projection.");
        // perform simple laser scan projection
        laserProjector.projectLaser(inputScan, tmpPointCloud, -1.0,
            channelOptions);
      }

      // convert to filtering frame
      if (tmpPointCloud.header.frame_id == this->filteringFrame) {
        projectedPointCloud = std::move(tmpPointCloud);
      } else {
        ROS_INFO_ONCE("RobotBodyFilter: Transforming scan from frame %s to %s",
            tmpPointCloud.header.frame_id.c_str(), this->filteringFrame.c_str());
        std::string err;
        if (!this->tfBuffer->canTransform(this->filteringFrame,
            tmpPointCloud.header.frame_id, scanTime,
            remainingTime(scanTime, this->reachableTransformTimeout), &err)) {
          ROS_ERROR_DELAYED_THROTTLE(3, "RobotBodyFilter: Cannot transform "
              "laser scan to filtering frame. Something's wrong with TFs: %s",
              err.c_str());
          return false;
        }

        transformWithChannels(tmpPointCloud, projectedPointCloud,
            *this->tfBuffer, this->filteringFrame, this->channelsToTransform);
      }
    }

    ROS_DEBUG("RobotBodyFilter: Scan transformation run time is %.5f secs.", double(clock()-stopwatchOverall) / CLOCKS_PER_SEC);

    vector<RayCastingShapeMask::MaskValue> pointMask;
    const auto success = this->computeMask(projectedPointCloud, pointMask, scanFrame);

    if (!success)
      return false;

    { // remove invalid points
      const float INVALID_POINT_VALUE = std::numeric_limits<float>::quiet_NaN();
      try {
        sensor_msgs::PointCloud2Iterator<int> indexIt(projectedPointCloud, "index");

        size_t indexInScan;
        for (const auto maskValue : pointMask) {
          switch (maskValue) {
            case RayCastingShapeMask::MaskValue::INSIDE:
            case RayCastingShapeMask::MaskValue::SHADOW:
            case RayCastingShapeMask::MaskValue::CLIP:
              indexInScan = static_cast<const size_t>(*indexIt);
              filteredScan.ranges[indexInScan] = INVALID_POINT_VALUE;
              break;
            case RayCastingShapeMask::MaskValue::OUTSIDE:
              break;
          }
          ++indexIt;
        }
      }
      catch (std::runtime_error&) {
        ROS_ERROR("RobotBodyFilter: projectedPointCloud doesn't have field called 'index',"
                  " but the algorithm relies on that.");
        return false;
      }
    }
  }

  return true;
}

bool RobotBodyFilterPointCloud2::update(const sensor_msgs::PointCloud2 &inputCloud,
                                        sensor_msgs::PointCloud2 &filteredCloud)
{
  const auto& scanTime = inputCloud.header.stamp;

  if (!this->configured_) {
    ROS_DEBUG("RobotBodyFilter: Ignore cloud from time %u.%u - filter not yet initialized.",
              scanTime.sec, scanTime.nsec);
    return false;
  }

  if ((scanTime < this->timeConfigured) && ((scanTime + this->tfBufferLength) >= this->timeConfigured)) {
    ROS_DEBUG("RobotBodyFilter: Ignore cloud from time %u.%u - filter not yet initialized.",
              scanTime.sec, scanTime.nsec);
    return false;
  }

  if ((scanTime < this->timeConfigured) && ((scanTime + this->tfBufferLength) < this->timeConfigured)) {
    ROS_WARN("RobotBodyFilter: Old TF data received. Clearing TF buffer and "
             "reconfiguring laser filter. If you're replaying a bag file, make "
             "sure rosparam /use_sim_time is set to true");
    this->configure();
    return false;
  }

  const auto inputCloudFrame = this->sensorFrame.empty() ?
      stripLeadingSlash(inputCloud.header.frame_id, true) : this->sensorFrame;

  if (!this->tfFramesWatchdog->isReachable(inputCloudFrame))
  {
    ROS_DEBUG("RobotBodyFilter: Throwing away scan since sensor frame is unreachable.");
    // if this->sensorFrame is empty, it can happen that we're not actually monitoring the cloud
    // frame, so start monitoring it
    if (!this->tfFramesWatchdog->isMonitored(inputCloudFrame))
      this->tfFramesWatchdog->addMonitoredFrame(inputCloudFrame);
    return false;
  }

  if (this->requireAllFramesReachable && !this->tfFramesWatchdog->areAllFramesReachable())
  {
    ROS_DEBUG("RobotBodyFilter: Throwing away scan since not all frames are reachable.");
    return false;
  }

  bool hasStampsField = false;
  bool hasVpXField = false, hasVpYField = false, hasVpZField = false;
  for (const auto& field : inputCloud.fields) {
    if (field.name == "stamps" && field.datatype == sensor_msgs::PointField::FLOAT32)
      hasStampsField = true;
    else if (field.name == "vp_x" && field.datatype == sensor_msgs::PointField::FLOAT32)
      hasVpXField = true;
    else if (field.name == "vp_y" && field.datatype == sensor_msgs::PointField::FLOAT32)
      hasVpYField = true;
    else if (field.name == "vp_z" && field.datatype == sensor_msgs::PointField::FLOAT32)
      hasVpZField = true;
  }

  // Verify the pointcloud and its fields

  if (this->pointByPointScan) {
    if (inputCloud.height != 1 && inputCloud.is_dense == 0) {
      ROS_WARN_ONCE("RobotBodyFilter: The pointcloud seems to be an organized "
                    "pointcloud, which usually means it was captured all at once."
                    " Consider setting 'point_by_point_scan' to false to get a "
                    "more efficient computation.");
    }
    if (!hasStampsField || !hasVpXField || !hasVpYField || !hasVpZField) {
      throw std::runtime_error("A point-by-point scan has to contain float32"
                               "fields 'stamps', 'vp_x', 'vp_y' and 'vp_z'.");
    }
  } else if (hasStampsField) {
    ROS_WARN_ONCE("RobotBodyFilter: The pointcloud has a 'stamps' field, "
                  "which indicates each point was probably captured at a "
                  "different time instant. Consider setting parameter "
                  "'point_by_point_scan' to true to get correct results.");
  } else if (inputCloud.height == 1 && inputCloud.is_dense == 1) {
    ROS_WARN_ONCE("RobotBodyFilter: The pointcloud is dense, which usually means"
                  " it was captured each point at a different time instant. "
                  "Consider setting 'point_by_point_scan' to true to get a more"
                  " accurate version.");
  }

  // Transform to filtering frame

  sensor_msgs::PointCloud2 transformedCloud;
  if (inputCloud.header.frame_id == this->filteringFrame) {
    transformedCloud = inputCloud;
  } else {
    ROS_INFO_ONCE("RobotBodyFilter: Transforming cloud from frame %s to %s",
                  inputCloud.header.frame_id.c_str(), this->filteringFrame.c_str());
    std::lock_guard<std::mutex> guard(*this->modelMutex);
    std::string err;
    if (!this->tfBuffer->canTransform(this->filteringFrame,
        inputCloud.header.frame_id, scanTime,
        remainingTime(scanTime, this->reachableTransformTimeout), &err)) {
      ROS_ERROR_DELAYED_THROTTLE(3, "RobotBodyFilter: Cannot transform "
          "point cloud to filtering frame. Something's wrong with TFs: %s",
          err.c_str());
      return false;
    }

    transformWithChannels(inputCloud, transformedCloud, *this->tfBuffer, this->filteringFrame,
                          this->channelsToTransform);
  }

  // Compute the mask and use it (transform message only if sensorFrame is specified)
  vector<RayCastingShapeMask::MaskValue> pointMask;
  {
    std::lock_guard<std::mutex> guard(*this->modelMutex);

    const auto success = this->computeMask(transformedCloud, pointMask, inputCloudFrame);
    if (!success)
      return false;
  }

  // Filter the cloud

  sensor_msgs::PointCloud2 tmpCloud;
  CREATE_FILTERED_CLOUD(transformedCloud, tmpCloud, this->keepCloudsOrganized,
                        (pointMask[i] == RayCastingShapeMask::MaskValue::OUTSIDE))

  // Transform to output frame

  if (tmpCloud.header.frame_id == this->outputFrame) {
    filteredCloud = std::move(tmpCloud);
  } else {
    ROS_INFO_ONCE("RobotBodyFilter: Transforming cloud from frame %s to %s",
        tmpCloud.header.frame_id.c_str(), this->outputFrame.c_str());
    std::lock_guard<std::mutex> guard(*this->modelMutex);
    std::string err;
    if (!this->tfBuffer->canTransform(this->outputFrame,
        tmpCloud.header.frame_id, scanTime,
        remainingTime(scanTime, this->reachableTransformTimeout), &err)) {
      ROS_ERROR_DELAYED_THROTTLE(3, "RobotBodyFilter: Cannot transform "
          "point cloud to output frame. Something's wrong with TFs: %s",
          err.c_str());
      return false;
    }

    transformWithChannels(tmpCloud, filteredCloud, *this->tfBuffer, this->outputFrame, this->channelsToTransform);
  }

  return true;
}

template<typename T>
bool RobotBodyFilter<T>::getShapeTransform(point_containment_filter::ShapeHandle shapeHandle, Eigen::Isometry3d &transform) const {
  // make sure you locked this->modelMutex

  // check if the given shapeHandle has been registered to a link during addRobotMaskFromUrdf call.
  if (this->shapesToLinks.find(shapeHandle) == this->shapesToLinks.end()) {
    ROS_ERROR_STREAM_THROTTLE(3, "RobotBodyFilter: Invalid shape handle: " << to_string(shapeHandle));
    return false;
  }

  const auto& collision = this->shapesToLinks.at(shapeHandle);

  if (this->transformCache.find(collision.cacheKey) == this->transformCache.end()) {
    // do not log the error because shape mask would do it for us
    return false;
  }

  if (!this->pointByPointScan) {
    transform = *this->transformCache.at(collision.cacheKey);
  } else {
    if (this->transformCacheAfterScan.find(collision.cacheKey) ==
        this->transformCacheAfterScan.end()) {
      // do not log the error because shape mask would do it for us
      return false;
    }

    const auto& tf1 = *this->transformCache.at(collision.cacheKey);
    const auto& tf2 = *this->transformCacheAfterScan.at(collision.cacheKey);
    const Eigen::Quaterniond quat1(tf1.rotation().matrix());
    const Eigen::Quaterniond quat2(tf1.rotation().matrix());
    const auto r = this->cacheLookupBetweenScansRatio;

    transform.translation() = tf1.translation() * (1 - r) + tf2.translation() * r;
    const Eigen::Quaterniond quat3 = quat1.slerp(r, quat2);
    transform.linear() = quat3.toRotationMatrix();
  }

  return true;
}

template<typename T>
void RobotBodyFilter<T>::updateTransformCache(const ros::Time &time, const ros::Time& afterScanTime) {
  // make sure you locked this->modelMutex

  // clear the cache so that maskContainment always uses only these tf data and not some older
  this->transformCache.clear();
  if (afterScanTime.sec != 0)
    this->transformCacheAfterScan.clear();

  // iterate over all links corresponding to some masking shape and update their cached transforms relative
  // to fixed_frame
  for (auto &shapeToLink : this->shapesToLinks) {

    const auto &collisionBody = shapeToLink.second;
    const auto &collision = collisionBody.collision;
    const auto &link = collisionBody.link;

    // here we assume the tf frames' names correspond to the link names
    const auto linkFrame = link->name;

    // the collision object may have a different origin than the visual, we need to account for that
    const auto &collisionOffsetTransform = urdfPose2EigenTransform(collision->origin);

    {
      auto linkTransformTfOptional = this->tfFramesWatchdog->lookupTransform(
          linkFrame, time, remainingTime(time, this->reachableTransformTimeout));

      if (!linkTransformTfOptional)  // has no value
        continue;

      const auto &linkTransformTf = linkTransformTfOptional.value();
      const auto &linkTransformEigen = tf2::transformToEigen(linkTransformTf);

      const auto &transform = linkTransformEigen * collisionOffsetTransform;

      this->transformCache[collisionBody.cacheKey] =
          std::allocate_shared<Eigen::Isometry3d>(Eigen::aligned_allocator<Eigen::Isometry3d>(), transform);
    }

    if (afterScanTime.sec != 0)
    {
      auto linkTransformTfOptional = this->tfFramesWatchdog->lookupTransform(
          linkFrame, afterScanTime, remainingTime(time, this->reachableTransformTimeout));

      if (!linkTransformTfOptional)  // has no value
        continue;

      const auto &linkTransformTf = linkTransformTfOptional.value();
      const auto &linkTransformEigen = tf2::transformToEigen(linkTransformTf);

      const auto &transform = linkTransformEigen * collisionOffsetTransform;

      this->transformCacheAfterScan[collisionBody.cacheKey] =
        std::allocate_shared<Eigen::Isometry3d>(Eigen::aligned_allocator<Eigen::Isometry3d>(), transform);
    }
  }
}

template<typename T>
void RobotBodyFilter<T>::addRobotMaskFromUrdf(const string& urdfModel) {
  if (urdfModel.empty()) {
    ROS_ERROR("RobotBodyFilter: Empty string passed as robot model to addRobotMaskFromUrdf. "
              "Robot body filtering is not going to work.");
    return;
  }

  // parse the URDF model
  urdf::Model parsedUrdfModel;
  bool urdfParseSucceeded = parsedUrdfModel.initString(urdfModel);
  if (!urdfParseSucceeded) {
    ROS_ERROR_STREAM("RobotBodyFilter: The URDF model given in parameter '" <<
        this->robotDescriptionParam << "' cannot be parsed. See "
        "urdf::Model::initString for debugging, or try running "
        "'gzsdf my_robot.urdf'");
    return;
  }

  {
    std::lock_guard<std::mutex> guard(*this->modelMutex);

    this->shapesIgnoredInBoundingSphere.clear();
    this->shapesIgnoredInBoundingBox.clear();
    std::unordered_set<MultiShapeHandle> ignoreInContainsTest;
    std::unordered_set<MultiShapeHandle> ignoreInShadowTest;

    // add all model's collision links as masking shapes
    for (const auto &links : parsedUrdfModel.links_) {

      const auto& link = links.second;

      // every link can have multiple collision elements
      size_t collisionIndex = 0;
      for (const auto& collision : link->collision_array) {
        if (collision->geometry == nullptr) {
          ROS_WARN("RobotBodyFilter: Collision element without geometry found in link %s of robot %s. "
                   "This collision element will not be filtered out.",
                   link->name.c_str(), parsedUrdfModel.getName().c_str());
          continue;  // collisionIndex is intentionally not increased
        }

        const auto NAME_LINK = link->name;
        const auto NAME_COLLISION_NAME = "*::" + collision->name;
        const auto NAME_LINK_COLLISION_NR = link->name + "::" + std::to_string(collisionIndex);
        const auto NAME_LINK_COLLISON_NAME = link->name + "::" + collision->name;

        const std::vector<std::string> collisionNames = {
            NAME_LINK,
            NAME_COLLISION_NAME,
            NAME_LINK_COLLISION_NR,
            NAME_LINK_COLLISON_NAME,
        };

        std::set<std::string> collisionNamesSet;
        std::set<std::string> collisionNamesContains;
        std::set<std::string> collisionNamesShadow;
        for (const auto& name : collisionNames)
        {
          collisionNamesSet.insert(name);
          collisionNamesContains.insert(name + CONTAINS_SUFFIX);
          collisionNamesShadow.insert(name + SHADOW_SUFFIX);
        }

        // if onlyLinks is nonempty, make sure this collision belongs to a specified link
        if (!this->onlyLinks.empty()) {
          if (isSetIntersectionEmpty(collisionNamesSet, this->onlyLinks)) {
            ++collisionIndex;
            continue;
          }
        }

        // if the link is ignored, go on
        if (!isSetIntersectionEmpty(collisionNamesSet, this->linksIgnoredEverywhere)) {
          ++collisionIndex;
          continue;
        }

        const auto collisionShape = constructShape(*collision->geometry);
        const auto shapeName = collision->name.empty() ? NAME_LINK_COLLISION_NR : NAME_LINK_COLLISON_NAME;

        // if the shape could not be constructed, ignore it (e.g. mesh was not found)
        if (collisionShape == nullptr) {
          ROS_WARN("Could not construct shape for collision %s, ignoring it.", shapeName.c_str());
          ++collisionIndex;
          continue;
        }

        // add the collision shape to shapeMask; the inflation parameters come into play here
        const auto containsTestInflation = this->getLinkInflationForContainsTest(collisionNames);
        const auto shadowTestInflation = this->getLinkInflationForShadowTest(collisionNames);
        const auto bsphereInflation = this->getLinkInflationForBoundingSphere(collisionNames);
        const auto bboxInflation = this->getLinkInflationForBoundingBox(collisionNames);
        const auto shapeHandle = this->shapeMask->addShape(collisionShape,
            containsTestInflation.scale, containsTestInflation.padding, shadowTestInflation.scale,
            shadowTestInflation.padding, bsphereInflation.scale, bsphereInflation.padding,
            bboxInflation.scale, bboxInflation.padding, false, shapeName);
        this->shapesToLinks[shapeHandle.contains] = this->shapesToLinks[shapeHandle.shadow] =
            this->shapesToLinks[shapeHandle.bsphere] = this->shapesToLinks[shapeHandle.bbox] =
                CollisionBodyWithLink(collision, link, collisionIndex, shapeHandle);

        if (!isSetIntersectionEmpty(collisionNamesSet, this->linksIgnoredInBoundingSphere)) {
          this->shapesIgnoredInBoundingSphere.insert(shapeHandle.bsphere);
        }

        if (!isSetIntersectionEmpty(collisionNamesSet, this->linksIgnoredInBoundingBox)) {
          this->shapesIgnoredInBoundingBox.insert(shapeHandle.bbox);
        }

        if (!isSetIntersectionEmpty(collisionNamesSet, this->linksIgnoredInContainsTest)) {
          ignoreInContainsTest.insert(shapeHandle);
        }

        if (!isSetIntersectionEmpty(collisionNamesSet, this->linksIgnoredInShadowTest)) {
          ignoreInShadowTest.insert(shapeHandle);
        }

        ++collisionIndex;
      }

      // no collision element found; only warn for links that are not ignored and have at least one visual
      if (collisionIndex == 0 && !link->visual_array.empty()) {
        if ((this->onlyLinks.empty() || (this->onlyLinks.find(link->name) != this->onlyLinks.end())) &&
             this->linksIgnoredEverywhere.find(link->name) == this->linksIgnoredEverywhere.end()) {
          ROS_WARN(
            "RobotBodyFilter: No collision element found for link %s of robot %s. This link will not be filtered out "
            "from laser scans.", link->name.c_str(), parsedUrdfModel.getName().c_str());
        }
      }
    }

    this->shapeMask->setIgnoreInContainsTest(ignoreInContainsTest);
    this->shapeMask->setIgnoreInShadowTest(ignoreInShadowTest);

    this->shapeMask->updateInternalShapeLists();

    std::set<std::string> monitoredFrames;
    for (const auto& shapeToLink : this->shapesToLinks)
      monitoredFrames.insert(shapeToLink.second.link->name);
    // Issue #6: Monitor sensor frame even if it is not a part of the model
    if (!this->sensorFrame.empty())
      monitoredFrames.insert(this->sensorFrame);

    this->tfFramesWatchdog->setMonitoredFrames(monitoredFrames);
  }
}

template<typename T>
void RobotBodyFilter<T>::clearRobotMask() {
  {
    std::lock_guard<std::mutex> guard(*this->modelMutex);

    std::unordered_set<MultiShapeHandle> removedMultiShapes;
    for (const auto& shapeToLink : this->shapesToLinks) {
      const auto& multiShape = shapeToLink.second.multiHandle;
      if (removedMultiShapes.find(multiShape) == removedMultiShapes.end())
      {
        this->shapeMask->removeShape(multiShape, false);
        removedMultiShapes.insert(multiShape);
      }
    }
    this->shapeMask->updateInternalShapeLists();

    this->shapesToLinks.clear();
    this->shapesIgnoredInBoundingSphere.clear();
    this->shapesIgnoredInBoundingBox.clear();
    this->transformCache.clear();
    this->transformCacheAfterScan.clear();
  }

  this->tfFramesWatchdog->clear();
}

template <typename T>
void RobotBodyFilter<T>::publishDebugMarkers(const ros::Time& scanTime) const {
  // assume this->modelMutex is locked

  if (this->publishDebugContainsMarker) {
    visualization_msgs::MarkerArray markerArray;
    std_msgs::ColorRGBA color;
    color.g = 1.0;
    color.a = 0.5;
    createBodyVisualizationMsg(this->shapeMask->getBodiesForContainsTest(), scanTime,
                               color, markerArray);
    this->debugContainsMarkerPublisher.publish(markerArray);
  }

  if (this->publishDebugShadowMarker) {
    visualization_msgs::MarkerArray markerArray;
    std_msgs::ColorRGBA color;
    color.b = 1.0;
    color.a = 0.5;
    createBodyVisualizationMsg(this->shapeMask->getBodiesForShadowTest(), scanTime,
                               color, markerArray);
    this->debugShadowMarkerPublisher.publish(markerArray);
  }

  if (this->publishDebugBsphereMarker) {
    visualization_msgs::MarkerArray markerArray;
    std_msgs::ColorRGBA color;
    color.g = 1.0;
    color.b = 1.0;
    color.a = 0.5;
    createBodyVisualizationMsg(this->shapeMask->getBodiesForBoundingSphere(), scanTime,
                               color, markerArray);
    this->debugBsphereMarkerPublisher.publish(markerArray);
  }

  if (this->publishDebugBboxMarker) {
    visualization_msgs::MarkerArray markerArray;
    std_msgs::ColorRGBA color;
    color.r = 1.0;
    color.b = 1.0;
    color.a = 0.5;
    createBodyVisualizationMsg(this->shapeMask->getBodiesForBoundingBox(), scanTime,
                               color, markerArray);
    this->debugBboxMarkerPublisher.publish(markerArray);
  }
}

template <typename T>
void RobotBodyFilter<T>::publishDebugPointClouds(
    const sensor_msgs::PointCloud2& projectedPointCloud,
    const std::vector<RayCastingShapeMask::MaskValue> &pointMask) const
{
  if (this->publishDebugPclInside)
  {
    sensor_msgs::PointCloud2 insideCloud;
    CREATE_FILTERED_CLOUD(projectedPointCloud, insideCloud, this->keepCloudsOrganized,
      (pointMask[i] == RayCastingShapeMask::MaskValue::INSIDE));
    this->debugPointCloudInsidePublisher.publish(insideCloud);
  }

  if (this->publishDebugPclClip)
  {
    sensor_msgs::PointCloud2 clipCloud;
    CREATE_FILTERED_CLOUD(projectedPointCloud, clipCloud, this->keepCloudsOrganized,
      (pointMask[i] == RayCastingShapeMask::MaskValue::CLIP));
    this->debugPointCloudClipPublisher.publish(clipCloud);
  }

  if (this->publishDebugPclShadow)
  {
    sensor_msgs::PointCloud2 shadowCloud;
    CREATE_FILTERED_CLOUD(projectedPointCloud, shadowCloud, this->keepCloudsOrganized,
      (pointMask[i] == RayCastingShapeMask::MaskValue::SHADOW));
    this->debugPointCloudShadowPublisher.publish(shadowCloud);
  }
}

template<typename T>
void RobotBodyFilter<T>::computeAndPublishBoundingSphere(
    const sensor_msgs::PointCloud2& projectedPointCloud) const
{
  if (!this->computeBoundingSphere && !this->computeDebugBoundingSphere)
    return;

  // assume this->modelMutex is locked

  // when computing bounding spheres for publication, we want to publish them to the time of the
  // scan, so we need to set cacheLookupBetweenScansRatio again to zero
  if (this->cacheLookupBetweenScansRatio != 0.0)
  {
    this->cacheLookupBetweenScansRatio = 0.0;
    this->shapeMask->updateBodyPoses();
  }

  const auto& scanTime = projectedPointCloud.header.stamp;
  std::vector<bodies::BoundingSphere> spheres;
  {
    visualization_msgs::MarkerArray boundingSphereDebugMsg;
    for (const auto &shapeHandleAndBody : this->shapeMask->getBodiesForBoundingSphere())
    {
      const auto &shapeHandle = shapeHandleAndBody.first;
      const auto &body = shapeHandleAndBody.second;

      if (this->shapesIgnoredInBoundingSphere.find(shapeHandle) != this->shapesIgnoredInBoundingSphere.end())
        continue;

      bodies::BoundingSphere sphere;
      body->computeBoundingSphere(sphere);

      spheres.push_back(sphere);

      if (this->computeDebugBoundingSphere)
      {
        visualization_msgs::Marker msg;
        msg.header.stamp = scanTime;
        msg.header.frame_id = this->filteringFrame;

        msg.scale.x = msg.scale.y = msg.scale.z = sphere.radius * 2;

        msg.pose.position.x = sphere.center[0];
        msg.pose.position.y = sphere.center[1];
        msg.pose.position.z = sphere.center[2];
        msg.pose.orientation.w = 1;

        msg.color.g = 1.0;
        msg.color.a = 0.5;
        msg.type = visualization_msgs::Marker::SPHERE;
        msg.action = visualization_msgs::Marker::ADD;
        msg.ns = "bsphere/" + this->shapesToLinks.at(shapeHandle).cacheKey;
        msg.frame_locked = static_cast<unsigned char>(true);

        boundingSphereDebugMsg.markers.push_back(msg);
      }
    }

    if (this->computeDebugBoundingSphere) {
      this->boundingSphereDebugMarkerPublisher.publish(boundingSphereDebugMsg);
    }
  }

  if (this->computeBoundingSphere)
  {
    bodies::BoundingSphere boundingSphere;
    bodies::mergeBoundingSpheres(spheres, boundingSphere);

    robot_body_filter::SphereStamped boundingSphereMsg;
    boundingSphereMsg.header.stamp = scanTime;
    boundingSphereMsg.header.frame_id = this->filteringFrame;
    boundingSphereMsg.sphere.radius =
        static_cast<float>(boundingSphere.radius);
    boundingSphereMsg.sphere.center = tf2::toMsg(boundingSphere.center);

    this->boundingSpherePublisher.publish(boundingSphereMsg);

    if (this->publishBoundingSphereMarker)
    {
      visualization_msgs::Marker msg;
      msg.header.stamp = scanTime;
      msg.header.frame_id = this->filteringFrame;

      msg.scale.x = msg.scale.y = msg.scale.z = boundingSphere.radius * 2;

      msg.pose.position.x = boundingSphere.center[0];
      msg.pose.position.y = boundingSphere.center[1];
      msg.pose.position.z = boundingSphere.center[2];
      msg.pose.orientation.w = 1;

      msg.color.g = 1.0;
      msg.color.a = 0.5;
      msg.type = visualization_msgs::Marker::SPHERE;
      msg.action = visualization_msgs::Marker::ADD;
      msg.ns = "bounding_sphere";
      msg.frame_locked = static_cast<unsigned char>(true);

      this->boundingSphereMarkerPublisher.publish(msg);
    }

    if (this->publishNoBoundingSpherePointcloud)
    {
      sensor_msgs::PointCloud2 noSphereCloud;
      CREATE_FILTERED_CLOUD(projectedPointCloud, noSphereCloud, this->keepCloudsOrganized,
        ((Eigen::Vector3d(*x_it, *y_it, *z_it)- boundingSphere.center).norm() > boundingSphere.radius));
      this->scanPointCloudNoBoundingSpherePublisher.publish(noSphereCloud);
    }
  }
}

template<typename T>
void RobotBodyFilter<T>::computeAndPublishBoundingBox(
    const sensor_msgs::PointCloud2& projectedPointCloud) const
{
  if (!this->computeBoundingBox && !this->computeDebugBoundingBox)
    return;

  // assume this->modelMutex is locked

  // when computing bounding boxes for publication, we want to publish them to the time of the
  // scan, so we need to set cacheLookupBetweenScansRatio again to zero
  if (this->cacheLookupBetweenScansRatio != 0.0)
  {
    this->cacheLookupBetweenScansRatio = 0.0;
    this->shapeMask->updateBodyPoses();
  }

  const auto& scanTime = projectedPointCloud.header.stamp;
  std::vector<bodies::AxisAlignedBoundingBox> boxes;

  {
    visualization_msgs::MarkerArray boundingBoxDebugMsg;
    for (const auto& shapeHandleAndBody : this->shapeMask->getBodiesForBoundingBox())
    {
      const auto& shapeHandle = shapeHandleAndBody.first;
      const auto& body = shapeHandleAndBody.second;

      if (this->shapesIgnoredInBoundingBox.find(shapeHandle) != this->shapesIgnoredInBoundingBox.end())
        continue;

      bodies::AxisAlignedBoundingBox box;
      body->computeBoundingBox(box);

      boxes.push_back(box);

      if (this->computeDebugBoundingBox) {
        visualization_msgs::Marker msg;
        msg.header.stamp = scanTime;
        msg.header.frame_id = this->filteringFrame;

        // it is aligned to fixed frame, not necessarily robot frame
        tf2::toMsg(box.sizes(), msg.scale);
        msg.pose.position = tf2::toMsg((Eigen::Vector3d)box.center());
        msg.pose.orientation.w = 1;

        msg.color.g = 1.0;
        msg.color.a = 0.5;
        msg.type = visualization_msgs::Marker::CUBE;
        msg.action = visualization_msgs::Marker::ADD;
        msg.ns = "bbox/" + this->shapesToLinks.at(shapeHandle).cacheKey;
        msg.frame_locked = static_cast<unsigned char>(true);

        boundingBoxDebugMsg.markers.push_back(msg);
      }
    }

    if (this->computeDebugBoundingBox) {
      this->boundingBoxDebugMarkerPublisher.publish(boundingBoxDebugMsg);
    }
  }

  if (this->computeBoundingBox)
  {
    bodies::AxisAlignedBoundingBox box;
    bodies::mergeBoundingBoxes(boxes, box);
    const auto boxFloat = box.cast<float>();

    geometry_msgs::PolygonStamped boundingBoxMsg;

    boundingBoxMsg.header.stamp = scanTime;
    boundingBoxMsg.header.frame_id = this->filteringFrame;

    boundingBoxMsg.polygon.points.resize(2);
    tf2::toMsg(box.min(), boundingBoxMsg.polygon.points[0]);
    tf2::toMsg(box.max(), boundingBoxMsg.polygon.points[1]);

    this->boundingBoxPublisher.publish(boundingBoxMsg);

    if (this->publishBoundingBoxMarker)
    {
      visualization_msgs::Marker msg;
      msg.header.stamp = scanTime;
      msg.header.frame_id = this->filteringFrame;

      // it is aligned to fixed frame and not necessarily to robot frame
      tf2::toMsg(box.sizes(), msg.scale);
      msg.pose.position = tf2::toMsg((Eigen::Vector3d)box.center());
      msg.pose.orientation.w = 1;

      msg.color.r = 1.0;
      msg.color.a = 0.5;
      msg.type = visualization_msgs::Marker::CUBE;
      msg.action = visualization_msgs::Marker::ADD;
      msg.ns = "bounding_box";
      msg.frame_locked = static_cast<unsigned char>(true);

      this->boundingBoxMarkerPublisher.publish(msg);
    }

    // compute and publish the scan_point_cloud with robot bounding box removed
    if (this->publishNoBoundingBoxPointcloud) {

      pcl::PCLPointCloud2::Ptr bboxCropInput(new pcl::PCLPointCloud2());
      pcl_conversions::toPCL(projectedPointCloud, *(bboxCropInput));

      robot_body_filter::CropBoxPointCloud2 cropBox;
      cropBox.setNegative(true);
      cropBox.setInputCloud(bboxCropInput);
      cropBox.setKeepOrganized(this->keepCloudsOrganized);

      cropBox.setMin(Eigen::Vector4f(boxFloat.min()[0], boxFloat.min()[1], boxFloat.min()[2], 0.0));
      cropBox.setMax(Eigen::Vector4f(boxFloat.max()[0], boxFloat.max()[1], boxFloat.max()[2], 0.0));

      pcl::PCLPointCloud2 pclOutput;
      cropBox.filter(pclOutput);

      sensor_msgs::PointCloud2Ptr boxFilteredCloud(new sensor_msgs::PointCloud2());
      pcl_conversions::moveFromPCL(pclOutput, *boxFilteredCloud);
      boxFilteredCloud->header.stamp = scanTime;  // PCL strips precision of timestamp

      this->scanPointCloudNoBoundingBoxPublisher.publish(boxFilteredCloud);
    }
  }
}

template<typename T>
void RobotBodyFilter<T>::computeAndPublishOrientedBoundingBox(
    const sensor_msgs::PointCloud2& projectedPointCloud) const
{
  if (!this->computeOrientedBoundingBox && !this->computeDebugOrientedBoundingBox)
    return;

  // assume this->modelMutex is locked

  // when computing bounding boxes for publication, we want to publish them to the time of the
  // scan, so we need to set cacheLookupBetweenScansRatio again to zero
  if (this->cacheLookupBetweenScansRatio != 0.0)
  {
    this->cacheLookupBetweenScansRatio = 0.0;
    this->shapeMask->updateBodyPoses();
  }

  const auto& scanTime = projectedPointCloud.header.stamp;
  std::vector<bodies::OrientedBoundingBox> boxes;

  {
    visualization_msgs::MarkerArray boundingBoxDebugMsg;
    for (const auto& shapeHandleAndBody : this->shapeMask->getBodiesForBoundingBox())
    {
      const auto& shapeHandle = shapeHandleAndBody.first;
      const auto& body = shapeHandleAndBody.second;

      if (this->shapesIgnoredInBoundingBox.find(shapeHandle) != this->shapesIgnoredInBoundingBox.end())
        continue;

      bodies::OrientedBoundingBox box;
      body->computeBoundingBox(box);

      boxes.push_back(box);

      if (this->computeDebugOrientedBoundingBox) {
        visualization_msgs::Marker msg;
        msg.header.stamp = scanTime;
        msg.header.frame_id = this->filteringFrame;

        tf2::toMsg(box.getExtents(), msg.scale);
        msg.pose.position = tf2::toMsg((Eigen::Vector3d)box.getPose().translation());
        msg.pose.orientation = tf2::toMsg(Eigen::Quaterniond(box.getPose().linear()));

        msg.color.g = 1.0;
        msg.color.a = 0.5;
        msg.type = visualization_msgs::Marker::CUBE;
        msg.action = visualization_msgs::Marker::ADD;
        msg.ns = "obbox/" + this->shapesToLinks.at(shapeHandle).cacheKey;
        msg.frame_locked = static_cast<unsigned char>(true);

        boundingBoxDebugMsg.markers.push_back(msg);
      }
    }

    if (this->computeDebugOrientedBoundingBox) {
      this->orientedBoundingBoxDebugMarkerPublisher.publish(boundingBoxDebugMsg);
    }
  }

  if (this->computeOrientedBoundingBox)
  {
    bodies::OrientedBoundingBox box(Eigen::Isometry3d::Identity(), Eigen::Vector3d::Zero());
    bodies::mergeBoundingBoxesApprox(boxes, box);

    robot_body_filter::OrientedBoundingBoxStamped boundingBoxMsg;

    boundingBoxMsg.header.stamp = scanTime;
    boundingBoxMsg.header.frame_id = this->filteringFrame;

    tf2::toMsg(box.getExtents(), boundingBoxMsg.obb.extents);
    tf2::toMsg(box.getPose().translation(), boundingBoxMsg.obb.pose.translation);
    boundingBoxMsg.obb.pose.rotation = tf2::toMsg(Eigen::Quaterniond(box.getPose().linear()));

    this->orientedBoundingBoxPublisher.publish(boundingBoxMsg);

    if (this->publishOrientedBoundingBoxMarker)
    {
      visualization_msgs::Marker msg;
      msg.header.stamp = scanTime;
      msg.header.frame_id = this->filteringFrame;

      tf2::toMsg(box.getExtents(), msg.scale);
      msg.pose.position = tf2::toMsg((Eigen::Vector3d)box.getPose().translation());
      msg.pose.orientation = tf2::toMsg(Eigen::Quaterniond(box.getPose().linear()));

      msg.color.r = 1.0;
      msg.color.a = 0.5;
      msg.type = visualization_msgs::Marker::CUBE;
      msg.action = visualization_msgs::Marker::ADD;
      msg.ns = "oriented_bounding_box";
      msg.frame_locked = static_cast<unsigned char>(true);

      this->orientedBoundingBoxMarkerPublisher.publish(msg);
    }

    // compute and publish the scan_point_cloud with robot bounding box removed
    if (this->publishNoOrientedBoundingBoxPointcloud) {

      pcl::PCLPointCloud2::Ptr bboxCropInput(new pcl::PCLPointCloud2());
      pcl_conversions::toPCL(projectedPointCloud, *(bboxCropInput));

      robot_body_filter::CropBoxPointCloud2 cropBox;
      cropBox.setNegative(true);
      cropBox.setInputCloud(bboxCropInput);
      cropBox.setKeepOrganized(this->keepCloudsOrganized);

      const auto e = box.getExtents();
      cropBox.setMin(Eigen::Vector4f(-e.x()/2, -e.y()/2, -e.z()/2, 0.0));
      cropBox.setMax(Eigen::Vector4f(e.x()/2, e.y()/2, e.z()/2, 0.0));
      cropBox.setTranslation(box.getPose().translation().cast<float>());
      cropBox.setRotation(box.getPose().linear().eulerAngles(0, 1, 2).cast<float>());

      pcl::PCLPointCloud2 pclOutput;
      cropBox.filter(pclOutput);

      sensor_msgs::PointCloud2Ptr boxFilteredCloud(new sensor_msgs::PointCloud2());
      pcl_conversions::moveFromPCL(pclOutput, *boxFilteredCloud);
      boxFilteredCloud->header.stamp = scanTime;  // PCL strips precision of timestamp

      this->scanPointCloudNoOrientedBoundingBoxPublisher.publish(boxFilteredCloud);
    }
  }
}

template<typename T>
void RobotBodyFilter<T>::computeAndPublishLocalBoundingBox(
    const sensor_msgs::PointCloud2& projectedPointCloud) const
{
  if (!this->computeLocalBoundingBox && !this->computeDebugLocalBoundingBox)
    return;

  // assume this->modelMutex is locked

  const auto& scanTime = projectedPointCloud.header.stamp;
  std::string err;
  try {
    if (!this->tfBuffer->canTransform(this->localBoundingBoxFrame,
                                      this->filteringFrame,
                                      scanTime,
                                      remainingTime(scanTime, this->reachableTransformTimeout),
                                      &err)) {
      ROS_ERROR_DELAYED_THROTTLE(3.0, "Cannot get transform %s->%s. Error is %s.",
                         this->filteringFrame.c_str(),
                         this->localBoundingBoxFrame.c_str(), err.c_str());
      return;
    }
  } catch (tf2::TransformException& e) {
    ROS_ERROR_DELAYED_THROTTLE(3.0, "Cannot get transform %s->%s. Error is %s.",
                       this->filteringFrame.c_str(),
                       this->localBoundingBoxFrame.c_str(), e.what());
    return;
  }

  const auto localTfMsg = this->tfBuffer->lookupTransform(this->localBoundingBoxFrame,
      this->filteringFrame, scanTime);
  const Eigen::Isometry3d localTf = tf2::transformToEigen(localTfMsg.transform);

  std::vector<bodies::AxisAlignedBoundingBox> boxes;

  {
    visualization_msgs::MarkerArray boundingBoxDebugMsg;
    for (const auto& shapeHandleAndBody : this->shapeMask->getBodiesForBoundingBox())
    {
      const auto& shapeHandle = shapeHandleAndBody.first;
      const auto& body = shapeHandleAndBody.second;

      if (this->shapesIgnoredInBoundingBox.find(shapeHandle) != this->shapesIgnoredInBoundingBox.end())
        continue;

      bodies::AxisAlignedBoundingBox box;
      bodies::computeBoundingBoxAt(body, box, localTf * body->getPose());

      boxes.push_back(box);

      if (this->computeDebugLocalBoundingBox) {
        visualization_msgs::Marker msg;
        msg.header.stamp = scanTime;
        msg.header.frame_id = this->localBoundingBoxFrame;

        tf2::toMsg(box.sizes(), msg.scale);
        msg.pose.position = tf2::toMsg((Eigen::Vector3d)box.center());
        msg.pose.orientation.w = 1;

        msg.color.g = 1.0;
        msg.color.a = 0.5;
        msg.type = visualization_msgs::Marker::CUBE;
        msg.action = visualization_msgs::Marker::ADD;
        msg.ns = "lbbox/" + this->shapesToLinks.at(shapeHandle).cacheKey;
        msg.frame_locked = static_cast<unsigned char>(true);

        boundingBoxDebugMsg.markers.push_back(msg);
      }
    }

    if (this->computeDebugLocalBoundingBox) {
      this->localBoundingBoxDebugMarkerPublisher.publish(boundingBoxDebugMsg);
    }
  }

  if (this->computeLocalBoundingBox)
  {
    bodies::AxisAlignedBoundingBox box;
    bodies::mergeBoundingBoxes(boxes, box);

    geometry_msgs::PolygonStamped boundingBoxMsg;

    boundingBoxMsg.header.stamp = scanTime;
    boundingBoxMsg.header.frame_id = this->localBoundingBoxFrame;

    boundingBoxMsg.polygon.points.resize(2);
    tf2::toMsg(box.min(), boundingBoxMsg.polygon.points[0]);
    tf2::toMsg(box.max(), boundingBoxMsg.polygon.points[1]);

    this->localBoundingBoxPublisher.publish(boundingBoxMsg);

    if (this->publishLocalBoundingBoxMarker)
    {
      visualization_msgs::Marker msg;
      msg.header.stamp = scanTime;
      msg.header.frame_id = this->localBoundingBoxFrame;

      tf2::toMsg(box.sizes(), msg.scale);
      msg.pose.position = tf2::toMsg((Eigen::Vector3d)box.center());
      msg.pose.orientation.w = 1;

      msg.color.r = 1.0;
      msg.color.a = 0.5;
      msg.type = visualization_msgs::Marker::CUBE;
      msg.action = visualization_msgs::Marker::ADD;
      msg.ns = "local_bounding_box";
      msg.frame_locked = static_cast<unsigned char>(true);

      this->localBoundingBoxMarkerPublisher.publish(msg);
    }

    // compute and publish the scan_point_cloud with robot bounding box removed
    if (this->publishNoLocalBoundingBoxPointcloud) {

      pcl::PCLPointCloud2::Ptr bboxCropInput(new pcl::PCLPointCloud2());
      pcl_conversions::toPCL(projectedPointCloud, *(bboxCropInput));

      robot_body_filter::CropBoxPointCloud2 cropBox;
      cropBox.setNegative(true);
      cropBox.setInputCloud(bboxCropInput);
      cropBox.setKeepOrganized(this->keepCloudsOrganized);

      cropBox.setMin(Eigen::Vector4f(box.min()[0], box.min()[1], box.min()[2], 0.0));
      cropBox.setMax(Eigen::Vector4f(box.max()[0], box.max()[1], box.max()[2], 0.0));
      const Eigen::Isometry3d localTfInv = localTf.inverse();
      cropBox.setTranslation(localTfInv.translation().cast<float>());
      cropBox.setRotation(localTfInv.linear().eulerAngles(0, 1, 2).cast<float>());

      pcl::PCLPointCloud2 pclOutput;
      cropBox.filter(pclOutput);

      sensor_msgs::PointCloud2Ptr boxFilteredCloud(new sensor_msgs::PointCloud2());
      pcl_conversions::moveFromPCL(pclOutput, *boxFilteredCloud);
      boxFilteredCloud->header.stamp = scanTime;  // PCL strips precision of timestamp

      this->scanPointCloudNoLocalBoundingBoxPublisher.publish(boxFilteredCloud);
    }
  }
}

template<typename T>
void RobotBodyFilter<T>::createBodyVisualizationMsg(
    const std::map<point_containment_filter::ShapeHandle, const bodies::Body*>& bodies,
    const ros::Time& stamp, const std_msgs::ColorRGBA& color,
    visualization_msgs::MarkerArray& markerArray) const
{
  // when computing the markers for publication, we want to publish them to the time of the
  // scan, so we need to set cacheLookupBetweenScansRatio again to zero
  if (this->cacheLookupBetweenScansRatio != 0.0)
  {
    this->cacheLookupBetweenScansRatio = 0.0;
    this->shapeMask->updateBodyPoses();
  }

  for (const auto &shapeHandleAndBody : bodies)
  {
    const auto &shapeHandle = shapeHandleAndBody.first;
    auto body = shapeHandleAndBody.second;

    visualization_msgs::Marker msg;
    bodies::constructMarkerFromBody(body, msg);

    msg.header.stamp = stamp;
    msg.header.frame_id = this->filteringFrame;

    msg.color = color;
    msg.action = visualization_msgs::Marker::ADD;
    msg.ns = this->shapesToLinks.at(shapeHandle).cacheKey;
    msg.frame_locked = static_cast<unsigned char>(true);

    markerArray.markers.push_back(msg);
  }
}

template<typename T>
void RobotBodyFilter<T>::robotDescriptionUpdated(dynamic_reconfigure::ConfigConstPtr newConfig) {
  auto robotDescriptionIdx = static_cast<size_t>(-1);
  for (size_t i = 0; i < newConfig->strs.size(); ++i) {
    if (newConfig->strs[i].name == this->robotDescriptionUpdatesFieldName) {
      robotDescriptionIdx = i;
      break;
    }
  }

  // robot_description parameter was not found, so we don't have to restart the filter
  if (robotDescriptionIdx == static_cast<size_t>(-1))
    return;

  auto urdf = newConfig->strs[robotDescriptionIdx].value;

  ROS_INFO("RobotBodyFilter: Reloading robot model because of dynamic_reconfigure update. Filter operation stopped.");

  this->tfFramesWatchdog->pause();
  this->configured_ = false;

  this->clearRobotMask();
  this->addRobotMaskFromUrdf(urdf);

  this->tfFramesWatchdog->unpause();
  this->timeConfigured = ros::Time::now();
  this->configured_ = true;

  ROS_INFO("RobotBodyFilter: Robot model reloaded, resuming filter operation.");
}

template<typename T>
bool RobotBodyFilter<T>::triggerModelReload(std_srvs::TriggerRequest &,
                                              std_srvs::TriggerResponse &)
{
  std::string urdf;
  auto success = this->nodeHandle.getParam(this->robotDescriptionParam, urdf);

  if (!success)
  {
    ROS_ERROR_STREAM("RobotBodyFilter: Parameter " << this->robotDescriptionParam
        << " doesn't exist.");
    return false;
  }

  ROS_INFO("RobotBodyFilter: Reloading robot model because of trigger. Filter operation stopped.");

  this->tfFramesWatchdog->pause();
  this->configured_ = false;

  this->clearRobotMask();
  this->addRobotMaskFromUrdf(urdf);

  this->tfFramesWatchdog->unpause();
  this->timeConfigured = ros::Time::now();
  this->configured_ = true;

  ROS_INFO("RobotBodyFilter: Robot model reloaded, resuming filter operation.");
  return true;
}

template<typename T>
RobotBodyFilter<T>::~RobotBodyFilter(){
  if (this->tfFramesWatchdog != nullptr)
    this->tfFramesWatchdog->stop();
}

template<typename T>
ScaleAndPadding RobotBodyFilter<T>::getLinkInflationForContainsTest(
    const string &linkName) const
{
  return this->getLinkInflationForContainsTest({linkName});
}

template<typename T>
ScaleAndPadding RobotBodyFilter<T>::getLinkInflationForContainsTest(
    const std::vector<std::string>& linkNames) const
{
  return this->getLinkInflation(linkNames, this->defaultContainsInflation,
      this->perLinkContainsInflation);
}

template<typename T>
ScaleAndPadding RobotBodyFilter<T>::getLinkInflationForShadowTest(
    const string &linkName) const
{
  return this->getLinkInflationForShadowTest({linkName});
}

template<typename T>
ScaleAndPadding RobotBodyFilter<T>::getLinkInflationForShadowTest(
    const std::vector<std::string>& linkNames) const
{
  return this->getLinkInflation(linkNames, this->defaultShadowInflation,
      this->perLinkShadowInflation);
}

template<typename T>
ScaleAndPadding RobotBodyFilter<T>::getLinkInflationForBoundingSphere(
    const string &linkName) const
{
  return this->getLinkInflationForBoundingSphere({linkName});
}

template<typename T>
ScaleAndPadding RobotBodyFilter<T>::getLinkInflationForBoundingSphere(
    const std::vector<std::string>& linkNames) const
{
  return this->getLinkInflation(linkNames, this->defaultBsphereInflation,
      this->perLinkBsphereInflation);
}

template<typename T>
ScaleAndPadding RobotBodyFilter<T>::getLinkInflationForBoundingBox(
    const string &linkName) const
{
  return this->getLinkInflationForBoundingBox({linkName});
}

template<typename T>
ScaleAndPadding RobotBodyFilter<T>::getLinkInflationForBoundingBox(
    const std::vector<std::string>& linkNames) const
{
  return this->getLinkInflation(linkNames, this->defaultBboxInflation,
      this->perLinkBboxInflation);
}

template<typename T>
ScaleAndPadding RobotBodyFilter<T>::getLinkInflation(
    const std::vector<std::string>& linkNames, const ScaleAndPadding& defaultInflation,
    const std::map<std::string, ScaleAndPadding>& perLinkInflation) const
{
  ScaleAndPadding result = defaultInflation;

  for (const auto& linkName : linkNames)
  {
    if (perLinkInflation.find(linkName) != perLinkInflation.end())
      result = perLinkInflation.at(linkName);
  }

  return result;
}

ScaleAndPadding::ScaleAndPadding(double scale, double padding)
    :scale(scale), padding(padding)
{
}

bool ScaleAndPadding::operator==(const ScaleAndPadding& other) const
{
  return this->scale == other.scale && this->padding == other.padding;
}

bool ScaleAndPadding::operator!=(const ScaleAndPadding& other) const
{
  return !(*this == other);
}

}

PLUGINLIB_EXPORT_CLASS(robot_body_filter::RobotBodyFilterLaserScan, filters::FilterBase<sensor_msgs::LaserScan>)
PLUGINLIB_EXPORT_CLASS(robot_body_filter::RobotBodyFilterPointCloud2, filters::FilterBase<sensor_msgs::PointCloud2>)
