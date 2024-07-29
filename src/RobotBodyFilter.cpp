#include <utility>

#include <functional>
#include <memory>

#include "robot_body_filter/RobotBodyFilter.h"

// #include "pluginlib/class_list_macros.h"

#include <geometric_shapes/bodies.h>
#include <geometric_shapes/body_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/shape_to_marker.h>

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <robot_body_filter/utils/bodies.h>
#include <robot_body_filter/utils/crop_box.h>
#include <robot_body_filter/utils/set_utils.hpp>
#include <robot_body_filter/utils/shapes.h>
#include <robot_body_filter/utils/string_utils.hpp>
#include <robot_body_filter/utils/tf2_eigen.h>
#include <robot_body_filter/utils/tf2_sensor_msgs.h>
#include <robot_body_filter/utils/time_utils.hpp>
#include <robot_body_filter/utils/urdf_eigen.hpp>

#include <rclcpp/time.hpp>

namespace robot_body_filter {

template <typename T>
RobotBodyFilter<T>::RobotBodyFilter(std::shared_ptr<rclcpp::Node> inputNode)
    : privateNodeHandle("robot_body_filter_private"),
      nodeHandle(inputNode),
      modelPoseUpdateInterval(0, 0),
      reachableTransformTimeout(0, 0),
      unreachableTransformTimeout(0, 0),
      tfBufferLength(0, 0) {
  this->modelMutex = std::make_shared<std::mutex>();
}

void RobotBodyFilterPointCloud2::DeclareParameters(){
  if (this->nodeHandle->has_parameter("sensor/point_by_point") == false){
    this->nodeHandle->declare_parameter("sensor/point_by_point", false);
    RobotBodyFilter::DeclareParameters(); 
  }
};

template <typename T>
void RobotBodyFilter<T>::DeclareParameters(){
// Declare ROS2 Parameters in node constructor
  auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};

  param_desc.description = "frames/fixed";
  this->nodeHandle->declare_parameter("fixedFrame", "base_link", param_desc);
  param_desc.description = "frames/sensor";
  this->nodeHandle->declare_parameter("sensorFrame", "", param_desc);
  param_desc.description = "frames/filtering";
  this->nodeHandle->declare_parameter("filteringFrame", rclcpp::ParameterType::PARAMETER_STRING, param_desc);
  param_desc.description = "m";
  param_desc.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
  this->nodeHandle->declare_parameter("minDistance", 0.0, param_desc);
  this->nodeHandle->declare_parameter("maxDistance", 0.0, param_desc);
  param_desc.description = "body_model/robot_description_param name";
  this->nodeHandle->declare_parameter("body_model/robot_description_param", "robot_description", param_desc);
  this->nodeHandle->declare_parameter("filter/keep_clouds_organized", true);
  param_desc.description = "s";
  this->nodeHandle->declare_parameter("filter/model_pose_update_interval", 0.0, param_desc);
  this->nodeHandle->declare_parameter("filter/do_clipping", true);
  this->nodeHandle->declare_parameter("filter/do_contains_test", true);
  this->nodeHandle->declare_parameter("filter/do_shadow_test", true);
  param_desc.description = "m";
  this->nodeHandle->declare_parameter("filter/max_shadow_distance", rclcpp::ParameterType::PARAMETER_DOUBLE, param_desc);
  param_desc.description = "s";
  this->nodeHandle->declare_parameter("transforms/timeout/reachable", 0.1, param_desc);
  this->nodeHandle->declare_parameter("transforms/timeout/unreachable", 0.2, param_desc);
  this->nodeHandle->declare_parameter("transforms/require_all_reachable", false);
  this->nodeHandle->declare_parameter("bounding_sphere/publish_cut_out_pointcloud", false);
  this->nodeHandle->declare_parameter("bounding_box/publish_cut_out_pointcloud", false);
  this->nodeHandle->declare_parameter("oriented_bounding_box/publish_cut_out_pointcloud", false);
  this->nodeHandle->declare_parameter("local_bounding_box/publish_cut_out_pointcloud", false);
  this->nodeHandle->declare_parameter("bounding_sphere/compute", false);
  this->nodeHandle->declare_parameter("bounding_box/compute", false);
  this->nodeHandle->declare_parameter("oriented_bounding_box/compute", false);
  this->nodeHandle->declare_parameter("local_bounding_box/compute", false);
  this->nodeHandle->declare_parameter("bounding_sphere/debug", false);
  this->nodeHandle->declare_parameter("bounding_box/debug", false);
  this->nodeHandle->declare_parameter("oriented_bounding_box/debug", false);
  this->nodeHandle->declare_parameter("local_bounding_box/debug", false);
  this->nodeHandle->declare_parameter("bounding_sphere/marker", false);
  this->nodeHandle->declare_parameter("bounding_box/marker", false);
  this->nodeHandle->declare_parameter("oriented_bounding_box/marker", false);
  this->nodeHandle->declare_parameter("local_bounding_box/marker", false);
  this->nodeHandle->declare_parameter("local_bounding_box/frame_id", rclcpp::ParameterType::PARAMETER_STRING);
  this->nodeHandle->declare_parameter("debug/pcl/inside", false);
  this->nodeHandle->declare_parameter("debug/pcl/clip", false);
  this->nodeHandle->declare_parameter("debug/pcl/shadow", false);
  this->nodeHandle->declare_parameter("debug/marker/contains", false);
  this->nodeHandle->declare_parameter("debug/marker/shadow", false);
  this->nodeHandle->declare_parameter("debug/marker/bounding_sphere", false);
  this->nodeHandle->declare_parameter("debug/marker/bounding_box", false);
  param_desc.description = "m";
  this->nodeHandle->declare_parameter("body_model/inflation/padding", 0.0, param_desc);
  this->nodeHandle->declare_parameter("body_model/inflation/scale", 1.1);
  // NOTE: Default changed from inflationPadding/inflationScale to 0.0/1.0
  // TODO: Set defaults in GetParameter
  this->nodeHandle->declare_parameter("body_model/inflation/contains_test/padding", rclcpp::ParameterType::PARAMETER_DOUBLE, param_desc);
  this->nodeHandle->declare_parameter("body_model/inflation/contains_test/scale", rclcpp::ParameterType::PARAMETER_DOUBLE);
  this->nodeHandle->declare_parameter("body_model/inflation/shadow_test/padding", rclcpp::ParameterType::PARAMETER_DOUBLE, param_desc);
  this->nodeHandle->declare_parameter("body_model/inflation/shadow_test/scale", rclcpp::ParameterType::PARAMETER_DOUBLE);
  this->nodeHandle->declare_parameter("body_model/inflation/bounding_sphere/padding", rclcpp::ParameterType::PARAMETER_DOUBLE, param_desc);
  this->nodeHandle->declare_parameter("body_model/inflation/bounding_sphere/scale", rclcpp::ParameterType::PARAMETER_DOUBLE);
  this->nodeHandle->declare_parameter("body_model/inflation/bounding_box/padding", rclcpp::ParameterType::PARAMETER_DOUBLE, param_desc);
  this->nodeHandle->declare_parameter("body_model/inflation/bounding_box/scale", rclcpp::ParameterType::PARAMETER_DOUBLE);

  // TODO: This initialization may be incorrect, not sure if I understand how
  // this works
  // https://docs.ros2.org/foxy/api/rclcpp/classrclcpp_1_1Node.html#ae5ab12777100f65bd09163814dbbf486
  // this might need to be initialized with the names of each link
  this->nodeHandle->declare_parameters("body_model/inflation/per_link/padding", std::map<std::string, double>());
  this->nodeHandle->declare_parameters("body_model/inflation/per_link/scale", std::map<std::string, double>());

  this->nodeHandle->declare_parameter("ignored_links/bounding_sphere", std::vector<std::string>{});
  this->nodeHandle->declare_parameter("ignored_links/bounding_box", std::vector<std::string>{});
  this->nodeHandle->declare_parameter("ignored_links/contains_test", std::vector<std::string>{});
  this->nodeHandle->declare_parameter("ignored_links/shadow_test", std::vector<std::string>{});
  this->nodeHandle->declare_parameter("ignored_links/everywhere", std::vector<std::string>{});
  this->nodeHandle->declare_parameter("only_links", std::vector<std::string>{});
  this->nodeHandle->declare_parameter("body_model/dynamic_robot_description/field_name", "robot_model");

  this->nodeHandle->declare_parameter("frames/output", "base_link");
  this->nodeHandle->declare_parameter("cloud/point_channels", std::vector<std::string>{"vp_"});
  this->nodeHandle->declare_parameter("cloud/direction_channels", std::vector<std::string>{"normal_"});
  this->nodeHandle->declare_parameter("transforms/buffer_length", 60.0);

  this->nodeHandle->declare_parameter("robot_description", std::string{});
}

template <typename T>
bool RobotBodyFilter<T>::configure() {
  double tempBufferLength = 0.0;
  this->nodeHandle->get_parameter("transforms/buffer_length", tempBufferLength);
  this->tfBufferLength = rclcpp::Duration::from_seconds(tempBufferLength);
  if (this->tfBuffer == nullptr) {
    tf2::Duration tf2_duration = tf2_ros::fromRclcpp(this->tfBufferLength);
    RCLCPP_INFO(this->nodeHandle->get_logger(), "Creating TF buffer with length %f", this->tfBufferLength.seconds());
    this->tfBuffer = std::make_shared<tf2_ros::Buffer>(this->nodeHandle->get_clock());
    this->tfListener = std::make_shared<tf2_ros::TransformListener>(*this->tfBuffer);
  } else {
    // clear the TF buffer (useful if calling configure() after receiving old TF
    // data)
    this->tfBuffer->clear();
  }

  this->nodeHandle->get_parameter("fixedFrame", this->fixedFrame);
  stripLeadingSlash(this->fixedFrame, true);
  this->nodeHandle->get_parameter("sensorFrame", this->sensorFrame);
  stripLeadingSlash(this->sensorFrame, true);
  this->nodeHandle->get_parameter_or("filteringFrame", this->filteringFrame, this->fixedFrame);
  stripLeadingSlash(this->sensorFrame, true);
  this->nodeHandle->get_parameter("minDistance", this->minDistance);
  this->nodeHandle->get_parameter("maxDistance", this->maxDistance);
  this->nodeHandle->get_parameter("body_model/robot_description_param", this->robotDescriptionParam);
  this->nodeHandle->get_parameter("filter/keep_clouds_organized", this->keepCloudsOrganized);
  double tempModelPoseUpdateInterval;
  this->nodeHandle->get_parameter("filter/model_pose_update_interval", tempModelPoseUpdateInterval);
  this->modelPoseUpdateInterval = rclcpp::Duration::from_seconds(tempModelPoseUpdateInterval);
  bool tempDoClipping;
  this->nodeHandle->get_parameter("filter/do_clipping", tempDoClipping);
  const bool doClipping = tempDoClipping;
  bool tempDoContainsTest;
  this->nodeHandle->get_parameter("filter/do_contains_test", tempDoContainsTest);
  const bool doContainsTest = tempDoContainsTest;
  bool tempDoShadowTest;
  this->nodeHandle->get_parameter("filter/do_shadow_test", tempDoShadowTest);
  const bool doShadowTest = tempDoShadowTest;
  double tempMaxShadowDistance;
  this->nodeHandle->get_parameter_or("filter/max_shadow_distance", tempMaxShadowDistance, this->maxDistance);
  const double maxShadowDistance = tempMaxShadowDistance;
  double tempReachableTransformTimeout;
  this->nodeHandle->get_parameter("transforms/timeout/reachable", tempReachableTransformTimeout);
  this->reachableTransformTimeout = rclcpp::Duration::from_seconds(tempReachableTransformTimeout);
  double tempUnreachableTransformTimeout;
  this->nodeHandle->get_parameter("transforms/timeout/unreachable", tempUnreachableTransformTimeout);
  this->unreachableTransformTimeout = rclcpp::Duration::from_seconds(tempUnreachableTransformTimeout);
  this->nodeHandle->get_parameter("transforms/require_all_reachable", this->requireAllFramesReachable);
  this->nodeHandle->get_parameter("bounding_sphere/publish_cut_out_pointcloud", this->publishNoBoundingSpherePointcloud);
  this->nodeHandle->get_parameter("bounding_box/publish_cut_out_pointcloud", this->publishNoBoundingBoxPointcloud);
  this->nodeHandle->get_parameter("oriented_bounding_box/publish_cut_out_pointcloud",
                                 this->publishNoOrientedBoundingBoxPointcloud);
  this->nodeHandle->get_parameter("local_bounding_box/publish_cut_out_pointcloud",
                                 this->publishNoLocalBoundingBoxPointcloud);
  this->nodeHandle->get_parameter("bounding_sphere/compute", this->computeBoundingSphere);
  this->computeBoundingSphere = this->computeBoundingSphere || this->publishNoBoundingSpherePointcloud;
  this->nodeHandle->get_parameter("bounding_box/compute", this->computeBoundingBox);
  this->computeBoundingBox = this->computeBoundingBox || this->publishNoBoundingBoxPointcloud;
  this->nodeHandle->get_parameter("oriented_bounding_box/compute", this->computeOrientedBoundingBox);
  this->computeOrientedBoundingBox = this->computeOrientedBoundingBox || this->publishNoOrientedBoundingBoxPointcloud;
  this->nodeHandle->get_parameter("local_bounding_box/compute", this->computeLocalBoundingBox);
  this->computeLocalBoundingBox = this->computeLocalBoundingBox || this->publishNoLocalBoundingBoxPointcloud;
  this->nodeHandle->get_parameter("bounding_sphere/debug", this->computeDebugBoundingSphere);
  this->nodeHandle->get_parameter("bounding_box/debug", this->computeDebugBoundingBox);
  this->nodeHandle->get_parameter("oriented_bounding_box/debug", this->computeDebugOrientedBoundingBox);
  this->nodeHandle->get_parameter("local_bounding_box/debug", this->computeDebugLocalBoundingBox);
  this->nodeHandle->get_parameter("bounding_sphere/marker", this->publishBoundingSphereMarker);
  this->nodeHandle->get_parameter("bounding_box/marker", this->publishBoundingBoxMarker);
  this->nodeHandle->get_parameter("oriented_bounding_box/marker", this->publishOrientedBoundingBoxMarker);
  this->nodeHandle->get_parameter("local_bounding_box/marker", this->publishLocalBoundingBoxMarker);
  this->nodeHandle->get_parameter_or("local_bounding_box/frame_id", this->localBoundingBoxFrame, this->fixedFrame);
  this->nodeHandle->get_parameter("debug/pcl/inside", this->publishDebugPclInside);
  this->nodeHandle->get_parameter("debug/pcl/clip", this->publishDebugPclClip);
  this->nodeHandle->get_parameter("debug/pcl/shadow", this->publishDebugPclShadow);
  this->nodeHandle->get_parameter("debug/marker/contains", this->publishDebugContainsMarker);
  this->nodeHandle->get_parameter("debug/marker/shadow", this->publishDebugShadowMarker);
  this->nodeHandle->get_parameter("debug/marker/bounding_sphere", this->publishDebugBsphereMarker);
  this->nodeHandle->get_parameter("debug/marker/bounding_box", this->publishDebugBboxMarker);
  double tempInflationPadding;
  this->nodeHandle->get_parameter("body_model/inflation/padding", tempInflationPadding);
  const double inflationPadding = tempInflationPadding;
  double tempInflationScale;
  this->nodeHandle->get_parameter("body_model/inflation/scale", tempInflationScale);
  const double inflationScale = tempInflationScale;
  this->nodeHandle->get_parameter_or("body_model/inflation/contains_test/padding", this->defaultContainsInflation.padding, inflationPadding);
  this->nodeHandle->get_parameter_or("body_model/inflation/contains_test/scale", this->defaultContainsInflation.scale, inflationScale);
  this->nodeHandle->get_parameter_or("body_model/inflation/shadow_test/padding", this->defaultShadowInflation.padding, inflationPadding);
  this->nodeHandle->get_parameter_or("body_model/inflation/shadow_test/scale", this->defaultShadowInflation.scale, inflationScale);
  this->nodeHandle->get_parameter_or("body_model/inflation/bounding_sphere/padding", this->defaultBsphereInflation.padding, inflationPadding);
  this->nodeHandle->get_parameter_or("body_model/inflation/bounding_sphere/scale", this->defaultBsphereInflation.scale, inflationScale);
  this->nodeHandle->get_parameter_or("body_model/inflation/bounding_box/padding", this->defaultBboxInflation.padding, inflationPadding);
  this->nodeHandle->get_parameter_or("body_model/inflation/bounding_box/scale", this->defaultBboxInflation.scale, inflationScale);

  // read per-link padding
  std::map<std::string, double> perLinkInflationPadding;
  this->nodeHandle->get_parameters("body_model/inflation/per_link/padding", perLinkInflationPadding);

  for (const auto& inflationPair : perLinkInflationPadding) {
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
      this->perLinkBboxInflation[linkName] = ScaleAndPadding(this->defaultBboxInflation.scale, inflationPair.second);
  }

  // read per-link scale
  std::map<std::string, double> perLinkInflationScale;
  this->nodeHandle->get_parameters("body_model/inflation/per_link/scale", perLinkInflationScale);

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
        this->perLinkBboxInflation[linkName] =
            ScaleAndPadding(inflationPair.second, this->defaultBboxInflation.padding);
      else
        this->perLinkBboxInflation[linkName].scale = inflationPair.second;
    }
  }

  // can contain either whole link names, or scoped names of their
  // collisions (i.e. "link::collision_1" or "link::my_collision")
  // Note: ROS2 does not by default have a std::set parameter, this was the
  // workaround
  std::vector<std::string> tempLinksIgnoredInBoundingSphereVector;
  this->nodeHandle->get_parameter("ignored_links/bounding_sphere", tempLinksIgnoredInBoundingSphereVector);
  this->linksIgnoredInBoundingSphere.insert(tempLinksIgnoredInBoundingSphereVector.begin(),
                                            tempLinksIgnoredInBoundingSphereVector.end());

  std::vector<std::string> tempLinksIgnoredInBoundingBoxVector;
  this->nodeHandle->get_parameter("ignored_links/bounding_box", tempLinksIgnoredInBoundingBoxVector);
  this->linksIgnoredInBoundingBox.insert(tempLinksIgnoredInBoundingBoxVector.begin(),
                                         tempLinksIgnoredInBoundingBoxVector.end());

  std::vector<std::string> tempLinksIgnoredInContainsTest;
  this->nodeHandle->get_parameter("ignored_links/contains_test", tempLinksIgnoredInContainsTest);
  this->linksIgnoredInContainsTest.insert(tempLinksIgnoredInContainsTest.begin(), tempLinksIgnoredInContainsTest.end());

  std::vector<std::string> tempLinksIgnoredInShadowTest;
  this->nodeHandle->get_parameter("ignored_links/shadow_test", tempLinksIgnoredInShadowTest);
  this->linksIgnoredInShadowTest.insert(tempLinksIgnoredInShadowTest.begin(), tempLinksIgnoredInShadowTest.end());

  std::vector<std::string> tempLinksIgnoredEverywhere;
  this->nodeHandle->get_parameter("ignored_links/everywhere", tempLinksIgnoredEverywhere);
  this->linksIgnoredEverywhere.insert(tempLinksIgnoredEverywhere.begin(), tempLinksIgnoredEverywhere.end());

  std::vector<std::string> tempOnlyLinks;
  this->nodeHandle->get_parameter("only_links", tempOnlyLinks);
  this->onlyLinks.insert(tempOnlyLinks.begin(), tempOnlyLinks.end());

  this->nodeHandle->get_parameter("body_model/dynamic_robot_description/field_name",
                                 this->robotDescriptionUpdatesFieldName);

  this->nodeHandle->get_parameter("sensor/point_by_point", this->pointByPointScan);

  //TODO: Reload service
  // this->reloadRobotModelServiceServer = this->nodeHandle->template create_service<std_srvs::srv::Trigger>(
  //     "reload_model", std::bind(&RobotBodyFilter::triggerModelReload, this, std::placeholders::_1, std::placeholders::_2));

  if (this->computeBoundingSphere) {
    this->boundingSpherePublisher =
        nodeHandle->create_publisher<shape_msgs::msg::SolidPrimitive>("robot_bounding_sphere", rclcpp::QoS(100));
  }

  if (this->computeBoundingBox) {
    this->boundingBoxPublisher = nodeHandle->create_publisher<geometry_msgs::msg::PolygonStamped>("robot_bounding_box", rclcpp::QoS(100));
  }

  if (this->computeOrientedBoundingBox) {
    this->orientedBoundingBoxPublisher = nodeHandle->create_publisher<shape_msgs::msg::SolidPrimitive>("robot_oriented_bounding_box", rclcpp::QoS(100));
  }

  if (this->computeLocalBoundingBox) {
    this->localBoundingBoxPublisher = nodeHandle->create_publisher<geometry_msgs::msg::PolygonStamped>("robot_local_bounding_box", rclcpp::QoS(100));
  }

  if (this->publishBoundingSphereMarker && this->computeBoundingSphere) {
    this->boundingSphereMarkerPublisher = this->nodeHandle->template
    create_publisher<visualization_msgs::msg::Marker>("robot_bounding_sphere_marker",
    100);
  }

  if (this->publishBoundingBoxMarker && this->computeBoundingBox) {
    this->boundingBoxMarkerPublisher = this->nodeHandle->template
    create_publisher<visualization_msgs::msg::Marker>("robot_bounding_box_marker",
    100);
  }

  if (this->publishOrientedBoundingBoxMarker && this->computeOrientedBoundingBox) {
    this->orientedBoundingBoxMarkerPublisher = this->nodeHandle->template
    create_publisher<visualization_msgs::msg::Marker>("robot_oriented_bounding_box_marker",
    100);
  }

  if (this->publishLocalBoundingBoxMarker && this->computeLocalBoundingBox) {
    this->localBoundingBoxMarkerPublisher = this->nodeHandle->template
    create_publisher<visualization_msgs::msg::Marker>("robot_local_bounding_box_marker",
    100);
  }

  if (this->publishNoBoundingBoxPointcloud) {
    this->scanPointCloudNoBoundingBoxPublisher = this->nodeHandle->template
    create_publisher<sensor_msgs::msg::PointCloud2>("scan_point_cloud_no_bbox",
    100);
  }

  if (this->publishNoOrientedBoundingBoxPointcloud) {
    this->scanPointCloudNoOrientedBoundingBoxPublisher = this->nodeHandle->template
    create_publisher<sensor_msgs::msg::PointCloud2>("scan_point_cloud_no_oriented_bbox",
    100);
  }

  if (this->publishNoLocalBoundingBoxPointcloud) {
    this->scanPointCloudNoLocalBoundingBoxPublisher = this->nodeHandle->template
    create_publisher<sensor_msgs::msg::PointCloud2>("scan_point_cloud_no_local_bbox",
    100);
  }

  if (this->publishNoBoundingSpherePointcloud) {
    this->scanPointCloudNoBoundingSpherePublisher = this->nodeHandle->template
    create_publisher<sensor_msgs::msg::PointCloud2>("scan_point_cloud_no_bsphere",
    100);
  }

  if (this->publishDebugPclInside) {
    this->debugPointCloudInsidePublisher = this->nodeHandle->template
    create_publisher<sensor_msgs::msg::PointCloud2>("scan_point_cloud_inside",
    100);
  }

  if (this->publishDebugPclClip) {
    this->debugPointCloudClipPublisher = this->nodeHandle->template
    create_publisher<sensor_msgs::msg::PointCloud2>("scan_point_cloud_clip",
    100);
  }

  if (this->publishDebugPclShadow) {
    this->debugPointCloudShadowPublisher = this->nodeHandle->template
    create_publisher<sensor_msgs::msg::PointCloud2>("scan_point_cloud_shadow",
    100);
  }

  if (this->publishDebugContainsMarker) {
    this->debugContainsMarkerPublisher = this->nodeHandle->template
    create_publisher<visualization_msgs::msg::MarkerArray>("robot_model_for_contains_test",
    100);
  }

  if (this->publishDebugShadowMarker) {
    this->debugShadowMarkerPublisher = this->nodeHandle->template
    create_publisher<visualization_msgs::msg::MarkerArray>("robot_model_for_shadow_test",
    100);
  }

  if (this->publishDebugBsphereMarker) {
    this->debugBsphereMarkerPublisher = this->nodeHandle->template
    create_publisher<visualization_msgs::msg::MarkerArray>("robot_model_for_bounding_sphere",
    100);
  }

  if (this->publishDebugBboxMarker) {
    this->debugBboxMarkerPublisher = this->nodeHandle->template
    create_publisher<visualization_msgs::msg::MarkerArray>("robot_model_for_bounding_box",
    100);
  }

  if (this->computeDebugBoundingBox) {
    this->boundingBoxDebugMarkerPublisher = this->nodeHandle->template
    create_publisher<visualization_msgs::msg::MarkerArray>("robot_bounding_box_debug",
    100);
  }

  if (this->computeDebugOrientedBoundingBox) {
    this->orientedBoundingBoxDebugMarkerPublisher = this->nodeHandle->template
    create_publisher<visualization_msgs::msg::MarkerArray>("robot_oriented_bounding_box_debug",
    100);
  }

  if (this->computeDebugLocalBoundingBox) {
    this->localBoundingBoxDebugMarkerPublisher = this->nodeHandle->template
    create_publisher<visualization_msgs::msg::MarkerArray>("robot_local_bounding_box_debug",
    100);
  }

  if (this->computeDebugBoundingSphere) {
    this->boundingSphereDebugMarkerPublisher = this->nodeHandle->template
    create_publisher<visualization_msgs::msg::MarkerArray>("robot_bounding_sphere_debug",
    100);
  }

  // initialize the 3D body masking tool
  auto getShapeTransformCallback =
  std::bind(&RobotBodyFilter::getShapeTransform, this,
  std::placeholders::_1, std::placeholders::_2); shapeMask =
  std::make_unique<RayCastingShapeMask>(getShapeTransformCallback,
      this->minDistance, this->maxDistance,
      doClipping, doContainsTest, doShadowTest, maxShadowDistance);

  // the other case happens when configure() is called again from
  // update() (e.g. when a new bag file
  // started playing)
  if (this->tfFramesWatchdog == nullptr) {
    std::set<std::string> initialMonitoredFrames;
    if (!this->sensorFrame.empty())
    {
      initialMonitoredFrames.insert(this->sensorFrame);
    }
    auto loop_rate = std::make_shared<rclcpp::Rate>(rclcpp::Duration::from_seconds(1.0).nanoseconds());
    RCLCPP_DEBUG(this->nodeHandle->get_logger(), "Creating TF frames watchdog");
    RCLCPP_DEBUG(this->nodeHandle->get_logger(), "Filtering data in frame %s", this->filteringFrame.c_str()); 
    this->tfFramesWatchdog =
        std::make_shared<TFFramesWatchdog>(this->nodeHandle, this->filteringFrame, initialMonitoredFrames,
                                           this->tfBuffer, this->unreachableTransformTimeout, loop_rate);
    this->tfFramesWatchdog->start();
  }

  AsyncCallbackGroup = nodeHandle->create_callback_group(rclcpp::CallbackGroupType::Reentrant, true);
  rclcpp::QoS qos(rclcpp::KeepLast(10));
  const rmw_qos_profile_t &qos_profile = qos.get_rmw_qos_profile();
  robotDescriptionUpdatesListener = std::make_shared<rclcpp::AsyncParametersClient>(
      this->nodeHandle, "/robot_state_publisher", qos_profile, AsyncCallbackGroup);
  auto parameters_future = robotDescriptionUpdatesListener->get_parameters(
      {"robot_description"}, [this](std::shared_future<std::vector<rclcpp::Parameter>> future) {
        std_msgs::msg::String::SharedPtr msg = std::make_shared<std_msgs::msg::String>();
        msg->data = future.get()[0].as_string();
        RCLCPP_INFO(this->nodeHandle->get_logger(), "New URDF: %s", msg->data.c_str());
        robotDescriptionUpdated(msg);
      });

  // {  // initialize the robot body to be masked out
  //   auto sleeper = rclcpp::Rate(rclcpp::Duration::from_seconds(1.0).nanoseconds());
  //   std::string robotUrdf;
  //   while (!this->nodeHandle->get_parameter(this->robotDescriptionParam.c_str(), robotUrdf) || robotUrdf.length() == 0) {
  //     if (this->failWithoutRobotDescription) {
  //       throw std::runtime_error("RobotBodyFilter: " + this->robotDescriptionParam + " is empty or not set.");
  //     }
  //     if (!rclcpp::ok()) return false;

  //     RCLCPP_ERROR(this->nodeHandle->get_logger(),
  //                  "RobotBodyFilter: %s is empty or not set. Please, provide "
  //                  "the robot model. Waiting 1s. ",
  //                  robotDescriptionParam.c_str());
  //     rclcpp::sleep_for(std::chrono::seconds(1));
  //   }

  //   // happens when configure() is called again from update() (e.g. when
  //   // a new bag file started
  //   // playing)
  //   if (!this->shapesToLinks.empty()) this->clearRobotMask();
  //   this->addRobotMaskFromUrdf(robotUrdf);
  // }

  RCLCPP_INFO(nodeHandle->get_logger(), "RobotBodyFilter: Successfullyconfigured.");
  RCLCPP_INFO(nodeHandle->get_logger(), "Filtering data inframe %s", this->filteringFrame.c_str());
  RCLCPP_INFO(nodeHandle->get_logger(), "RobotBodyFilter: Filtering into the following categories:");
  RCLCPP_INFO(nodeHandle->get_logger(), "RobotBodyFilter: \tOUTSIDE");
  if (doClipping) RCLCPP_INFO(nodeHandle->get_logger(), "RobotBodyFilter: \tCLIP");
  if (doContainsTest) RCLCPP_INFO(nodeHandle->get_logger(), "RobotBodyFilter: \tINSIDE");
  if (doShadowTest) RCLCPP_INFO(nodeHandle->get_logger(), "RobotBodyFilter: \tSHADOW");

  if (this->onlyLinks.empty()) {
    RCLCPP_INFO(nodeHandle->get_logger(),"RobotBodyFilter: onlyLinks is empty");
    if (this->linksIgnoredEverywhere.empty()) {
      RCLCPP_INFO(nodeHandle->get_logger(),"RobotBodyFilter: Filtering applied to all links.");
    } else {
      RCLCPP_INFO(nodeHandle->get_logger(),"RobotBodyFilter: Filtering applied to all links except %s.",
      to_string(this->linksIgnoredEverywhere).c_str());
    }
  } 
  else {
    if (this->linksIgnoredEverywhere.empty()) {
      // RCLCPP_INFO(nodeHandle->get_logger(),"RobotBodyFilter: Filtering applied to links %s.", to_string(this->onlyLinks).c_str());
    } 
    else {
    //TODO: The to_string call is crashing at runtime here, dont understand why
    // RCLCPP_INFO(nodeHandle->get_logger(),
    //             "RobotBodyFilter: Filtering applied to links %s with these links excluded: %s.",
    //             to_string(tempOnlyLinks).c_str(), to_string(tempLinksIgnoredEverywhere).c_str());
    for (const auto& link : this->onlyLinks) {
      RCLCPP_INFO(nodeHandle->get_logger(), "RobotBodyFilter: Filtering only applied to link %s.", link.c_str());
    }
    }
  }

  this->timeConfigured = this->nodeHandle->now();

  return true;
}

bool RobotBodyFilterLaserScan::configure() {

  RCLCPP_INFO(nodeHandle->get_logger(),"Declaring Parameters");
  //TODO: Implement
  // DeclareParameters();
  bool success = RobotBodyFilter::configure();
  return false;
}

bool RobotBodyFilterPointCloud2::configure() {

  RCLCPP_INFO(nodeHandle->get_logger(),"Declaring Parameters");

  DeclareParameters();

  RCLCPP_INFO(nodeHandle->get_logger(),"Configuring RobotBodyFilterPointCloud2");

  bool success = RobotBodyFilter::configure();
  if (!success) return false;

  this->nodeHandle->get_parameter_or("frames/output", this->outputFrame, this->filteringFrame);
  std::vector<std::string> tempPointChannels;
  std::vector<std::string> tempDirectionChannels;
  this->nodeHandle->get_parameter("cloud/point_channels", tempPointChannels);
  this->nodeHandle->get_parameter("cloud/direction_channels", tempDirectionChannels);
  const auto pointChannels = tempPointChannels;
  const auto directionChannels = tempDirectionChannels;

  for (const auto& channel : pointChannels) this->channelsToTransform[channel] = CloudChannelType::POINT;
  for (const auto& channel : directionChannels) this->channelsToTransform[channel] = CloudChannelType::DIRECTION;

  stripLeadingSlash(this->outputFrame, true);

  return true;
}

template <typename T>
bool RobotBodyFilter<T>::computeMask(
  const sensor_msgs::msg::PointCloud2& projectedPointCloud,
  std::vector<RayCastingShapeMask::MaskValue>& pointMask,
  const std::string& sensorFrame) {
  // this->modelMutex has to be already locked!
  RCLCPP_DEBUG(nodeHandle->get_logger(), "Computing Mask");
  const clock_t stopwatchOverall = clock();
  const auto& scanTime = projectedPointCloud.header.stamp;

  // compute a mask of point indices for points from projectedPointCloud
  // that tells if they are inside or outside robot, or shadow points
  if (!this->pointByPointScan) {
    Eigen::Vector3d sensorPosition;
    try {
      const auto sensorTf = this->tfBuffer->lookupTransform(this->filteringFrame, sensorFrame, scanTime,
                                                            remainingTime(*this->nodeHandle->get_clock(), scanTime, this->reachableTransformTimeout));
      tf2::fromMsg(sensorTf.transform.translation, sensorPosition);
    } catch (tf2::TransformException& e) {
      RCLCPP_ERROR(nodeHandle->get_logger(),
                   "RobotBodyFilter: Could not compute filtering mask due to this "
                   "TF exception: %s",
                   e.what());
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
    for (CloudConstIter stamps_end_it(projectedPointCloud, "stamps"); stamps_end_it != stamps_end_it.end();
         ++stamps_end_it) {
      if ((*stamps_end_it) > static_cast<float>(scanDuration)) scanDuration = static_cast<double>(*stamps_end_it);
    }

    const rclcpp::Time afterScanTime(rclcpp::Time(scanTime) + rclcpp::Duration::from_seconds(scanDuration));

    size_t updateBodyPosesEvery;
    if (this->modelPoseUpdateInterval.seconds() == 0 && this->modelPoseUpdateInterval.nanoseconds() == 0) {
      updateBodyPosesEvery = 1;
    } else {
      updateBodyPosesEvery = static_cast<size_t>(
          ceil(this->modelPoseUpdateInterval.seconds() / scanDuration * num_points(projectedPointCloud)));
      // prevent division by zero
      if (updateBodyPosesEvery == 0) updateBodyPosesEvery = 1;
    }

    // prevent division by zero in ratio computation in case the pointcloud
    // isn't really taken point by point with different timestamps
    if (scanDuration == 0.0) {
      updateBodyPosesEvery = num_points(projectedPointCloud) + 1;
      RCLCPP_WARN_ONCE(nodeHandle->get_logger(),
                       "RobotBodyFilter: sensor/point_by_point is set to true but "
                       "all points in the cloud have the same timestamp. You should"
                       " change the parameter to false to gain performance.");
    }

    // update transforms cache, which is then used in body masking
    this->updateTransformCache(scanTime, afterScanTime);

    Eigen::Vector3f point;
    Eigen::Vector3d viewPoint;
    RayCastingShapeMask::MaskValue mask;

    this->cacheLookupBetweenScansRatio = 0.0;
    for (size_t i = 0; i < num_points(projectedPointCloud);
         ++i, ++x_it, ++y_it, ++z_it, ++vp_x_it, ++vp_y_it, ++vp_z_it, ++stamps_it) {
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

  RCLCPP_DEBUG(nodeHandle->get_logger(), "RobotBodyFilter: Mask computed in %.5f secs.",
               double(clock() - stopwatchOverall) / CLOCKS_PER_SEC);

  this->publishDebugPointClouds(projectedPointCloud, pointMask);
  this->publishDebugMarkers(scanTime);
  this->computeAndPublishBoundingSphere(projectedPointCloud);
  this->computeAndPublishBoundingBox(projectedPointCloud);
  RCLCPP_DEBUG(nodeHandle->get_logger(), "computed and published bounding box");
  // this->computeAndPublishOrientedBoundingBox(projectedPointCloud);
  RCLCPP_DEBUG(nodeHandle->get_logger(), "computed and published oriented bounding box");
  // this->computeAndPublishLocalBoundingBox(projectedPointCloud);
  RCLCPP_DEBUG(nodeHandle->get_logger(), "computed and published local bounding box");

  RCLCPP_DEBUG(nodeHandle->get_logger(), "RobotBodyFilter: Filtering run time is %.5f secs.",
               double(clock() - stopwatchOverall) / CLOCKS_PER_SEC);
  return true;
}

bool RobotBodyFilterLaserScan::update(const sensor_msgs::msg::LaserScan& inputScan,
                                      sensor_msgs::msg::LaserScan& filteredScan) {
  
  const auto& headerScanTime = inputScan.header.stamp;
  const auto& scanTime = rclcpp::Time(inputScan.header.stamp);

  if (!this->configured_) {
    RCLCPP_DEBUG(nodeHandle->get_logger(),
                 "RobotBodyFilter: Ignore scan from time %f.%ld - filter not yet "
                 "initialized.",
                 scanTime.seconds(), scanTime.nanoseconds());
    return false;
  }

  if ((scanTime < timeConfigured) && ((scanTime + tfBufferLength) >= timeConfigured)) {
    RCLCPP_DEBUG(nodeHandle->get_logger(),
                 "RobotBodyFilter: Ignore scan from time %f.%ld - filter not yet "
                 "initialized.",
                 scanTime.seconds(), scanTime.nanoseconds());
    return false;
  }

  if ((scanTime < timeConfigured) && ((scanTime + tfBufferLength) < timeConfigured)) {
    RCLCPP_WARN(nodeHandle->get_logger(),
                "RobotBodyFilter: Old TF data received. Clearing TF buffer and "
                "reconfiguring laser"
                "filter. If you're replaying a bag file, make sure rosparam "
                "/use_sim_time is set to "
                "true");
    this->configure();
    return false;
  }

  // tf2 doesn't like frames starting with slash
  const auto scanFrame = stripLeadingSlash(inputScan.header.frame_id, true);

  // Passing a sensorFrame does not make sense. Scan messages can't be
  // transformed to other frames.
  if (!this->sensorFrame.empty() && this->sensorFrame != scanFrame) {
    RCLCPP_WARN_ONCE(nodeHandle->get_logger(),
                     "RobotBodyFilter: frames/sensor is set to frame_id '%s' different than "
                     "the frame_id of the incoming message '%s'. This is an invalid "
                     "configuration: "
                     "the frames/sensor parameter will be neglected.",
                     this->sensorFrame.c_str(), scanFrame.c_str());
  }

  if (!this->tfFramesWatchdog->isReachable(scanFrame)) {
    RCLCPP_DEBUG(nodeHandle->get_logger(),
                 "RobotBodyFilter: Throwing away scan since sensor frame is "
                 "unreachable.");
    // if this->sensorFrame is empty, it can happen that we're not actually
    // monitoring the sensor frame, so start monitoring it
    if (!this->tfFramesWatchdog->isMonitored(scanFrame)) this->tfFramesWatchdog->addMonitoredFrame(scanFrame);
    return false;
  }

  if (this->requireAllFramesReachable && !this->tfFramesWatchdog->areAllFramesReachable()) {
    RCLCPP_DEBUG(nodeHandle->get_logger(),
                 "RobotBodyFilter: Throwing away scan since not all frames are "
                 "reachable.");
    return false;
  }

  const clock_t stopwatchOverall = clock();

  // create the output copy of the input scan
  filteredScan = inputScan;
  filteredScan.header.frame_id = scanFrame;
  filteredScan.range_min = fmax(inputScan.range_min, (float)this->minDistance);
  if (this->maxDistance > 0.0) filteredScan.range_max = fmin(inputScan.range_max, (float)this->maxDistance);

  {  // acquire the lock here, because we work with the tfBuffer all the time
    std::lock_guard<std::mutex> guard(*this->modelMutex);

    if (this->pointByPointScan) {  // make sure we have all the tfs between
                                   // sensor frame and fixedFrame during the time
                                   // of scan acquisition
      const auto scanDuration = inputScan.ranges.size() * inputScan.time_increment;
      const auto afterScanTime = scanTime + rclcpp::Duration::from_seconds(scanDuration);

      std::string err;
      if (!this->tfBuffer->canTransform(this->fixedFrame, scanFrame, scanTime,
                                        remainingTime(*this->nodeHandle->get_clock(), scanTime, this->reachableTransformTimeout), &err) ||
          !this->tfBuffer->canTransform(this->fixedFrame, scanFrame, afterScanTime,
                                        remainingTime(*this->nodeHandle->get_clock(), afterScanTime, this->reachableTransformTimeout), &err)) {
        if (err.find("future") != std::string::npos) {
          const auto delay = nodeHandle->now() - scanTime;
          auto& clk = *nodeHandle->get_clock();
          RCLCPP_ERROR_THROTTLE(nodeHandle->get_logger(), clk, 3,
                                "RobotBodyFilter: Cannot transform laser scan to "
                                "fixed frame. The scan is too much delayed (%s s). TF error: %s",
                                to_string(delay).c_str(), err.c_str());
        } else {
          auto& clk = *nodeHandle->get_clock();
          RCLCPP_ERROR_THROTTLE(nodeHandle->get_logger(), clk, 3,
                                "RobotBodyFilter: Cannot transform laser scan to "
                                "fixed frame. Something's wrong with TFs: %s",
                                err.c_str());
        }
        return false;
      }
    }

    // The point cloud will have fields x, y, z, intensity (float32) and index
    // (int32) and for point-by-point scans also timestamp and viewpoint
    sensor_msgs::msg::PointCloud2 projectedPointCloud;
    {  // project the scan measurements to a point cloud in the filteringFrame

      sensor_msgs::msg::PointCloud2 tmpPointCloud;

      // the projected point cloud can omit some measurements if they are out of
      // the defined scan's range; for this case, the second channel ("index")
      // contains indices of the point cloud's points into the scan
      auto channelOptions = laser_geometry::channel_option::Intensity | laser_geometry::channel_option::Index;

      if (this->pointByPointScan) {
        RCLCPP_INFO_ONCE(nodeHandle->get_logger(), "RobotBodyFilter: Applying complex laser scan projection.");
        // perform the complex laser scan projection
        channelOptions |= laser_geometry::channel_option::Timestamp | laser_geometry::channel_option::Viewpoint;

        laserProjector.transformLaserScanToPointCloud(this->fixedFrame, inputScan, tmpPointCloud, *this->tfBuffer, -1,
                                                      channelOptions);
      } else {
        RCLCPP_INFO_ONCE(nodeHandle->get_logger(), "RobotBodyFilter: Applying simple laser scan projection.");
        // perform simple laser scan projection
        laserProjector.projectLaser(inputScan, tmpPointCloud, -1.0, channelOptions);
      }

      // convert to filtering frame
      if (tmpPointCloud.header.frame_id == this->filteringFrame) {
        projectedPointCloud = std::move(tmpPointCloud);
      } else {
        RCLCPP_INFO_ONCE(nodeHandle->get_logger(), "RobotBodyFilter: Transforming scan from frame %s to %s",
                         tmpPointCloud.header.frame_id.c_str(), this->filteringFrame.c_str());
        std::string err;
        if (!this->tfBuffer->canTransform(this->filteringFrame, tmpPointCloud.header.frame_id, scanTime,
                                          remainingTime(*this->nodeHandle->get_clock(), scanTime, this->reachableTransformTimeout), &err)) {
          auto& clk = *nodeHandle->get_clock();
          RCLCPP_ERROR_THROTTLE(nodeHandle->get_logger(), clk, 3,
                                "RobotBodyFilter: Cannot transform "
                                "laser scan to filtering frame. Something's wrong with TFs: %s",
                                err.c_str());
          return false;
        }

        transformWithChannels(tmpPointCloud, projectedPointCloud, *this->tfBuffer, this->filteringFrame,
                              this->channelsToTransform);
      }
    }

    RCLCPP_DEBUG(nodeHandle->get_logger(), "RobotBodyFilter: Scan transformation run time is %.5f secs.",
                 double(clock() - stopwatchOverall) / CLOCKS_PER_SEC);

    std::vector<RayCastingShapeMask::MaskValue> pointMask;
    const auto success = this->computeMask(projectedPointCloud, pointMask, scanFrame);

    if (!success) return false;

    {  // remove invalid points
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
      } catch (std::runtime_error& ) {
        RCLCPP_ERROR(this->nodeHandle->get_logger(),
                     "RobotBodyFilter: projectedPointCloud doesn't have field "
                     "called 'index',"
                     " but the algorithm relies on that.");
        return false;
      }
    }
  }

  return true;
}

bool RobotBodyFilterPointCloud2::update(const sensor_msgs::msg::PointCloud2& inputCloud,
                                        sensor_msgs::msg::PointCloud2& filteredCloud) {
  const auto& headerScanTime = inputCloud.header.stamp;
  const auto& scanTime = rclcpp::Time(headerScanTime);

  if (!this->configured_) {
    RCLCPP_DEBUG(nodeHandle->get_logger(),
                 "RobotBodyFilter: Ignore cloud from time %f.%ld - filter not yet "
                 "initialized.",
                 scanTime.seconds(), scanTime.nanoseconds());
    return false;
  }

  if ((scanTime < this->timeConfigured) && ((scanTime + this->tfBufferLength) >= this->timeConfigured)) {
    RCLCPP_DEBUG(nodeHandle->get_logger(),
                 "RobotBodyFilter: Ignore cloud from time %f.%ld - filter not yet "
                 "initialized.",
                 scanTime.seconds(), scanTime.nanoseconds());
    return false;
  }

  if ((scanTime < this->timeConfigured) && ((scanTime + this->tfBufferLength) < this->timeConfigured)) {
    RCLCPP_WARN(nodeHandle->get_logger(),
                "RobotBodyFilter: Old TF data received. Clearing TF buffer and "
                "reconfiguring laser filter. If you're replaying a bag file, make "
                "sure rosparam /use_sim_time is set to true");
    this->configure();
    return false;
  }

  const auto inputCloudFrame =
      this->sensorFrame.empty() ? stripLeadingSlash(inputCloud.header.frame_id, true) : this->sensorFrame;

  if (!this->tfFramesWatchdog->isReachable(inputCloudFrame)) {
    RCLCPP_DEBUG(nodeHandle->get_logger(),
                 "inputCloudFrame: %s", inputCloudFrame.c_str());
    RCLCPP_DEBUG(nodeHandle->get_logger(),
                 "RobotBodyFilter: Throwing away scan since sensor frame is "
                 "unreachable.");
    // if this->sensorFrame is empty, it can happen that we're not actually
    // monitoring the cloud frame, so start monitoring it
    if (!this->tfFramesWatchdog->isMonitored(inputCloudFrame))
      this->tfFramesWatchdog->addMonitoredFrame(inputCloudFrame);
    return false;
  }

  if (this->requireAllFramesReachable && !this->tfFramesWatchdog->areAllFramesReachable()) {
    RCLCPP_DEBUG(nodeHandle->get_logger(),
                 "RobotBodyFilter: Throwing away scan since not all frames are "
                 "reachable.");
    return false;
  }

  bool hasStampsField = false;
  bool hasVpXField = false, hasVpYField = false, hasVpZField = false;
  for (const auto& field : inputCloud.fields) {
    if (field.name == "stamps" && field.datatype == sensor_msgs::msg::PointField::FLOAT32)
      hasStampsField = true;
    else if (field.name == "vp_x" && field.datatype == sensor_msgs::msg::PointField::FLOAT32)
      hasVpXField = true;
    else if (field.name == "vp_y" && field.datatype == sensor_msgs::msg::PointField::FLOAT32)
      hasVpYField = true;
    else if (field.name == "vp_z" && field.datatype == sensor_msgs::msg::PointField::FLOAT32)
      hasVpZField = true;
  }

  // Verify the pointcloud and its fields

  if (this->pointByPointScan) {
    if (inputCloud.height != 1 && inputCloud.is_dense == 0) {
      RCLCPP_WARN_ONCE(nodeHandle->get_logger(),
                       "RobotBodyFilter: The pointcloud seems to be an organized "
                       "pointcloud, which usually means it was captured all at once."
                       " Consider setting 'point_by_point_scan' to false to get a "
                       "more efficient computation.");
    }
    if (!hasStampsField || !hasVpXField || !hasVpYField || !hasVpZField) {
      throw std::runtime_error(
          "A point-by-point scan has to contain float32"
          "fields 'stamps', 'vp_x', 'vp_y' and 'vp_z'.");
    }
  } else if (hasStampsField) {
    RCLCPP_WARN_ONCE(nodeHandle->get_logger(),
                     "RobotBodyFilter: The pointcloud has a 'stamps' field, "
                     "which indicates each point was probably captured at a "
                     "different time instant. Consider setting parameter "
                     "'point_by_point_scan' to true to get correct results.");
  } else if (inputCloud.height == 1 && inputCloud.is_dense == 1) {
    RCLCPP_WARN_ONCE(nodeHandle->get_logger(),
                     "RobotBodyFilter: The pointcloud is dense, which usually means"
                     " it was captured each point at a different time instant. "
                     "Consider setting 'point_by_point_scan' to true to get a more"
                     " accurate version.");
  }

  // Transform to filtering frame
  RCLCPP_DEBUG(nodeHandle->get_logger(), "RobotBodyFilter: Transforming point cloud from input frame: %s to filtering frame: %s",
               inputCloud.header.frame_id.c_str(), this->filteringFrame.c_str());

  sensor_msgs::msg::PointCloud2 transformedCloud;
  if (inputCloud.header.frame_id == this->filteringFrame) {
    transformedCloud = inputCloud;
  } else {
    RCLCPP_INFO_ONCE(nodeHandle->get_logger(), "RobotBodyFilter: Transforming cloud from frame %s to %s",
                     inputCloud.header.frame_id.c_str(), this->filteringFrame.c_str());
    std::lock_guard<std::mutex> guard(*this->modelMutex);
    std::string err;
    if (!this->tfBuffer->canTransform(this->filteringFrame, inputCloud.header.frame_id, scanTime,
                                      remainingTime(*this->nodeHandle->get_clock(), scanTime, this->reachableTransformTimeout), &err)) {
      auto& clk = *nodeHandle->get_clock();
      RCLCPP_ERROR_THROTTLE(nodeHandle->get_logger(), clk, 3,
                            "RobotBodyFilter: Cannot transform "
                            "point cloud to filtering frame. Something's wrong with TFs: %s",
                            err.c_str());
      return false;
    }
    transformWithChannels(inputCloud, transformedCloud, *this->tfBuffer, this->filteringFrame,
                          this->channelsToTransform);
  }

  // Compute the mask and use it (transform message only if sensorFrame is
  // specified)
  RCLCPP_DEBUG(nodeHandle->get_logger(), "Computing Mask");
  std::vector<RayCastingShapeMask::MaskValue> pointMask;
  {
    std::lock_guard<std::mutex> guard(*this->modelMutex);

    const auto success = this->computeMask(transformedCloud, pointMask, inputCloudFrame);
    if (!success) return false;
  }

  // Filter the cloud
  RCLCPP_DEBUG(nodeHandle->get_logger(), "Filtering the cloud");

  sensor_msgs::msg::PointCloud2 tmpCloud;
  CREATE_FILTERED_CLOUD(transformedCloud, tmpCloud, this->keepCloudsOrganized,
                        (pointMask[i] == RayCastingShapeMask::MaskValue::OUTSIDE))

  // Transform to output frame
  RCLCPP_DEBUG(nodeHandle->get_logger(), "Transform to output frame");

  if (tmpCloud.header.frame_id == this->outputFrame) {
    filteredCloud = std::move(tmpCloud);
  } else {
    RCLCPP_INFO_ONCE(nodeHandle->get_logger(), "RobotBodyFilter: Transforming cloud from frame %s to %s",
                     tmpCloud.header.frame_id.c_str(), this->outputFrame.c_str());
    std::lock_guard<std::mutex> guard(*this->modelMutex);
    std::string err;
    if (!this->tfBuffer->canTransform(this->outputFrame, tmpCloud.header.frame_id, scanTime,
                                      remainingTime(*this->nodeHandle->get_clock(), scanTime, this->reachableTransformTimeout), &err)) {
      auto& clk = *nodeHandle->get_clock();
      RCLCPP_ERROR_THROTTLE(nodeHandle->get_logger(), clk, 3,
                            "RobotBodyFilter: Cannot transform "
                            "point cloud to output frame. Something's wrong with TFs: %s",
                            err.c_str());
      return false;
    }

    transformWithChannels(tmpCloud, filteredCloud, *this->tfBuffer, this->outputFrame, this->channelsToTransform);
  }

  return true;
}

template <typename T>
bool RobotBodyFilter<T>::getShapeTransform(point_containment_filter::ShapeHandle shapeHandle,
                                           Eigen::Isometry3d& transform) const {
  // make sure you locked this->modelMutex

  // check if the given shapeHandle has been registered to a link during
  // addRobotMaskFromUrdf call.
  if (this->shapesToLinks.find(shapeHandle) == this->shapesToLinks.end()) {
    auto& clk = *nodeHandle->get_clock();
    RCLCPP_ERROR_THROTTLE(nodeHandle->get_logger(), clk,
        3, "RobotBodyFilter: Invalid shape handle: %s", to_string(shapeHandle).c_str());
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
    if (this->transformCacheAfterScan.find(collision.cacheKey) == this->transformCacheAfterScan.end()) {
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

template <typename T>
void RobotBodyFilter<T>::updateTransformCache(const rclcpp::Time& time, const rclcpp::Time& afterScanTime) {
  // make sure you locked this->modelMutex

  // clear the cache so that maskContainment always uses only these tf data and
  // not some older
  this->transformCache.clear();
  if (afterScanTime.seconds() != 0) this->transformCacheAfterScan.clear();

  // iterate over all links corresponding to some masking shape and update their
  // cached transforms relative to fixed_frame
  for (auto& shapeToLink : this->shapesToLinks) {
    const auto& collisionBody = shapeToLink.second;
    const auto& collision = collisionBody.collision;
    const auto& link = collisionBody.link;

    // here we assume the tf frames' names correspond to the link names
    const auto linkFrame = link->name;

    // the collision object may have a different origin than the visual, we need
    // to account for that
    const auto& collisionOffsetTransform = urdfPose2EigenTransform(collision->origin);

    {
      auto linkTransformTfOptional = this->tfFramesWatchdog->lookupTransform(
          linkFrame, time, remainingTime(*this->nodeHandle->get_clock(), time, this->reachableTransformTimeout));

      if (!linkTransformTfOptional)  // has no value
        continue;

      const auto& linkTransformTf = linkTransformTfOptional.value();
      const auto& linkTransformEigen = tf2::transformToEigen(linkTransformTf);

      const auto& transform = linkTransformEigen * collisionOffsetTransform;

      this->transformCache[collisionBody.cacheKey] =
          std::allocate_shared<Eigen::Isometry3d>(Eigen::aligned_allocator<Eigen::Isometry3d>(), transform);
    }

    if (afterScanTime.seconds() != 0) {
      auto linkTransformTfOptional = this->tfFramesWatchdog->lookupTransform(
          linkFrame, afterScanTime, remainingTime(*this->nodeHandle->get_clock(), time, this->reachableTransformTimeout));

      if (!linkTransformTfOptional)  // has no value
        continue;

      const auto& linkTransformTf = linkTransformTfOptional.value();
      const auto& linkTransformEigen = tf2::transformToEigen(linkTransformTf);

      const auto& transform = linkTransformEigen * collisionOffsetTransform;

      this->transformCacheAfterScan[collisionBody.cacheKey] =
          std::allocate_shared<Eigen::Isometry3d>(Eigen::aligned_allocator<Eigen::Isometry3d>(), transform);
    }
  }
}

template <typename T>
void RobotBodyFilter<T>::addRobotMaskFromUrdf(const std::string& urdfModel) {
  if (urdfModel.empty()) {
    RCLCPP_ERROR(nodeHandle->get_logger(),
                 "RobotBodyFilter: Empty string passed as robot model to "
                 "addRobotMaskFromUrdf. "
                 "Robot body filtering is not going to work.");
    return;
  }

  // parse the URDF model
  urdf::Model parsedUrdfModel;
  bool urdfParseSucceeded = parsedUrdfModel.initString(urdfModel);
  if (!urdfParseSucceeded) {
    RCLCPP_ERROR(nodeHandle->get_logger(),
                 "RobotBodyFilter: The URDF model given in parameter %s cannot be parsed. See urdf::Model::initString for debugging, or try running gzsdf my_robot.urdf",
                 this->robotDescriptionParam.c_str());
    return;
  }

  {
    std::lock_guard<std::mutex> guard(*this->modelMutex);

    this->shapesIgnoredInBoundingSphere.clear();
    this->shapesIgnoredInBoundingBox.clear();
    std::unordered_set<MultiShapeHandle> ignoreInContainsTest;
    std::unordered_set<MultiShapeHandle> ignoreInShadowTest;

    // add all model's collision links as masking shapes
    for (const auto& links : parsedUrdfModel.links_) {
      RCLCPP_DEBUG(nodeHandle->get_logger(), "RobotBodyFilter: Adding link %s to the mask.", links.first.c_str());
      const auto& link = links.second;

      // every link can have multiple collision elements
      size_t collisionIndex = 0;
      for (const auto& collision : link->collision_array) {
        if (collision->geometry == nullptr) {
          RCLCPP_WARN(nodeHandle->get_logger(),
                      "RobotBodyFilter: Collision element without geometry found "
                      "in link %s of robot %s. "
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
        for (const auto& name : collisionNames) {
          collisionNamesSet.insert(name);
          collisionNamesContains.insert(name + CONTAINS_SUFFIX);
          collisionNamesShadow.insert(name + SHADOW_SUFFIX);
        }

        // if onlyLinks is nonempty, make sure this collision belongs to a
        // specified link
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

        // if the shape could not be constructed, ignore it (e.g. mesh was not
        // found)
        if (collisionShape == nullptr) {
          RCLCPP_WARN(nodeHandle->get_logger(), "Could not construct shape for collision %s, ignoring it.",
                      shapeName.c_str());
          ++collisionIndex;
          continue;
        }

        // add the collision shape to shapeMask; the inflation parameters come
        // into play here
        const auto containsTestInflation = this->getLinkInflationForContainsTest(collisionNames);
        const auto shadowTestInflation = this->getLinkInflationForShadowTest(collisionNames);
        const auto bsphereInflation = this->getLinkInflationForBoundingSphere(collisionNames);
        const auto bboxInflation = this->getLinkInflationForBoundingBox(collisionNames);
        const auto shapeHandle = this->shapeMask->addShape(
            collisionShape, containsTestInflation.scale, containsTestInflation.padding, shadowTestInflation.scale,
            shadowTestInflation.padding, bsphereInflation.scale, bsphereInflation.padding, bboxInflation.scale,
            bboxInflation.padding, false, shapeName);
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

      // no collision element found; only warn for links that are not ignored
      // and have at least one visual
      if (collisionIndex == 0 && !link->visual_array.empty()) {
        if ((this->onlyLinks.empty() || (this->onlyLinks.find(link->name) != this->onlyLinks.end())) &&
            this->linksIgnoredEverywhere.find(link->name) == this->linksIgnoredEverywhere.end()) {
          RCLCPP_WARN(nodeHandle->get_logger(),
                      "RobotBodyFilter: No collision element found for link %s of "
                      "robot %s. This link will not be filtered out "
                      "from laser scans.",
                      link->name.c_str(), parsedUrdfModel.getName().c_str());
        }
      }
    }

    this->shapeMask->setIgnoreInContainsTest(ignoreInContainsTest);
    this->shapeMask->setIgnoreInShadowTest(ignoreInShadowTest);

    this->shapeMask->updateInternalShapeLists();

    std::set<std::string> monitoredFrames;
    for (const auto& shapeToLink : this->shapesToLinks) monitoredFrames.insert(shapeToLink.second.link->name);
    // Issue #6: Monitor sensor frame even if it is not a part of the model
    if (!this->sensorFrame.empty()) monitoredFrames.insert(this->sensorFrame);

    this->tfFramesWatchdog->setMonitoredFrames(monitoredFrames);
  }
}

template <typename T>
void RobotBodyFilter<T>::clearRobotMask() {
  {
    RCLCPP_INFO(nodeHandle->get_logger(), "RobotBodyFilter: modelMutex not yet locked.");

    std::lock_guard<std::mutex> guard(*this->modelMutex);

    RCLCPP_INFO(nodeHandle->get_logger(), "RobotBodyFilter: modelMutex locked.");

    std::unordered_set<MultiShapeHandle> removedMultiShapes;
    for (const auto& shapeToLink : this->shapesToLinks) {
      const auto& multiShape = shapeToLink.second.multiHandle;
      if (removedMultiShapes.find(multiShape) == removedMultiShapes.end()) {
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
void RobotBodyFilter<T>::publishDebugMarkers(const rclcpp::Time& scanTime) const {
  // assume this->modelMutex is locked

  if (this->publishDebugContainsMarker) {
    visualization_msgs::msg::MarkerArray markerArray;
    std_msgs::msg::ColorRGBA color;
    color.g = 1.0;
    color.a = 0.5;
    createBodyVisualizationMsg(this->shapeMask->getBodiesForContainsTest(), scanTime, color, markerArray);
    this->debugContainsMarkerPublisher->publish(markerArray);
  }

  if (this->publishDebugShadowMarker) {
    visualization_msgs::msg::MarkerArray markerArray;
    std_msgs::msg::ColorRGBA color;
    color.b = 1.0;
    color.a = 0.5;
    createBodyVisualizationMsg(this->shapeMask->getBodiesForShadowTest(), scanTime, color, markerArray);
    this->debugShadowMarkerPublisher->publish(markerArray);
  }

  if (this->publishDebugBsphereMarker) {
    visualization_msgs::msg::MarkerArray markerArray;
    std_msgs::msg::ColorRGBA color;
    color.g = 1.0;
    color.b = 1.0;
    color.a = 0.5;
    createBodyVisualizationMsg(this->shapeMask->getBodiesForBoundingSphere(), scanTime, color, markerArray);
    this->debugBsphereMarkerPublisher->publish(markerArray);
  }

  if (this->publishDebugBboxMarker) {
    visualization_msgs::msg::MarkerArray markerArray;
    std_msgs::msg::ColorRGBA color;
    color.r = 1.0;
    color.b = 1.0;
    color.a = 0.5;
    createBodyVisualizationMsg(this->shapeMask->getBodiesForBoundingBox(), scanTime, color, markerArray);
    this->debugBboxMarkerPublisher->publish(markerArray);
  }
}

template <typename T>
void RobotBodyFilter<T>::publishDebugPointClouds(const sensor_msgs::msg::PointCloud2& projectedPointCloud,
                                                 const std::vector<RayCastingShapeMask::MaskValue>& pointMask) const {
  if (this->publishDebugPclInside) {
    sensor_msgs::msg::PointCloud2 insideCloud;
    CREATE_FILTERED_CLOUD(projectedPointCloud, insideCloud, this->keepCloudsOrganized,
                          (pointMask[i] == RayCastingShapeMask::MaskValue::INSIDE));
    this->debugPointCloudInsidePublisher->publish(insideCloud);
  }

  if (this->publishDebugPclClip) {
    sensor_msgs::msg::PointCloud2 clipCloud;
    CREATE_FILTERED_CLOUD(projectedPointCloud, clipCloud, this->keepCloudsOrganized,
                          (pointMask[i] == RayCastingShapeMask::MaskValue::CLIP));
    this->debugPointCloudClipPublisher->publish(clipCloud);
  }

  if (this->publishDebugPclShadow) {
    sensor_msgs::msg::PointCloud2 shadowCloud;
    CREATE_FILTERED_CLOUD(projectedPointCloud, shadowCloud, this->keepCloudsOrganized,
                          (pointMask[i] == RayCastingShapeMask::MaskValue::SHADOW));
    this->debugPointCloudShadowPublisher->publish(shadowCloud);
  }
}

template <typename T>
void RobotBodyFilter<T>::computeAndPublishBoundingSphere(
    const sensor_msgs::msg::PointCloud2& projectedPointCloud) const {
  if (!this->computeBoundingSphere && !this->computeDebugBoundingSphere) return;

  RCLCPP_DEBUG(nodeHandle->get_logger(), "Computing and Publishing Bounding Sphere");

  // assume this->modelMutex is locked

  // when computing bounding spheres for publication, we want to publish them to
  // the time of the scan, so we need to set cacheLookupBetweenScansRatio again
  // to zero
  if (this->cacheLookupBetweenScansRatio != 0.0) {
    this->cacheLookupBetweenScansRatio = 0.0;
    this->shapeMask->updateBodyPoses();
  }

  const auto& scanTime = projectedPointCloud.header.stamp;
  std::vector<bodies::BoundingSphere> spheres;
  {
    visualization_msgs::msg::MarkerArray boundingSphereDebugMsg;

    for (const auto& shapeHandleAndBody : this->shapeMask->getBodiesForBoundingSphere()) {
      const auto& shapeHandle = shapeHandleAndBody.first;
      const auto& body = shapeHandleAndBody.second;
      if (this->shapesIgnoredInBoundingSphere.find(shapeHandle) != this->shapesIgnoredInBoundingSphere.end()) continue;

      bodies::BoundingSphere sphere;
      body->computeBoundingSphere(sphere);

      spheres.push_back(sphere);
      if (this->computeDebugBoundingSphere) {
        visualization_msgs::msg::Marker msg;
        msg.header.stamp = scanTime;
        msg.header.frame_id = this->filteringFrame;

        msg.scale.x = msg.scale.y = msg.scale.z = sphere.radius * 2;

        msg.pose.position.x = sphere.center[0];
        msg.pose.position.y = sphere.center[1];
        msg.pose.position.z = sphere.center[2];
        msg.pose.orientation.w = 1;

        msg.color.g = 1.0;
        msg.color.a = 0.5;
        msg.type = visualization_msgs::msg::Marker::SPHERE;
        msg.action = visualization_msgs::msg::Marker::ADD;
        msg.ns = "bsphere/" + this->shapesToLinks.at(shapeHandle).cacheKey;
        msg.frame_locked = static_cast<unsigned char>(true);

        boundingSphereDebugMsg.markers.push_back(msg);
      }
    }

    if (this->computeDebugBoundingSphere) {
      this->boundingSphereDebugMarkerPublisher->publish(boundingSphereDebugMsg);
    }
  }

  if (this->computeBoundingSphere) {
    bodies::BoundingSphere boundingSphere;
    bodies::mergeBoundingSpheres(spheres, boundingSphere);

    // robot_body_filter::SphereStamped boundingSphereMsg;
    // boundingSphereMsg.header.stamp = scanTime;
    // boundingSphereMsg.header.frame_id = this->filteringFrame;
    // boundingSphereMsg.sphere.radius =
    // static_cast<float>(boundingSphere.radius);
    // boundingSphereMsg.sphere.center = tf2::toMsg(boundingSphere.center);

    // this->boundingSpherePublisher.publish(boundingSphereMsg);

    if (this->publishBoundingSphereMarker) {
      visualization_msgs::msg::Marker msg;
      msg.header.stamp = scanTime;
      msg.header.frame_id = this->filteringFrame;

      msg.scale.x = msg.scale.y = msg.scale.z = boundingSphere.radius * 2;

      msg.pose.position.x = boundingSphere.center[0];
      msg.pose.position.y = boundingSphere.center[1];
      msg.pose.position.z = boundingSphere.center[2];
      msg.pose.orientation.w = 1;

      msg.color.g = 1.0;
      msg.color.a = 0.5;
      msg.type = visualization_msgs::msg::Marker::SPHERE;
      msg.action = visualization_msgs::msg::Marker::ADD;
      msg.ns = "bounding_sphere";
      msg.frame_locked = static_cast<unsigned char>(true);

      this->boundingSphereMarkerPublisher->publish(msg);
    }

    if (this->publishNoBoundingSpherePointcloud) {
      sensor_msgs::msg::PointCloud2 noSphereCloud;
      CREATE_FILTERED_CLOUD(
          projectedPointCloud, noSphereCloud, this->keepCloudsOrganized,
          ((Eigen::Vector3d(*x_it, *y_it, *z_it) - boundingSphere.center).norm() > boundingSphere.radius));
      this->scanPointCloudNoBoundingSpherePublisher->publish(noSphereCloud);
    }
  }
}

template <typename T>
void RobotBodyFilter<T>::computeAndPublishBoundingBox(const sensor_msgs::msg::PointCloud2& projectedPointCloud) const {
  if (!this->computeBoundingBox && !this->computeDebugBoundingBox) return;

  // assume this->modelMutex is locked

  // when computing bounding boxes for publication, we want to publish them to
  // the time of the scan, so we need to set cacheLookupBetweenScansRatio again
  // to zero
  if (this->cacheLookupBetweenScansRatio != 0.0) {
    this->cacheLookupBetweenScansRatio = 0.0;
    this->shapeMask->updateBodyPoses();
  }

  const auto& scanTime = projectedPointCloud.header.stamp;
  std::vector<bodies::AxisAlignedBoundingBox> boxes;
  {
    visualization_msgs::msg::MarkerArray boundingBoxDebugMsg;
    RCLCPP_INFO(nodeHandle->get_logger(), "Number of bodies: %zu", this->shapeMask->getBodiesForBoundingBox().size());
    for (const auto& shapeHandleAndBody : this->shapeMask->getBodiesForBoundingBox()) {
      RCLCPP_INFO(nodeHandle->get_logger(), "declaring variables");
      const auto& shapeHandle = shapeHandleAndBody.first;
      const auto& body = shapeHandleAndBody.second;
      RCLCPP_INFO(nodeHandle->get_logger(), "variables declared");
      // RCLCPP_INFO(nodeHandle->get_logger(), "Looking for shape handle %s", to_string(shapeHandle).c_str());
      if (this->shapesIgnoredInBoundingBox.find(shapeHandle) != this->shapesIgnoredInBoundingBox.end()) continue;

      bodies::AxisAlignedBoundingBox box;
      // RCLCPP_INFO(nodeHandle->get_logger(), "Computing bounding box for shape %s", to_string(shapeHandle).c_str());
      body->computeBoundingBox(box);
      RCLCPP_INFO(nodeHandle->get_logger(), "Computed");

      boxes.push_back(box);

      if (this->computeDebugBoundingBox) {
        visualization_msgs::msg::Marker msg;
        msg.header.stamp = scanTime;
        msg.header.frame_id = this->filteringFrame;

        // it is aligned to fixed frame, not necessarily robot frame
        tf2::toMsg(box.sizes(), msg.scale);
        msg.pose.position = tf2::toMsg((Eigen::Vector3d)box.center());
        msg.pose.orientation.w = 1;

        msg.color.g = 1.0;
        msg.color.a = 0.5;
        msg.type = visualization_msgs::msg::Marker::CUBE;
        msg.action = visualization_msgs::msg::Marker::ADD;
        msg.ns = "bbox/" + this->shapesToLinks.at(shapeHandle).cacheKey;
        msg.frame_locked = static_cast<unsigned char>(true);

        boundingBoxDebugMsg.markers.push_back(msg);
      }
    }

    if (this->computeDebugBoundingBox) {
      RCLCPP_INFO(nodeHandle->get_logger(), "Publishing bounding box debug markers");
      this->boundingBoxDebugMarkerPublisher->publish(boundingBoxDebugMsg);
    }
  }

  if (this->computeBoundingBox) {
    bodies::AxisAlignedBoundingBox box;
    bodies::mergeBoundingBoxes(boxes, box);
    const auto boxFloat = box.cast<float>();

    geometry_msgs::msg::PolygonStamped boundingBoxMsg;

    boundingBoxMsg.header.stamp = scanTime;
    boundingBoxMsg.header.frame_id = this->filteringFrame;

    boundingBoxMsg.polygon.points.resize(2);
    tf2::toMsg(box.min(), boundingBoxMsg.polygon.points[0]);
    tf2::toMsg(box.max(), boundingBoxMsg.polygon.points[1]);

    RCLCPP_INFO(nodeHandle->get_logger(), "Publishing bounding box message");
    this->boundingBoxPublisher->publish(boundingBoxMsg);

    if (this->publishBoundingBoxMarker) {
      visualization_msgs::msg::Marker msg;
      msg.header.stamp = scanTime;
      msg.header.frame_id = this->filteringFrame;

      // it is aligned to fixed frame and not necessarily to robot frame
      tf2::toMsg(box.sizes(), msg.scale);
      msg.pose.position = tf2::toMsg((Eigen::Vector3d)box.center());
      msg.pose.orientation.w = 1;

      msg.color.r = 1.0;
      msg.color.a = 0.5;
      msg.type = visualization_msgs::msg::Marker::CUBE;
      msg.action = visualization_msgs::msg::Marker::ADD;
      msg.ns = "bounding_box";
      msg.frame_locked = static_cast<unsigned char>(true);

      RCLCPP_INFO(nodeHandle->get_logger(), "Publishing bounding box Marker");
      this->boundingBoxMarkerPublisher->publish(msg);
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

      sensor_msgs::msg::PointCloud2::SharedPtr boxFilteredCloud(new sensor_msgs::msg::PointCloud2());
      pcl_conversions::moveFromPCL(pclOutput, *boxFilteredCloud);
      boxFilteredCloud->header.stamp = scanTime;  // PCL strips precision of timestamp
      RCLCPP_INFO(nodeHandle->get_logger(), "Publish scan cloud");
      this->scanPointCloudNoBoundingBoxPublisher->publish(*boxFilteredCloud);
    }
  }
}

// template <typename T>
// void RobotBodyFilter<T>::computeAndPublishOrientedBoundingBox(
//     const sensor_msgs::msg::PointCloud2& projectedPointCloud) const {
//   if (!this->computeOrientedBoundingBox && !this->computeDebugOrientedBoundingBox) return;

//   // assume this->modelMutex is locked

//   // when computing bounding boxes for publication, we want to publish them to
//   // the time of the scan, so we need to set cacheLookupBetweenScansRatio again
//   // to zero
//   if (this->cacheLookupBetweenScansRatio != 0.0) {
//     this->cacheLookupBetweenScansRatio = 0.0;
//     this->shapeMask->updateBodyPoses();
//   }

//   const auto& scanTime = projectedPointCloud.header.stamp;
//   std::vector<bodies::OrientedBoundingBox> boxes;

//   {
//     visualization_msgs::msg::MarkerArray boundingBoxDebugMsg;
//     for (const auto& shapeHandleAndBody : this->shapeMask->getBodiesForBoundingBox()) {
//       const auto& shapeHandle = shapeHandleAndBody.first;
//       const auto& body = shapeHandleAndBody.second;

//       if (this->shapesIgnoredInBoundingBox.find(shapeHandle) != this->shapesIgnoredInBoundingBox.end())
//         continue;

//       bodies::OrientedBoundingBox box;
//       body->computeBoundingBox(box);

//       boxes.push_back(box);

//       if (this->computeDebugOrientedBoundingBox) {
//         visualization_msgs::msg::Marker msg;
//         msg.header.stamp = scanTime;
//         msg.header.frame_id = this->filteringFrame;

//         tf2::toMsg(box.getExtents(), msg.scale);
//         msg.pose.position = tf2::toMsg((Eigen::Vector3d)box.getPose().translation());
//         msg.pose.orientation = tf2::toMsg(Eigen::Quaterniond(box.getPose().linear()));

//         msg.color.g = 1.0;
//         msg.color.a = 0.5;
//         msg.type = visualization_msgs::msg::Marker::CUBE;
//         msg.action = visualization_msgs::msg::Marker::ADD;
//         msg.ns = "obbox/" + this->shapesToLinks.at(shapeHandle).cacheKey;
//         msg.frame_locked = static_cast<unsigned char>(true);

//         boundingBoxDebugMsg.markers.push_back(msg);
//       }
//     }

//     if (this->computeDebugOrientedBoundingBox) {
//       this->orientedBoundingBoxDebugMarkerPublisher->publish(boundingBoxDebugMsg);
//     }
//   }

//   if (this->computeOrientedBoundingBox) {
//     bodies::OrientedBoundingBox box(Eigen::Isometry3d::Identity(), Eigen::Vector3d::Zero());
//     // TODO: fix this
//     bodies::mergeBoundingBoxesApprox(boxes, box);

//     // robot_body_filter::OrientedBoundingBoxStamped boundingBoxMsg;

//     // boundingBoxMsg.header.stamp = scanTime;
//     // boundingBoxMsg.header.frame_id = this->filteringFrame;

//     // tf2::toMsg(box.getExtents(), boundingBoxMsg.obb.extents);
//     // tf2::toMsg(box.getPose().translation(),
//     //            boundingBoxMsg.obb.pose.translation);
//     // boundingBoxMsg.obb.pose.rotation =
//     //     tf2::toMsg(Eigen::Quaterniond(box.getPose().linear()));

//     // this->orientedBoundingBoxPublisher.publish(boundingBoxMsg);

//     if (this->publishOrientedBoundingBoxMarker) {
//       visualization_msgs::msg::Marker msg;
//       msg.header.stamp = scanTime;
//       msg.header.frame_id = this->filteringFrame;

//       tf2::toMsg(box.getExtents(), msg.scale);
//       msg.pose.position = tf2::toMsg((Eigen::Vector3d)box.getPose().translation());
//       msg.pose.orientation = tf2::toMsg(Eigen::Quaterniond(box.getPose().linear()));

//       msg.color.r = 1.0;
//       msg.color.a = 0.5;
//       msg.type = visualization_msgs::msg::Marker::CUBE;
//       msg.action = visualization_msgs::msg::Marker::ADD;
//       msg.ns = "oriented_bounding_box";
//       msg.frame_locked = static_cast<unsigned char>(true);

//       this->orientedBoundingBoxMarkerPublisher->publish(msg);
//     }

//     // compute and publish the scan_point_cloud with robot bounding box removed
//     if (this->publishNoOrientedBoundingBoxPointcloud) {
//       pcl::PCLPointCloud2::Ptr bboxCropInput(new pcl::PCLPointCloud2());
//       pcl_conversions::toPCL(projectedPointCloud, *(bboxCropInput));

//       robot_body_filter::CropBoxPointCloud2 cropBox;
//       cropBox.setNegative(true);
//       cropBox.setInputCloud(bboxCropInput);
//       cropBox.setKeepOrganized(this->keepCloudsOrganized);

//       const auto e = box.getExtents();
//       cropBox.setMin(Eigen::Vector4f(-e.x() / 2, -e.y() / 2, -e.z() / 2, 0.0));
//       cropBox.setMax(Eigen::Vector4f(e.x() / 2, e.y() / 2, e.z() / 2, 0.0));
//       cropBox.setTranslation(box.getPose().translation().cast<float>());
//       cropBox.setRotation(box.getPose().linear().eulerAngles(0, 1, 2).cast<float>());

//       pcl::PCLPointCloud2 pclOutput;
//       cropBox.filter(pclOutput);

//       sensor_msgs::msg::PointCloud2::SharedPtr boxFilteredCloud(new sensor_msgs::msg::PointCloud2());
//       pcl_conversions::moveFromPCL(pclOutput, *boxFilteredCloud);
//       boxFilteredCloud->header.stamp = scanTime;  // PCL strips precision of timestamp

//       this->scanPointCloudNoOrientedBoundingBoxPublisher->publish(*boxFilteredCloud);
//     }
//   }
// }

// template <typename T>
// void RobotBodyFilter<T>::computeAndPublishLocalBoundingBox(
//     const sensor_msgs::msg::PointCloud2& projectedPointCloud) const {
//   if (!this->computeLocalBoundingBox && !this->computeDebugLocalBoundingBox) return;

//   // assume this->modelMutex is locked

//   const auto& scanTime = projectedPointCloud.header.stamp;
//   std::string err;
//   try {
//     if (!this->tfBuffer->canTransform(this->localBoundingBoxFrame, this->filteringFrame, scanTime,
//                                       remainingTime(*this->nodeHandle->get_clock(), scanTime, this->reachableTransformTimeout), &err)) {
//       RCLCPP_ERROR(nodeHandle->get_logger(), "Cannot get transform %s->%s. Error is %s.", this->filteringFrame.c_str(),
//                    this->localBoundingBoxFrame.c_str(), err.c_str());
//       return;
//     }
//   } catch (tf2::TransformException& e) {
//     RCLCPP_ERROR(nodeHandle->get_logger(), "Cannot get transform %s->%s. Error is %s.", this->filteringFrame.c_str(),
//                  this->localBoundingBoxFrame.c_str(), e.what());
//     return;
//   }

//   const auto localTfMsg = this->tfBuffer->lookupTransform(this->localBoundingBoxFrame, this->filteringFrame, scanTime);
//   const Eigen::Isometry3d localTf = tf2::transformToEigen(localTfMsg.transform);

//   std::vector<bodies::AxisAlignedBoundingBox> boxes;

//   {
//     visualization_msgs::msg::MarkerArray boundingBoxDebugMsg;
//     for (const auto& shapeHandleAndBody : this->shapeMask->getBodiesForBoundingBox()) {
//       const auto& shapeHandle = shapeHandleAndBody.first;
//       const auto& body = shapeHandleAndBody.second;

//       if (this->shapesIgnoredInBoundingBox.find(shapeHandle) != this->shapesIgnoredInBoundingBox.end()) continue;

//       bodies::AxisAlignedBoundingBox box;
//       bodies::computeBoundingBoxAt(body, box, localTf * body->getPose());

//       boxes.push_back(box);

//       if (this->computeDebugLocalBoundingBox) {
//         visualization_msgs::msg::Marker msg;
//         msg.header.stamp = scanTime;
//         msg.header.frame_id = this->localBoundingBoxFrame;

//         tf2::toMsg(box.sizes(), msg.scale);
//         msg.pose.position = tf2::toMsg((Eigen::Vector3d)box.center());
//         msg.pose.orientation.w = 1;

//         msg.color.g = 1.0;
//         msg.color.a = 0.5;
//         msg.type = visualization_msgs::msg::Marker::CUBE;
//         msg.action = visualization_msgs::msg::Marker::ADD;
//         msg.ns = "lbbox/" + this->shapesToLinks.at(shapeHandle).cacheKey;
//         msg.frame_locked = static_cast<unsigned char>(true);

//         boundingBoxDebugMsg.markers.push_back(msg);
//       }
//     }

//     if (this->computeDebugLocalBoundingBox) {
//       this->localBoundingBoxDebugMarkerPublisher->publish(boundingBoxDebugMsg);
//     }
//   }

//   if (this->computeLocalBoundingBox) {
//     bodies::AxisAlignedBoundingBox box;
//     bodies::mergeBoundingBoxes(boxes, box);

//     geometry_msgs::msg::PolygonStamped boundingBoxMsg;

//     boundingBoxMsg.header.stamp = scanTime;
//     boundingBoxMsg.header.frame_id = this->localBoundingBoxFrame;

//     boundingBoxMsg.polygon.points.resize(2);
//     tf2::toMsg(box.min(), boundingBoxMsg.polygon.points[0]);
//     tf2::toMsg(box.max(), boundingBoxMsg.polygon.points[1]);

//     this->localBoundingBoxPublisher->publish(boundingBoxMsg);

//     if (this->publishLocalBoundingBoxMarker) {
//       visualization_msgs::msg::Marker msg;
//       msg.header.stamp = scanTime;
//       msg.header.frame_id = this->localBoundingBoxFrame;

//       tf2::toMsg(box.sizes(), msg.scale);
//       msg.pose.position = tf2::toMsg((Eigen::Vector3d)box.center());
//       msg.pose.orientation.w = 1;

//       msg.color.r = 1.0;
//       msg.color.a = 0.5;
//       msg.type = visualization_msgs::msg::Marker::CUBE;
//       msg.action = visualization_msgs::msg::Marker::ADD;
//       msg.ns = "local_bounding_box";
//       msg.frame_locked = static_cast<unsigned char>(true);

//       this->localBoundingBoxMarkerPublisher->publish(msg);
//     }

//     // compute and publish the scan_point_cloud with robot bounding box removed
//     if (this->publishNoLocalBoundingBoxPointcloud) {
//       pcl::PCLPointCloud2::Ptr bboxCropInput(new pcl::PCLPointCloud2());
//       pcl_conversions::toPCL(projectedPointCloud, *(bboxCropInput));

//       robot_body_filter::CropBoxPointCloud2 cropBox;
//       cropBox.setNegative(true);
//       cropBox.setInputCloud(bboxCropInput);
//       cropBox.setKeepOrganized(this->keepCloudsOrganized);

//       cropBox.setMin(Eigen::Vector4f(box.min()[0], box.min()[1], box.min()[2], 0.0));
//       cropBox.setMax(Eigen::Vector4f(box.max()[0], box.max()[1], box.max()[2], 0.0));
//       const Eigen::Isometry3d localTfInv = localTf.inverse();
//       cropBox.setTranslation(localTfInv.translation().cast<float>());
//       cropBox.setRotation(localTfInv.linear().eulerAngles(0, 1, 2).cast<float>());

//       pcl::PCLPointCloud2 pclOutput;
//       cropBox.filter(pclOutput);

//       sensor_msgs::msg::PointCloud2::SharedPtr boxFilteredCloud(new sensor_msgs::msg::PointCloud2());
//       pcl_conversions::moveFromPCL(pclOutput, *boxFilteredCloud);
//       boxFilteredCloud->header.stamp = scanTime;  // PCL strips precision of timestamp

//       this->scanPointCloudNoLocalBoundingBoxPublisher->publish(*boxFilteredCloud);
//     }
//   }
// }

template <typename T>
void RobotBodyFilter<T>::createBodyVisualizationMsg(
    const std::map<point_containment_filter::ShapeHandle, const bodies::Body *>& bodies, const rclcpp::Time& stamp,
    const std_msgs::msg::ColorRGBA& color, visualization_msgs::msg::MarkerArray& markerArray) const {
  // when computing the markers for publication, we want to publish them to the
  // time of the scan, so we need to set cacheLookupBetweenScansRatio again to
  // zero
  if (this->cacheLookupBetweenScansRatio != 0.0) {
    this->cacheLookupBetweenScansRatio = 0.0;
    this->shapeMask->updateBodyPoses();
  }

  for (const auto& shapeHandleAndBody : bodies) {
    const auto& shapeHandle = shapeHandleAndBody.first;
    auto body = shapeHandleAndBody.second;

    visualization_msgs::msg::Marker msg;
    bodies::constructMarkerFromBody(body, msg);

    msg.header.stamp = stamp;
    msg.header.frame_id = this->filteringFrame;

    msg.color = color;
    msg.action = visualization_msgs::msg::Marker::ADD;
    msg.ns = this->shapesToLinks.at(shapeHandle).cacheKey;
    msg.frame_locked = static_cast<unsigned char>(true);

    markerArray.markers.push_back(msg);
  }
}

// ROS1 requires this dynamic reconfig, I believe we can simply get parameter again in ROS2
template <typename T>
void RobotBodyFilter<T>::robotDescriptionUpdated(const std_msgs::msg::String::SharedPtr msg) {
  const std::string newConfig = msg->data;
  // robot_description parameter was not found, so we don't have to restart
  // the filter
  if (newConfig.empty()) return;

  auto urdf = newConfig;

  RCLCPP_INFO(nodeHandle->get_logger(),
              "RobotBodyFilter: Reloading robot model because of "
              "dynamic_reconfigure update. Filter operation stopped.");

  this->tfFramesWatchdog->pause();
  this->configured_ = false;

  RCLCPP_DEBUG(nodeHandle->get_logger(), "RobotBodyFilter: tfWatchdog paused and configured set to false.");

  this->clearRobotMask();

  RCLCPP_DEBUG(nodeHandle->get_logger(), "RobotBodyFilter: Robot mask cleared. Reloading robot model.");

  this->addRobotMaskFromUrdf(urdf);

  RCLCPP_DEBUG(nodeHandle->get_logger(), "RobotBodyFilter: Robot mask succesfully added from URDF");

  this->tfFramesWatchdog->unpause();
  this->timeConfigured = nodeHandle->now();
  this->configured_ = true;

  RCLCPP_DEBUG(nodeHandle->get_logger(), "RobotBodyFilter: Robot model reloaded, resuming filter operation.");
}

template <typename T>
bool RobotBodyFilter<T>::triggerModelReload(std_srvs::srv::Trigger_Request&, std_srvs::srv::Trigger_Response&) {
  std::string urdf;
  auto success = this->nodeHandle->get_parameter(this->robotDescriptionParam, urdf);

  if (!success) {
    RCLCPP_ERROR(nodeHandle->get_logger(), "RobotBodyFilter: Parameter %s doesn't exist.", this->robotDescriptionParam.c_str());
    return false;
  }

  RCLCPP_INFO(nodeHandle->get_logger(),
              "RobotBodyFilter: Reloading robot model because of trigger. Filter "
              "operation stopped.");

  this->tfFramesWatchdog->pause();
  this->configured_ = false;

  this->clearRobotMask();
  this->addRobotMaskFromUrdf(urdf);

  this->tfFramesWatchdog->unpause();
  this->timeConfigured = nodeHandle->now();
  this->configured_ = true;

  RCLCPP_INFO(nodeHandle->get_logger(), "RobotBodyFilter: Robot model reloaded, resuming filter operation.");
  return true;
}

template <typename T>
RobotBodyFilter<T>::~RobotBodyFilter() {
  if (this->tfFramesWatchdog != nullptr) this->tfFramesWatchdog->stop();
}

template <typename T>
ScaleAndPadding RobotBodyFilter<T>::getLinkInflationForContainsTest(const std::string& linkName) const {
  return this->getLinkInflationForContainsTest({linkName});
}

template <typename T>
ScaleAndPadding RobotBodyFilter<T>::getLinkInflationForContainsTest(const std::vector<std::string>& linkNames) const {
  return this->getLinkInflation(linkNames, this->defaultContainsInflation, this->perLinkContainsInflation);
}

template <typename T>
ScaleAndPadding RobotBodyFilter<T>::getLinkInflationForShadowTest(const std::string& linkName) const {
  return this->getLinkInflationForShadowTest({linkName});
}

template <typename T>
ScaleAndPadding RobotBodyFilter<T>::getLinkInflationForShadowTest(const std::vector<std::string>& linkNames) const {
  return this->getLinkInflation(linkNames, this->defaultShadowInflation, this->perLinkShadowInflation);
}

template <typename T>
ScaleAndPadding RobotBodyFilter<T>::getLinkInflationForBoundingSphere(const std::string& linkName) const {
  return this->getLinkInflationForBoundingSphere({linkName});
}

template <typename T>
ScaleAndPadding RobotBodyFilter<T>::getLinkInflationForBoundingSphere(const std::vector<std::string>& linkNames) const {
  return this->getLinkInflation(linkNames, this->defaultBsphereInflation, this->perLinkBsphereInflation);
}

template <typename T>
ScaleAndPadding RobotBodyFilter<T>::getLinkInflationForBoundingBox(const std::string& linkName) const {
  return this->getLinkInflationForBoundingBox({linkName});
}

template <typename T>
ScaleAndPadding RobotBodyFilter<T>::getLinkInflationForBoundingBox(const std::vector<std::string>& linkNames) const {
  return this->getLinkInflation(linkNames, this->defaultBboxInflation, this->perLinkBboxInflation);
}

template <typename T>
ScaleAndPadding RobotBodyFilter<T>::getLinkInflation(
    const std::vector<std::string>& linkNames, const ScaleAndPadding& defaultInflation,
    const std::map<std::string, ScaleAndPadding>& perLinkInflation) const {
  ScaleAndPadding result = defaultInflation;

  for (const auto& linkName : linkNames) {
    if (perLinkInflation.find(linkName) != perLinkInflation.end()) result = perLinkInflation.at(linkName);
  }

  return result;
}

ScaleAndPadding::ScaleAndPadding(double scale, double padding) : scale(scale), padding(padding) {}

bool ScaleAndPadding::operator==(const ScaleAndPadding& other) const {
  return this->scale == other.scale && this->padding == other.padding;
}

bool ScaleAndPadding::operator!=(const ScaleAndPadding& other) const { return !(*this == other); }

}  // namespace robot_body_filter

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(robot_body_filter::RobotBodyFilterLaserScan, filters::FilterBase<sensor_msgs::msg::LaserScan>)
PLUGINLIB_EXPORT_CLASS(robot_body_filter::RobotBodyFilterPointCloud2,
                       filters::FilterBase<sensor_msgs::msg::PointCloud2>)
