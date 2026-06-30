// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

/* HACK HACK HACK */
/* We want to access private members of some Node implementations. */
#include <sstream>  // has to be there, otherwise we encounter build problems
#define private public  // NOLINT
#include <rclcpp/node_interfaces/node_parameters.hpp>
#include <rclcpp/node_interfaces/node_clock.hpp>
#undef private
/* HACK END HACK */

#include <algorithm>
#include <chrono>
#include <ctime>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_set>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/filters/crop_box.h>
#include <pcl/PCLPointCloud2.h>

#include <cras_cpp_common/cloud.hpp>
#include <cras_cpp_common/set_utils.hpp>
#include <cras_cpp_common/string_utils.hpp>
#include <cras_cpp_common/tf2_sensor_msgs.hpp>
#include <cras_cpp_common/time_utils.hpp>
#include <cras_cpp_common/urdf_utils.hpp>
#include <geometric_shapes/bodies.h>
#include <geometric_shapes/body_operations.h>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <moveit/point_containment_filter/shape_mask.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/create_publisher.hpp>
#include <rclcpp/create_service.hpp>
#include <rclcpp/create_subscription.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher_options.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/subscription_options.hpp>
#include <rclcpp/time.hpp>
#include <rmw/rmw.h>
#include <robot_body_filter/RayCastingShapeMask.hpp>
#include <robot_body_filter/RobotBodyFilter.hpp>
#include <robot_body_filter/TfFramesWatchdog.hpp>
#include <robot_body_filter/utils/bodies.hpp>
#include <robot_body_filter/utils/shapes.hpp>
#include <robot_body_filter/utils/tf2_eigen.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/exceptions.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/buffer_interface.hpp>
#include <tf2_ros/transform_listener.hpp>
// #include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <urdf/model.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using std::max;
using std::min;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;
using std::string;
using std::vector;

namespace robot_body_filter {

template<typename T>
RobotBodyFilter<T>::RobotBodyFilter()
  : should_stop_(false), model_pose_update_interval_(0, 0), reachable_transform_timeout_(0, 0),
    unreachable_transform_timeout_(0, 0), tf_buffer_length_(0, 0) {
  model_mutex_.reset(new std::mutex());
  executor_.reset(new rclcpp::executors::SingleThreadedExecutor());
  executor_thread_ = std::make_unique<std::thread>([this] {
    while (!should_stop_) {
      executor_->spin_all(std::chrono::milliseconds(100));
    }
  });
}

template<typename T>
bool RobotBodyFilter<T>::configure() {
  node_interfaces_ = createNodeInterfaces();

  clock_ = node_interfaces_.get_node_clock_interface()->get_clock();

  tf_buffer_length_ = this->getParamDuration(
    "transforms.buffer_length", rclcpp::Duration::from_seconds(60.0), "s");

  fixed_frame_ = this->getParamVerbose("frames.fixed", "base_link");
  cras::stripLeadingSlash(fixed_frame_, true);
  sensor_frame_ = this->getParamVerbose("frames.sensor", "");
  cras::stripLeadingSlash(sensor_frame_, true);
  filtering_frame_ = this->getParamVerbose("frames.filtering", fixed_frame_);
  cras::stripLeadingSlash(filtering_frame_, true);
  min_distance_ = this->getParamVerbose("sensor.min_distance", 0.0, "m");
  max_distance_ = this->getParamVerbose("sensor.max_distance", 0.0, "m");
  robot_description_topic_ = this->getParamVerbose("body_model.robot_description_topic", "robot_description");
  keep_clouds_organized_ = this->getParamVerbose("filter.keep_clouds_organized", true);
  model_pose_update_interval_ = this->getParamDuration(
    "filter.model_pose_update_interval", rclcpp::Duration(0, 0), "s");
  const bool do_clipping = this->getParamVerbose("filter.do_clipping", true);
  const bool do_contains_test = this->getParamVerbose("filter.do_contains_test", true);
  const bool do_shadow_test = this->getParamVerbose("filter.do_shadow_test", true);
  const double max_shadow_distance = this->getParamVerbose("filter.max_shadow_distance", max_distance_, "m");
  reachable_transform_timeout_ = this->getParamDuration(
    "transforms.timeout.reachable", rclcpp::Duration::from_seconds(0.1), "s");
  unreachable_transform_timeout_ = this->getParamDuration(
    "transforms.timeout.unreachable", rclcpp::Duration::from_seconds(0.2), "s");
  require_all_frames_reachable_ = this->getParamVerbose("transforms.require_all_reachable", false);
  link_tf_prefix_ = this->getParamVerbose("transforms.link_tf_prefix", "");
  publish_no_bounding_sphere_pointcloud_ = this->getParamVerbose(
    "bounding_sphere.publish_cut_out_pointcloud", false);
  publish_no_bounding_box_pointcloud_ = this->getParamVerbose("bounding_box.publish_cut_out_pointcloud", false);
  publish_no_oriented_bounding_box_pointcloud_ = this->getParamVerbose(
    "oriented_bounding_box.publish_cut_out_pointcloud", false);
  publish_no_local_bounding_box_pointcloud_ = this->getParamVerbose(
    "local_bounding_box.publish_cut_out_pointcloud", false);
  compute_bounding_sphere_ = this->getParamVerbose("bounding_sphere.compute", false) ||
    publish_no_bounding_sphere_pointcloud_;
  compute_bounding_box_ = this->getParamVerbose("bounding_box.compute", false) ||
    publish_no_bounding_box_pointcloud_;
  compute_oriented_bounding_box_ = this->getParamVerbose("oriented_bounding_box.compute", false) ||
    publish_no_oriented_bounding_box_pointcloud_;
  compute_local_bounding_box_ = this->getParamVerbose("local_bounding_box.compute", false) ||
    publish_no_local_bounding_box_pointcloud_;
  compute_debug_bounding_sphere_ = this->getParamVerbose("bounding_sphere.debug", false);
  compute_debug_bounding_box_ = this->getParamVerbose("bounding_box.debug", false);
  compute_debug_oriented_bounding_box_ = this->getParamVerbose("oriented_bounding_box.debug", false);
  compute_debug_local_bounding_box_ = this->getParamVerbose("local_bounding_box.debug", false);
  publish_bounding_sphere_marker_ = this->getParamVerbose("bounding_sphere.marker", false);
  publish_bounding_box_marker_ = this->getParamVerbose("bounding_box.marker", false);
  publish_oriented_bounding_box_marker_ = this->getParamVerbose("oriented_bounding_box.marker", false);
  publish_local_bounding_box_marker_ = this->getParamVerbose("local_bounding_box.marker", false);
  local_bounding_box_frame_ = this->getParamVerbose("local_bounding_box.frame_id", fixed_frame_);
  publish_debug_pcl_inside_ = this->getParamVerbose("debug.pcl.inside", false);
  publish_debug_pcl_clip_ = this->getParamVerbose("debug.pcl.clip", false);
  publish_debug_pcl_shadow_ = this->getParamVerbose("debug.pcl.shadow", false);
  publish_debug_contains_marker_ = this->getParamVerbose("debug.marker.contains", false);
  publish_debug_shadow_marker_ = this->getParamVerbose("debug.marker.shadow", false);
  publish_debug_bsphere_marker_ = this->getParamVerbose("debug.marker.bounding_sphere", false);
  publish_debug_bbox_marker_ = this->getParamVerbose("debug.marker.bounding_box", false);

  const auto inflation_padding = this->getParamVerbose("body_model.inflation.padding", 0.0, "m");
  const auto inflation_scale = this->getParamVerbose("body_model.inflation.scale", 1.0);
  default_contains_inflation_.padding = this->getParamVerbose(
    "body_model.inflation.contains_test.padding", inflation_padding, "m");
  default_contains_inflation_.scale = this->getParamVerbose(
    "body_model.inflation.contains_test.scale", inflation_scale);
  default_shadow_inflation_.padding = this->getParamVerbose(
    "body_model.inflation.shadow_test.padding", inflation_padding, "m");
  default_shadow_inflation_.scale = this->getParamVerbose(
    "body_model.inflation.shadow_test.scale", inflation_scale);
  default_bsphere_inflation_.padding = this->getParamVerbose(
    "body_model.inflation.bounding_sphere.padding", inflation_padding, "m");
  default_bsphere_inflation_.scale = this->getParamVerbose(
    "body_model.inflation.bounding_sphere.scale", inflation_scale);
  default_bbox_inflation_.padding = this->getParamVerbose(
    "body_model.inflation.bounding_box.padding", inflation_padding, "m");
  default_bbox_inflation_.scale = this->getParamVerbose(
    "body_model.inflation.bounding_box.scale", inflation_scale);

  // read per-link padding
  const auto per_link_inflation_padding = this->getParamVerboseMap(
    "body_model.inflation.per_link.padding", std::map<std::string, double>(), "m");

  for (const auto& [linkNameConst, padding] : per_link_inflation_padding) {
    bool contains_only;
    bool shadow_only;
    bool bsphere_only;
    bool bbox_only;

    auto linkName = linkNameConst;
    linkName = cras::removeSuffix(linkName, kContainsSuffix, &contains_only);
    linkName = cras::removeSuffix(linkName, kShadowSuffix, &shadow_only);
    linkName = cras::removeSuffix(linkName, kBsphereSuffix, &bsphere_only);
    linkName = cras::removeSuffix(linkName, kBboxSuffix, &bbox_only);

    if (!shadow_only && !bsphere_only && !bbox_only) {
      per_link_contains_inflation_[linkName] = ScaleAndPadding(default_contains_inflation_.scale, padding);
    }
    if (!contains_only && !bsphere_only && !bbox_only) {
      per_link_shadow_inflation_[linkName] = ScaleAndPadding(default_shadow_inflation_.scale, padding);
    }
    if (!contains_only && !shadow_only && !bbox_only) {
      per_link_bsphere_inflation_[linkName] = ScaleAndPadding(default_bsphere_inflation_.scale, padding);
    }
    if (!contains_only && !shadow_only && !bsphere_only) {
      per_link_bbox_inflation_[linkName] = ScaleAndPadding(default_bbox_inflation_.scale, padding);
    }
  }

  // read per-link scale
  const auto per_link_inflation_scale = this->getParamVerboseMap(
    "body_model.inflation.per_link.scale", std::map<std::string, double>());
  for (const auto& [linkNameConst, inflation] : per_link_inflation_scale) {
    bool contains_only;
    bool shadow_only;
    bool bsphere_only;
    bool bbox_only;

    auto linkName = linkNameConst;
    linkName = cras::removeSuffix(linkName, kContainsSuffix, &contains_only);
    linkName = cras::removeSuffix(linkName, kShadowSuffix, &shadow_only);
    linkName = cras::removeSuffix(linkName, kBsphereSuffix, &bsphere_only);
    linkName = cras::removeSuffix(linkName, kBboxSuffix, &bbox_only);

    if (!shadow_only && !bsphere_only && !bbox_only) {
      if (per_link_contains_inflation_.find(linkName) == per_link_contains_inflation_.end()) {
        per_link_contains_inflation_[linkName] =
          ScaleAndPadding(inflation, default_contains_inflation_.padding);
      } else {
        per_link_contains_inflation_[linkName].scale = inflation;
      }
    }

    if (!contains_only && !bsphere_only && !bbox_only) {
      if (per_link_shadow_inflation_.find(linkName) == per_link_shadow_inflation_.end()) {
        per_link_shadow_inflation_[linkName] =
          ScaleAndPadding(inflation, default_shadow_inflation_.padding);
      } else {
        per_link_shadow_inflation_[linkName].scale = inflation;
      }
    }

    if (!contains_only && !shadow_only && !bbox_only) {
      if (per_link_bsphere_inflation_.find(linkName) == per_link_bsphere_inflation_.end()) {
        per_link_bsphere_inflation_[linkName] =
          ScaleAndPadding(inflation, default_bsphere_inflation_.padding);
      } else {
        per_link_bsphere_inflation_[linkName].scale = inflation;
      }
    }

    if (!contains_only && !shadow_only && !bsphere_only) {
      if (per_link_bbox_inflation_.find(linkName) == per_link_bbox_inflation_.end()) {
        per_link_bbox_inflation_[linkName] =
          ScaleAndPadding(inflation, default_bbox_inflation_.padding);
      } else {
        per_link_bbox_inflation_[linkName].scale = inflation;
      }
    }
  }

  // can contain either whole link names, or scoped names of their collisions
  // (i.e. "link::collision_1" or "link::my_collision")
  links_ignored_in_bounding_sphere_ = this->template getParamVerboseSet<string>("ignored_links.bounding_sphere");
  links_ignored_in_bounding_box_ = this->template getParamVerboseSet<string>("ignored_links.bounding_box");
  links_ignored_in_contains_test_ = this->template getParamVerboseSet<string>("ignored_links.contains_test");
  links_ignored_in_shadow_test_ = this->template getParamVerboseSet<string>("ignored_links.shadow_test", {"laser"});
  links_ignored_everywhere_ = this->template getParamVerboseSet<string>("ignored_links.everywhere");
  only_links_ = this->template getParamVerboseSet<string>("only_links");

  auto base = node_interfaces_.get_node_base_interface();
  auto logging = this->logging_interface_;
  auto params = this->params_interface_;
  auto services = node_interfaces_.get_node_services_interface();
  auto topics = node_interfaces_.get_node_topics_interface();

  const auto cbg = node_interfaces_.get_node_base_interface()->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  executor_->add_callback_group(cbg, node_interfaces_.get_node_base_interface());

  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = cbg;

  rclcpp::PublisherOptions pub_options;
  pub_options.callback_group = cbg;

  // initialize the TF buffer; do not use the created callback group - it spins its own thread with its own group
  if (tf_buffer_ == nullptr) {
    tf2::Duration tf2_duration = tf2_ros::fromRclcpp(tf_buffer_length_);
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(clock_, tf2_duration);
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(
      *tf_buffer_, base, logging, params, topics, true);
  } else {
    // clear the TF buffer (useful if calling configure() after receiving old TF data)
    tf_buffer_->clear();
  }

  // subscribe for robot_description
  reload_robot_model_subscriber_ = rclcpp::create_subscription<std_msgs::msg::String>(
    params, topics, robot_description_topic_, rclcpp::QoS(1).transient_local(),
    std::bind(&RobotBodyFilter<T>::onRobotModelMsg, this, _1), sub_options);

  // TODO enabling this callback breaks params queried later during configure(), like frames.output
  // param_cb_ = params_interface_->add_on_set_parameters_callback(
  //   std::bind(&RobotBodyFilter<T>::paramUpdateCallback, this, std::placeholders::_1));

  reload_robot_model_service_server_ = rclcpp::create_service<std_srvs::srv::Trigger>(
    base, services, this->getName() + "/reload_model",
    std::bind(&RobotBodyFilter<T>::triggerModelReload, this, _1, _2, _3), rclcpp::ServicesQoS(), cbg);

  if (compute_bounding_sphere_) {
    bounding_sphere_publisher_ = rclcpp::create_publisher<robot_body_filter::msg::SphereStamped>(
      params, topics, "robot_bounding_sphere", rclcpp::QoS(100), pub_options);
  }

  if (compute_bounding_box_) {
    bounding_box_publisher_ = rclcpp::create_publisher<geometry_msgs::msg::PolygonStamped>(
      params, topics, "robot_bounding_box", rclcpp::QoS(100), pub_options);
  }

  if (compute_oriented_bounding_box_) {
    oriented_bounding_box_publisher_ = rclcpp::create_publisher<msg::OrientedBoundingBoxStamped>(
      params, topics, "robot_oriented_bounding_box", rclcpp::QoS(100), pub_options);
  }

  if (compute_local_bounding_box_) {
    local_bounding_box_publisher_ = rclcpp::create_publisher<geometry_msgs::msg::PolygonStamped>(
      params, topics, "robot_local_bounding_box", rclcpp::QoS(100), pub_options);
  }

  if (publish_bounding_sphere_marker_ && compute_bounding_sphere_) {
    bounding_sphere_marker_publisher_ = rclcpp::create_publisher<visualization_msgs::msg::Marker>(
      params, topics, "robot_bounding_sphere_marker", rclcpp::QoS(100), pub_options);
  }

  if (publish_bounding_box_marker_ && compute_bounding_box_) {
    bounding_box_marker_publisher_ = rclcpp::create_publisher<visualization_msgs::msg::Marker>(
      params, topics, "robot_bounding_box_marker", rclcpp::QoS(100), pub_options);
  }

  if (publish_oriented_bounding_box_marker_ && compute_oriented_bounding_box_) {
    oriented_bounding_box_marker_publisher_ = rclcpp::create_publisher<visualization_msgs::msg::Marker>(
      params, topics, "robot_oriented_bounding_box_marker", rclcpp::QoS(100), pub_options);
  }

  if (publish_local_bounding_box_marker_ && compute_local_bounding_box_) {
    local_bounding_box_marker_publisher_ = rclcpp::create_publisher<visualization_msgs::msg::Marker>(
      params, topics, "robot_local_bounding_box_marker", rclcpp::QoS(100), pub_options);
  }

  if (publish_no_bounding_box_pointcloud_) {
    scan_point_cloud_no_bounding_box_publisher_ = rclcpp::create_publisher<sensor_msgs::msg::PointCloud2>(
      params, topics, "scan_point_cloud_no_bbox", rclcpp::QoS(100), pub_options);
  }

  if (publish_no_oriented_bounding_box_pointcloud_) {
    scan_point_cloud_no_oriented_bounding_box_publisher_ =
      rclcpp::create_publisher<sensor_msgs::msg::PointCloud2>(
        params, topics, "scan_point_cloud_no_oriented_bbox", rclcpp::QoS(100), pub_options);
  }

  if (publish_no_local_bounding_box_pointcloud_) {
    scan_point_cloud_no_local_bounding_box_publisher_ =
      rclcpp::create_publisher<sensor_msgs::msg::PointCloud2>(
        params, topics, "scan_point_cloud_no_local_bbox", rclcpp::QoS(100), pub_options);
  }

  if (publish_no_bounding_sphere_pointcloud_) {
    scan_point_cloud_no_bounding_sphere_publisher_ =
      rclcpp::create_publisher<sensor_msgs::msg::PointCloud2>(
        params, topics, "scan_point_cloud_no_bsphere", rclcpp::QoS(100), pub_options);
  }

  if (publish_debug_pcl_inside_) {
    debug_point_cloud_inside_publisher_ = rclcpp::create_publisher<sensor_msgs::msg::PointCloud2>(
      params, topics, "scan_point_cloud_inside", rclcpp::QoS(100), pub_options);
  }

  if (publish_debug_pcl_clip_) {
    debug_point_cloud_clip_publisher_ = rclcpp::create_publisher<sensor_msgs::msg::PointCloud2>(
      params, topics, "scan_point_cloud_clip", rclcpp::QoS(100), pub_options);
  }

  if (publish_debug_pcl_shadow_) {
    debug_point_cloud_shadow_publisher_ = rclcpp::create_publisher<sensor_msgs::msg::PointCloud2>(
      params, topics, "scan_point_cloud_shadow", rclcpp::QoS(100), pub_options);
  }

  if (publish_debug_contains_marker_) {
    debug_contains_marker_publisher_ = rclcpp::create_publisher<visualization_msgs::msg::MarkerArray>(
      params, topics, "robot_model_for_contains_test", rclcpp::QoS(100), pub_options);
  }

  if (publish_debug_shadow_marker_) {
    debug_shadow_marker_publisher_ = rclcpp::create_publisher<visualization_msgs::msg::MarkerArray>(
      params, topics, "robot_model_for_shadow_test", rclcpp::QoS(100), pub_options);
  }

  if (publish_debug_bsphere_marker_) {
    debug_bsphere_marker_publisher_ = rclcpp::create_publisher<visualization_msgs::msg::MarkerArray>(
      params, topics, "robot_model_for_bounding_sphere", rclcpp::QoS(100), pub_options);
  }

  if (publish_debug_bbox_marker_) {
    debug_bbox_marker_publisher_ = rclcpp::create_publisher<visualization_msgs::msg::MarkerArray>(
      params, topics, "robot_model_for_bounding_box", rclcpp::QoS(100), pub_options);
  }

  if (compute_debug_bounding_box_) {
    bounding_box_debug_marker_publisher_ = rclcpp::create_publisher<visualization_msgs::msg::MarkerArray>(
      params, topics, "robot_bounding_box_debug", rclcpp::QoS(100), pub_options);
  }

  if (compute_debug_oriented_bounding_box_) {
    oriented_bounding_box_debug_marker_publisher_ =
      rclcpp::create_publisher<visualization_msgs::msg::MarkerArray>(
        params, topics, "robot_oriented_bounding_box_debug", rclcpp::QoS(100), pub_options);
  }

  if (compute_debug_local_bounding_box_) {
    local_bounding_box_debug_marker_publisher_ =
      rclcpp::create_publisher<visualization_msgs::msg::MarkerArray>(
        params, topics, "robot_local_bounding_box_debug", rclcpp::QoS(100), pub_options);
  }

  if (compute_debug_bounding_sphere_) {
    bounding_sphere_debug_marker_publisher_ =
      rclcpp::create_publisher<visualization_msgs::msg::MarkerArray>(
        params, topics, "robot_bounding_sphere_debug", rclcpp::QoS(100), pub_options);
  }

  // initialize the 3D body masking tool
  auto get_shape_transform_callback = std::bind(&RobotBodyFilter::getShapeTransform, this, _1, _2);
  shape_mask_ = std::make_unique<RayCastingShapeMask>(
    get_logger(), clock_, get_shape_transform_callback, min_distance_, max_distance_,
    do_clipping, do_contains_test, do_shadow_test, max_shadow_distance);

  // the other case happens when configure() is called again from update() (e.g. when a new bag file
  // started playing)
  if (tf_frames_watchdog_ == nullptr) {
    std::set<std::string> initial_monitored_frames;
    if (!sensor_frame_.empty()) {
      initial_monitored_frames.insert(sensor_frame_);
    }

    tf_frames_watchdog_ = std::make_shared<TFFramesWatchdog>(
      get_logger(), clock_, filtering_frame_, initial_monitored_frames, tf_buffer_,
      unreachable_transform_timeout_, std::make_shared<rclcpp::Rate>(1.0, clock_));
    tf_frames_watchdog_->start();
  }

  // happens when configure() is called again from update() (e.g. when a new bag file started playing)
  if (!shapes_to_links_.empty() && hasModel()) {
    clearRobotMask();
    addRobotMaskFromUrdf(robot_description_string_);
  }

  RCLCPP_INFO(get_logger(), "RobotBodyFilter: Successfully configured.");
  RCLCPP_INFO(get_logger(), "Filtering data in frame %s", filtering_frame_.c_str());
  RCLCPP_INFO(get_logger(), "RobotBodyFilter: Filtering into the following categories:");
  RCLCPP_INFO(get_logger(), "RobotBodyFilter: \tOUTSIDE");
  if (do_clipping) {
    RCLCPP_INFO(get_logger(), "RobotBodyFilter: \tCLIP");
  }
  if (do_contains_test) {
    RCLCPP_INFO(get_logger(), "RobotBodyFilter: \tINSIDE");
  }
  if (do_shadow_test) {
    RCLCPP_INFO(get_logger(), "RobotBodyFilter: \tSHADOW");
  }

  if (only_links_.empty()) {
    if (links_ignored_everywhere_.empty()) {
      RCLCPP_INFO(get_logger(), "RobotBodyFilter: Filtering applied to all links.");
    } else {
      RCLCPP_INFO(
        get_logger(), "RobotBodyFilter: Filtering applied to all links except %s.",
        cras::to_string(links_ignored_everywhere_).c_str());
    }
  } else {
    if (links_ignored_everywhere_.empty()) {
      RCLCPP_INFO(
        get_logger(), "RobotBodyFilter: Filtering applied to links %s.", cras::to_string(only_links_).c_str());
    } else {
      RCLCPP_INFO(
        get_logger(), "RobotBodyFilter: Filtering applied to links %s with these links excluded: %s.",
        cras::to_string(only_links_).c_str(), cras::to_string(links_ignored_everywhere_).c_str());
    }
  }

  time_configured_ = clock_->now();

  return true;
}

bool RobotBodyFilterLaserScan::configure() {
  point_by_point_scan_ = getParamVerbose("sensor.point_by_point", true);

  return RobotBodyFilter::configure();
}

bool RobotBodyFilterPointCloud2::configure() {
  point_by_point_scan_ = getParamVerbose("sensor.point_by_point", false);

  if (!RobotBodyFilter::configure()) {
    return false;
  }

  output_frame_ = getParamVerbose("frames.output", filtering_frame_);

  const auto point_channels = getParamVerbose("cloud.point_channels", std::vector<std::string>{"", "vp_"});
  const auto direction_channels = getParamVerbose(
    "cloud.direction_channels", std::vector<std::string>{"normal_"});

  for (const auto& channel : point_channels) {
    channels_to_transform_[channel] = cras::CloudChannelType::POINT;
  }
  for (const auto& channel : direction_channels) {
    channels_to_transform_[channel] = cras::CloudChannelType::DIRECTION;
  }

  cras::stripLeadingSlash(output_frame_, true);

  return true;
}

template<typename T>
bool RobotBodyFilter<T>::computeMask(
  const sensor_msgs::msg::PointCloud2& projected_point_cloud, std::vector<RayCastingShapeMask::MaskValue>& point_mask,
  const std::string& sensor_frame) {
  // model_mutex_ has to be already locked!

  const clock_t stopwatch_overall = clock();
  const auto& scan_time = projected_point_cloud.header.stamp;

  // compute a mask of point indices for points from projected_point_cloud
  // that tells if they are inside or outside robot, or shadow points

  if (!point_by_point_scan_) {
    Eigen::Vector3d sensor_pos;
    try {
      const auto sensor_tf = tf_buffer_->lookupTransform(
        filtering_frame_, sensor_frame, scan_time,
        cras::remainingTime(scan_time, reachable_transform_timeout_, clock_));
      tf2::fromMsg(sensor_tf.transform.translation, sensor_pos);
    } catch (tf2::TransformException& e) {
      RCLCPP_ERROR(
        get_logger(), "RobotBodyFilter: Could not compute filtering mask due to this TF exception: %s", e.what());
      return false;
    }

    // update transforms cache, which is then used in body masking
    updateTransformCache(scan_time);

    // updates shapes according to tf cache (by calling getShapeTransform
    // for each shape) and masks contained points
    shape_mask_->maskContainmentAndShadows(projected_point_cloud, point_mask, sensor_pos);
  } else {
    cras::CloudConstIter x_it(projected_point_cloud, "x");
    cras::CloudConstIter y_it(projected_point_cloud, "y");
    cras::CloudConstIter z_it(projected_point_cloud, "z");
    cras::CloudConstIter vp_x_it(projected_point_cloud, "vp_x");
    cras::CloudConstIter vp_y_it(projected_point_cloud, "vp_y");
    cras::CloudConstIter vp_z_it(projected_point_cloud, "vp_z");
    cras::CloudConstIter stamps_it(projected_point_cloud, "stamps");

    point_mask.resize(cras::numPoints(projected_point_cloud));

    double scan_duration = 0.0;
    for (cras::CloudConstIter stamps_end_it(projected_point_cloud, "stamps"); stamps_end_it != stamps_end_it.end();
         ++stamps_end_it) {
      if (*stamps_end_it > static_cast<float>(scan_duration)) {
        scan_duration = static_cast<double>(*stamps_end_it);
      }
    }
    const rclcpp::Time after_scan_time(rclcpp::Time(scan_time) + rclcpp::Duration::from_seconds(scan_duration));

    size_t update_body_poses_every;
    if (model_pose_update_interval_.seconds() == 0 && model_pose_update_interval_.nanoseconds() == 0) {
      update_body_poses_every = 1;
    } else {
      update_body_poses_every = static_cast<size_t>(
        ceil(model_pose_update_interval_.seconds() / scan_duration * cras::numPoints(projected_point_cloud)));
      // prevent division by zero
      if (update_body_poses_every == 0) {
        update_body_poses_every = 1;
      }
    }

    // prevent division by zero in ratio computation in case the pointcloud
    // isn't really taken point by point with different timestamps
    if (scan_duration == 0.0) {
      update_body_poses_every = cras::numPoints(projected_point_cloud) + 1;
      RCLCPP_WARN_ONCE(
        get_logger(), "RobotBodyFilter: sensor/point_by_point is set to true but all points in the cloud have the same "
        "timestamp. You should change the parameter to false to gain performance.");
    }

    // update transforms cache, which is then used in body masking
    updateTransformCache(scan_time, after_scan_time);

    Eigen::Vector3f point;
    Eigen::Vector3d view_point;
    RayCastingShapeMask::MaskValue mask;

    cache_lookup_between_scans_ratio_ = 0.0;
    for (size_t i = 0; i < cras::numPoints(projected_point_cloud);
         ++i, ++x_it, ++y_it, ++z_it, ++vp_x_it, ++vp_y_it, ++vp_z_it, ++stamps_it) {
      point.x() = *x_it;
      point.y() = *y_it;
      point.z() = *z_it;

      // TODO viewpoint can be autocomputed from stamps
      view_point.x() = static_cast<double>(*vp_x_it);
      view_point.y() = static_cast<double>(*vp_y_it);
      view_point.z() = static_cast<double>(*vp_z_it);

      const auto update_body_poses = i % update_body_poses_every == 0;

      if (update_body_poses && scan_duration > 0.0) {
        cache_lookup_between_scans_ratio_ = static_cast<double>(*stamps_it) / scan_duration;
      }

      // updates shapes according to tf cache (by calling getShapeTransform
      // for each shape) and masks contained points
      shape_mask_->maskContainmentAndShadows(point, mask, view_point, update_body_poses);
      point_mask[i] = mask;
    }
  }

  RCLCPP_DEBUG(
    get_logger(), "RobotBodyFilter: Mask computed in %.5f secs.",
    static_cast<double>(clock() - stopwatch_overall) / CLOCKS_PER_SEC);

  publishDebugPointClouds(projected_point_cloud, point_mask);
  publishDebugMarkers(scan_time);
  computeAndPublishBoundingSphere(projected_point_cloud);
  computeAndPublishBoundingBox(projected_point_cloud);
  computeAndPublishOrientedBoundingBox(projected_point_cloud);
  computeAndPublishLocalBoundingBox(projected_point_cloud);

  RCLCPP_DEBUG(
    get_logger(), "RobotBodyFilter: Filtering run time is %.5f secs.",
    static_cast<double>(clock() - stopwatch_overall) / CLOCKS_PER_SEC);
  return true;
}

bool RobotBodyFilterLaserScan::update(
    const sensor_msgs::msg::LaserScan& input_scan, sensor_msgs::msg::LaserScan& filtered_scan) {
  if (should_stop_) {
    return false;
  }

  const auto& scan_time = rclcpp::Time(input_scan.header.stamp);

  if (!configured_) {
    RCLCPP_DEBUG(
      get_logger(), "RobotBodyFilter: Ignore scan from time %f.%ld - filter not yet initialized.",
      scan_time.seconds(), scan_time.nanoseconds());
    return false;
  }

  if (scan_time < time_configured_ && (scan_time + tf_buffer_length_) >= time_configured_) {
    RCLCPP_DEBUG(
      get_logger(), "RobotBodyFilter: Ignore scan from time %f.%ld - filter not yet initialized.",
      scan_time.seconds(), scan_time.nanoseconds());
    return false;
  }

  if (scan_time < time_configured_ && (scan_time + tf_buffer_length_) < time_configured_) {
    RCLCPP_WARN(
      get_logger(), "RobotBodyFilter: Old TF data received. Clearing TF buffer and reconfiguring laser filter. If "
      "you're replaying a bag file, make sure rosparam /use_sim_time is set to true");
    configure();
    return false;
  }

  // tf2 doesn't like frames starting with slash
  const auto scan_frame = cras::stripLeadingSlash(input_scan.header.frame_id, true);

  // Passing a sensor_frame_ does not make sense. Scan messages can't be transformed to other frames.
  if (!sensor_frame_.empty() && sensor_frame_ != scan_frame) {
    RCLCPP_WARN_ONCE(
      get_logger(), "RobotBodyFilter: frames/sensor is set to frame_id '%s' different than the frame_id of the "
      "incoming message '%s'. This is an invalid configuration: the frames/sensor parameter will be neglected.",
      sensor_frame_.c_str(), scan_frame.c_str());
  }

  if (!tf_frames_watchdog_->isReachable(scan_frame)) {
    RCLCPP_DEBUG(get_logger(), "RobotBodyFilter: Throwing away scan since sensor frame is unreachable.");
    // if sensor_frame_ is empty, it can happen that we're not actually monitoring the sensor
    // frame, so start monitoring it
    if (!tf_frames_watchdog_->isMonitored(scan_frame)) {
      tf_frames_watchdog_->addMonitoredFrame(scan_frame);
    }
    return false;
  }

  if (require_all_frames_reachable_ && !tf_frames_watchdog_->areAllFramesReachable()) {
    RCLCPP_DEBUG(get_logger(), "RobotBodyFilter: Throwing away scan since not all frames are reachable.");
    return false;
  }

  const clock_t stopwatch_overall = clock();

  // create the output copy of the input scan
  filtered_scan = input_scan;
  filtered_scan.header.frame_id = scan_frame;
  filtered_scan.range_min = max(input_scan.range_min, static_cast<float>(min_distance_));
  if (max_distance_ > 0.0) {
    filtered_scan.range_max = min(input_scan.range_max, static_cast<float>(max_distance_));
  }

  {  // acquire the lock here, because we work with the tf_buffer_ all the time
    std::lock_guard<std::mutex> guard(*model_mutex_);

    if (point_by_point_scan_) {
      // make sure we have all the tfs between sensor frame and fixed_frame_ during the time of scan acquisition
      const auto scan_duration = input_scan.ranges.size() * input_scan.time_increment;
      const auto after_scan_time = scan_time + rclcpp::Duration::from_seconds(scan_duration);

      string err;
      if (!tf_buffer_->canTransform(
            fixed_frame_, scan_frame, scan_time,
            cras::remainingTime(scan_time, reachable_transform_timeout_, clock_), &err) ||
          !tf_buffer_->canTransform(
            fixed_frame_, scan_frame, after_scan_time,
            cras::remainingTime(after_scan_time, reachable_transform_timeout_, clock_), &err)) {
        if (err.find("future") != string::npos) {
          const auto delay = clock_->now() - scan_time;
          RCLCPP_ERROR_THROTTLE(
            get_logger(), *clock_, 3,
            "RobotBodyFilter: Cannot transform laser scan to fixed frame. The scan is too much delayed (%s s). "
            "TF error: %s", cras::to_string(delay).c_str(), err.c_str());
        } else {
          // TODO: Originally was delayed-throttle
          RCLCPP_ERROR_THROTTLE(
            get_logger(), *clock_, 3,
            "RobotBodyFilter: Cannot transform laser scan to fixed frame. Something's wrong with TFs: %s", err.c_str());
        }
        return false;
      }
    }

    // The point cloud will have fields x, y, z, intensity (float32) and index (int32)
    // and for point-by-point scans also timestamp and viewpoint
    sensor_msgs::msg::PointCloud2 projected_point_cloud;
    {
      // project the scan measurements to a point cloud in the filtering_frame_
      sensor_msgs::msg::PointCloud2 tmp_point_cloud;

      // the projected point cloud can omit some measurements if they are out of the defined scan's range;
      // for this case, the second channel ("index") contains indices of the point cloud's points into the scan
      auto channel_options =
        laser_geometry::channel_option::Intensity |
        laser_geometry::channel_option::Index;

      if (point_by_point_scan_) {
        RCLCPP_INFO_ONCE(get_logger(), "RobotBodyFilter: Applying complex laser scan projection.");
        // perform the complex laser scan projection
        channel_options |=
          laser_geometry::channel_option::Timestamp |
          laser_geometry::channel_option::Viewpoint;

        laser_projector_.transformLaserScanToPointCloud(
          fixed_frame_, input_scan, tmp_point_cloud, *tf_buffer_, -1, channel_options);
      } else {
        RCLCPP_INFO_ONCE(get_logger(), "RobotBodyFilter: Applying simple laser scan projection.");
        // perform simple laser scan projection
        laser_projector_.projectLaser(input_scan, tmp_point_cloud, -1.0, channel_options);
      }

      // convert to filtering frame
      if (tmp_point_cloud.header.frame_id == filtering_frame_) {
        projected_point_cloud = std::move(tmp_point_cloud);
      } else {
        RCLCPP_INFO_ONCE(
          get_logger(), "RobotBodyFilter: Transforming scan from frame %s to %s",
          tmp_point_cloud.header.frame_id.c_str(), filtering_frame_.c_str());
        std::string err;
        if (!tf_buffer_->canTransform(
          filtering_frame_, tmp_point_cloud.header.frame_id, scan_time,
          cras::remainingTime(scan_time, reachable_transform_timeout_, clock_), &err)) {
          // TODO: originally was delayed-throttle
          RCLCPP_ERROR_THROTTLE(
            get_logger(), *clock_, 3,
            "RobotBodyFilter: Cannot transform laser scan to filtering frame. Something's wrong with TFs: %s",
            err.c_str());
          return false;
        }

        cras::transformWithChannels(
          tmp_point_cloud, projected_point_cloud, *tf_buffer_, filtering_frame_,
          channels_to_transform_);
      }
    }

    RCLCPP_DEBUG(
      get_logger(), "RobotBodyFilter: Scan transformation run time is %.5f secs.",
      static_cast<double>(clock() - stopwatch_overall) / CLOCKS_PER_SEC);

    vector<RayCastingShapeMask::MaskValue> point_mask;
    const auto success = computeMask(projected_point_cloud, point_mask, scan_frame);

    if (!success) {
      return false;
    }

    {  // remove invalid points
      constexpr float INVALID_POINT_VALUE = std::numeric_limits<float>::quiet_NaN();
      try {
        sensor_msgs::PointCloud2Iterator<int> index_it(projected_point_cloud, "index");

        size_t index_in_scan;
        for (const auto mask_value : point_mask) {
          switch (mask_value) {
            case RayCastingShapeMask::MaskValue::INSIDE:
            case RayCastingShapeMask::MaskValue::SHADOW:
            case RayCastingShapeMask::MaskValue::CLIP:
              index_in_scan = static_cast<size_t>(*index_it);
              filtered_scan.ranges[index_in_scan] = INVALID_POINT_VALUE;
              break;
            case RayCastingShapeMask::MaskValue::OUTSIDE:
              break;
          }
          ++index_it;
        }
      } catch (std::runtime_error&) {
        RCLCPP_ERROR(
          get_logger(), "RobotBodyFilter: projectedPointCloud doesn't have field called 'index', but the algorithm "
          "relies on that.");
        return false;
      }
    }
  }

  return true;
}

bool RobotBodyFilterPointCloud2::update(
    const sensor_msgs::msg::PointCloud2& input_cloud, sensor_msgs::msg::PointCloud2& filtered_cloud) {
  const auto& scan_time = rclcpp::Time(input_cloud.header.stamp);

  if (should_stop_) {
    return false;
  }

  if (!configured_) {
    RCLCPP_DEBUG(
      get_logger(), "RobotBodyFilter: Ignore cloud from time %f.%ld - filter not yet initialized.",
      scan_time.seconds(), scan_time.nanoseconds());
    return false;
  }

  if (scan_time < time_configured_ && (scan_time + tf_buffer_length_ >= time_configured_)) {
    RCLCPP_DEBUG(
      get_logger(), "RobotBodyFilter: Ignore cloud from time %f.%ld - filter not yet initialized.",
      scan_time.seconds(), scan_time.nanoseconds());
    return false;
  }

  if (scan_time < time_configured_ && (scan_time + tf_buffer_length_) < time_configured_) {
    RCLCPP_WARN(
      get_logger(), "RobotBodyFilter: Old TF data received. Clearing TF buffer and reconfiguring laser filter. "
      "If you're replaying a bag file, make sure rosparam /use_sim_time is set to true");
    configure();
    return false;
  }

  const auto input_cloud_frame =
    sensor_frame_.empty() ? cras::stripLeadingSlash(input_cloud.header.frame_id, true) : sensor_frame_;

  if (!tf_frames_watchdog_->isReachable(input_cloud_frame)) {
    RCLCPP_DEBUG(get_logger(), "RobotBodyFilter: Throwing away scan since sensor frame is unreachable.");
    // if sensor_frame_ is empty, it can happen that we're not actually monitoring the cloud
    // frame, so start monitoring it
    if (!tf_frames_watchdog_->isMonitored(input_cloud_frame)) {
      tf_frames_watchdog_->addMonitoredFrame(input_cloud_frame);
    }
    return false;
  }

  if (require_all_frames_reachable_ && !tf_frames_watchdog_->areAllFramesReachable()) {
    RCLCPP_DEBUG(get_logger(), "RobotBodyFilter: Throwing away scan since not all frames are reachable.");
    return false;
  }

  bool has_stamps_field = false;
  bool has_vp_x_field = false, has_vp_y_field = false, has_vp_z_field = false;
  for (const auto& field : input_cloud.fields) {
    if (field.name == "stamps" && field.datatype == sensor_msgs::msg::PointField::FLOAT32) {
      has_stamps_field = true;
    } else if (field.name == "vp_x" && field.datatype == sensor_msgs::msg::PointField::FLOAT32) {
      has_vp_x_field = true;
    } else if (field.name == "vp_y" && field.datatype == sensor_msgs::msg::PointField::FLOAT32) {
      has_vp_y_field = true;
    } else if (field.name == "vp_z" && field.datatype == sensor_msgs::msg::PointField::FLOAT32) {
      has_vp_z_field = true;
    }
  }

  // Verify the pointcloud and its fields

  if (point_by_point_scan_) {
    if (input_cloud.height != 1 && input_cloud.is_dense == 0) {
      RCLCPP_WARN_ONCE(
        get_logger(), "RobotBodyFilter: The pointcloud seems to be an organized pointcloud, which usually means it was "
        "captured all at once. Consider setting 'point_by_point_scan' to false to get a more efficient computation.");
    }
    if (!has_stamps_field || !has_vp_x_field || !has_vp_y_field || !has_vp_z_field) {
      throw std::runtime_error(
        "A point-by-point scan has to contain float32 fields 'stamps', 'vp_x', 'vp_y' and 'vp_z'.");
    }
  } else if (has_stamps_field) {
    RCLCPP_WARN_ONCE(
      get_logger(), "RobotBodyFilter: The pointcloud has a 'stamps' field, which indicates each point was probably "
      "captured at a different time instant. Consider setting parameter 'point_by_point_scan' to true to get correct "
      "results.");
  } else if (input_cloud.height == 1 && input_cloud.is_dense == 1) {
    RCLCPP_WARN_ONCE(
      get_logger(), "RobotBodyFilter: The pointcloud is dense, which usually means it was captured each point at a "
      "different time instant. Consider setting 'point_by_point_scan' to true to get a more accurate version.");
  }

  // Transform to filtering frame

  sensor_msgs::msg::PointCloud2 transformed_cloud;
  if (input_cloud.header.frame_id == filtering_frame_) {
    transformed_cloud = input_cloud;
  } else {
    RCLCPP_INFO_ONCE(
      get_logger(), "RobotBodyFilter: Transforming cloud from frame %s to %s",
      input_cloud.header.frame_id.c_str(), filtering_frame_.c_str());
    std::lock_guard<std::mutex> guard(*model_mutex_);
    std::string err;
    if (!tf_buffer_->canTransform(
      filtering_frame_, input_cloud.header.frame_id, scan_time,
      cras::remainingTime(scan_time, reachable_transform_timeout_, clock_), &err)) {
      // TODO: Originally was delayed-throttle
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *clock_, 3,
        "RobotBodyFilter: Cannot transform point cloud to filtering frame. Something's wrong with TFs: %s",
        err.c_str());
      return false;
    }

    cras::transformWithChannels(
      input_cloud, transformed_cloud, *tf_buffer_, filtering_frame_, channels_to_transform_);
  }

  // Compute the mask and use it (transform message only if sensor_frame_ is specified)
  vector<RayCastingShapeMask::MaskValue> point_mask;
  {
    std::lock_guard<std::mutex> guard(*model_mutex_);

    const auto success = computeMask(transformed_cloud, point_mask, input_cloud_frame);
    if (!success) {
      return false;
    }
  }

  // Filter the cloud

  sensor_msgs::msg::PointCloud2 tmp_cloud;
  CREATE_FILTERED_CLOUD(
    transformed_cloud, tmp_cloud, keep_clouds_organized_,
    (point_mask[i] == RayCastingShapeMask::MaskValue::OUTSIDE))

  // Transform to output frame

  if (tmp_cloud.header.frame_id == output_frame_) {
    filtered_cloud = std::move(tmp_cloud);
  } else {
    RCLCPP_INFO_ONCE(
      get_logger(), "RobotBodyFilter: Transforming cloud from frame %s to %s",
      tmp_cloud.header.frame_id.c_str(), output_frame_.c_str());
    std::lock_guard<std::mutex> guard(*model_mutex_);
    std::string err;
    if (!tf_buffer_->canTransform(
      output_frame_, tmp_cloud.header.frame_id, scan_time,
      cras::remainingTime(scan_time, reachable_transform_timeout_, clock_), &err)) {
      // TODO: Originally was delayed-throttle
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *clock_, 3,
        "RobotBodyFilter: Cannot transform point cloud to output frame. Something's wrong with TFs: %s", err.c_str());
      return false;
    }

    cras::transformWithChannels(
      tmp_cloud, filtered_cloud, *tf_buffer_, output_frame_, channels_to_transform_);
  }

  return true;
}

template<typename T>
bool RobotBodyFilter<T>::getShapeTransform(
  const point_containment_filter::ShapeHandle shape_handle, Eigen::Isometry3d& transform) const {
  // make sure you locked model_mutex_

  const auto collision_it = shapes_to_links_.find(shape_handle);
  // check if the given shape_handle has been registered to a link during addRobotMaskFromUrdf call.
  if (collision_it == shapes_to_links_.end()) {
    RCLCPP_ERROR_STREAM_THROTTLE(
      get_logger(), *clock_, 3, "RobotBodyFilter: Invalid shape handle: " << cras::to_string(shape_handle));
    return false;
  }

  const auto& collision = collision_it->second;

  const auto transform_it = transform_cache_.find(collision.cache_key);
  if (transform_it == transform_cache_.end()) {
    // do not log the error because shape mask would do it for us
    return false;
  }

  if (!point_by_point_scan_) {
    transform = *transform_it->second;
  } else {
    if (transform_cache_after_scan_.find(collision.cache_key) == transform_cache_after_scan_.end()) {
      // do not log the error because shape mask would do it for us
      return false;
    }

    const auto& tf1 = *transform_cache_.at(collision.cache_key);
    const auto& tf2 = *transform_cache_after_scan_.at(collision.cache_key);
    const Eigen::Quaterniond quat1(tf1.rotation().matrix());
    const Eigen::Quaterniond quat2(tf1.rotation().matrix());
    const auto r = cache_lookup_between_scans_ratio_;

    transform.translation() = tf1.translation() * (1 - r) + tf2.translation() * r;
    const Eigen::Quaterniond quat3 = quat1.slerp(r, quat2);
    transform.linear() = quat3.toRotationMatrix();
  }

  return true;
}

template<typename T>
void RobotBodyFilter<T>::updateTransformCache(const rclcpp::Time& time, const rclcpp::Time& after_scan_time) {
  // make sure you locked model_mutex_

  // clear the cache so that maskContainment always uses only these tf data and not some older
  transform_cache_.clear();
  if (after_scan_time.seconds() != 0) {
    transform_cache_after_scan_.clear();
  }

  // iterate over all links corresponding to some masking shape and update their cached transforms relative
  // to fixed_frame
  for (const auto& [shape, collision_body] : shapes_to_links_) {
    const auto& collision = collision_body.collision;
    const auto& link = collision_body.link;

    // here we assume the tf frames' names correspond to the link names
    const auto link_frame = getLinkTfPrefix() + link->name;

    // the collision object may have a different origin than the visual, we need to account for that
    const auto& collision_offset_transform = cras::toEigen(collision->origin);

    {
      auto link_transform_tf_optional = tf_frames_watchdog_->lookupTransform(
        link_frame, time, cras::remainingTime(time, reachable_transform_timeout_, clock_));

      if (!link_transform_tf_optional) {  // has no value
        continue;
      }

      const auto& link_transform_tf = link_transform_tf_optional.value();
      const auto& link_transform_eigen = tf2::transformToEigen(link_transform_tf);

      const auto& transform = link_transform_eigen * collision_offset_transform;

      transform_cache_[collision_body.cache_key] =
        std::allocate_shared<Eigen::Isometry3d>(Eigen::aligned_allocator<Eigen::Isometry3d>(), transform);
    }

    if (after_scan_time.seconds() != 0) {
      auto maybe_link_transform_tf = tf_frames_watchdog_->lookupTransform(
        link_frame, after_scan_time, cras::remainingTime(time, reachable_transform_timeout_, clock_));

      if (!maybe_link_transform_tf) {  // has no value
        continue;
      }

      const auto& link_transform_tf = maybe_link_transform_tf.value();
      const auto& link_transform_eigen = tf2::transformToEigen(link_transform_tf);

      const auto& transform = link_transform_eigen * collision_offset_transform;

      transform_cache_after_scan_[collision_body.cache_key] =
        std::allocate_shared<Eigen::Isometry3d>(Eigen::aligned_allocator<Eigen::Isometry3d>(), transform);
    }
  }
}

template<typename T>
void RobotBodyFilter<T>::addRobotMaskFromUrdf(const string& urdf_model) {
  if (should_stop_) {
    return;
  }

  if (urdf_model.empty()) {
    RCLCPP_ERROR(
      get_logger(), "RobotBodyFilter: Empty string passed as robot model to addRobotMaskFromUrdf. Robot body filtering "
      "is not going to work.");
    return;
  }

  // parse the URDF model
  auto parsed_urdf_model = std::make_unique<urdf::Model>();
  bool urdf_parse_succeeded = parsed_urdf_model->initString(urdf_model);
  if (!urdf_parse_succeeded) {
    RCLCPP_ERROR_STREAM(
      get_logger(), "RobotBodyFilter: The given URDF model cannot be parsed. See urdf::Model::initString for "
      "debugging, or try running 'gz sdf -p my_robot.urdf'");
    // Issue #6: Monitor sensor frame even if it is not a part of the model
    if (!sensor_frame_.empty()) {
      tf_frames_watchdog_->setMonitoredFrames(std::set<std::string>{sensor_frame_});
    }

    return;
  }

  {
    std::lock_guard<std::mutex> guard(*model_mutex_);

    parsed_urdf_model_ = std::move(parsed_urdf_model);

    shapes_ignored_in_bounding_sphere_.clear();
    shapes_ignored_in_bounding_box_.clear();
    std::unordered_set<MultiShapeHandle> ignore_in_contains_test;
    std::unordered_set<MultiShapeHandle> ignore_in_shadow_test;

    // add all model's collision links as masking shapes
    for (const auto& [linkName, link] : parsed_urdf_model_->links_) {
      // every link can have multiple collision elements
      size_t collision_index = 0;
      for (const auto& collision : link->collision_array) {
        if (collision->geometry == nullptr) {
          RCLCPP_WARN(
            get_logger(), "RobotBodyFilter: Collision element without geometry found in link %s of robot %s. "
            "This collision element will not be filtered out.",
            link->name.c_str(), parsed_urdf_model_->getName().c_str());
          continue;  // collisionIndex is intentionally not increased
        }

        const auto NAME_LINK = link->name;
        const auto NAME_COLLISION_NAME = "*::" + collision->name;
        const auto NAME_LINK_COLLISION_NR = link->name + "::" + std::to_string(collision_index);
        const auto NAME_LINK_COLLISON_NAME = link->name + "::" + collision->name;

        const std::vector<std::string> collision_names = {
          NAME_LINK,
          NAME_COLLISION_NAME,
          NAME_LINK_COLLISION_NR,
          NAME_LINK_COLLISON_NAME,
        };

        std::set<std::string> collision_names_set;
        std::set<std::string> collision_names_contains;
        std::set<std::string> collision_names_shadow;
        for (const auto& name : collision_names) {
          collision_names_set.insert(name);
          collision_names_contains.insert(name + kContainsSuffix);
          collision_names_shadow.insert(name + kShadowSuffix);
        }

        // if only_links_ is nonempty, make sure this collision belongs to a specified link
        if (!only_links_.empty()) {
          if (cras::isSetIntersectionEmpty(collision_names_set, only_links_)) {
            ++collision_index;
            continue;
          }
        }

        // if the link is ignored, go on
        if (!cras::isSetIntersectionEmpty(collision_names_set, links_ignored_everywhere_)) {
          ++collision_index;
          continue;
        }

        const auto collision_shape = constructShape(*collision->geometry);
        const auto shape_name = collision->name.empty() ? NAME_LINK_COLLISION_NR : NAME_LINK_COLLISON_NAME;

        // if the shape could not be constructed, ignore it (e.g. mesh was not found)
        if (collision_shape == nullptr) {
          RCLCPP_WARN(get_logger(), "Could not construct shape for collision %s, ignoring it.", shape_name.c_str());
          ++collision_index;
          continue;
        }

        // add the collision shape to shape_mask_; the inflation parameters come into play here
        const auto contains_test_inflation = getLinkInflationForContainsTest(collision_names);
        const auto shadow_test_inflation = getLinkInflationForShadowTest(collision_names);
        const auto bsphere_inflation = getLinkInflationForBoundingSphere(collision_names);
        const auto bbox_inflation = getLinkInflationForBoundingBox(collision_names);
        const auto shape_handle = shape_mask_->addShape(
          collision_shape,
          contains_test_inflation.scale, contains_test_inflation.padding,
          shadow_test_inflation.scale, shadow_test_inflation.padding,
          bsphere_inflation.scale, bsphere_inflation.padding,
          bbox_inflation.scale, bbox_inflation.padding,
          false, shape_name);
        shapes_to_links_[shape_handle.contains] = shapes_to_links_[shape_handle.shadow] =
          shapes_to_links_[shape_handle.bsphere] = shapes_to_links_[shape_handle.bbox] =
          CollisionBodyWithLink(collision, link, collision_index, shape_handle);

        if (!cras::isSetIntersectionEmpty(collision_names_set, links_ignored_in_bounding_sphere_)) {
          shapes_ignored_in_bounding_sphere_.insert(shape_handle.bsphere);
        }

        if (!cras::isSetIntersectionEmpty(collision_names_set, links_ignored_in_bounding_box_)) {
          shapes_ignored_in_bounding_box_.insert(shape_handle.bbox);
        }

        if (!cras::isSetIntersectionEmpty(collision_names_set, links_ignored_in_contains_test_)) {
          ignore_in_contains_test.insert(shape_handle);
        }

        if (!cras::isSetIntersectionEmpty(collision_names_set, links_ignored_in_shadow_test_)) {
          ignore_in_shadow_test.insert(shape_handle);
        }

        ++collision_index;
      }

      // no collision element found; only warn for links that are not ignored and have at least one visual
      if (collision_index == 0 && !link->visual_array.empty()) {
        if ((only_links_.empty() || (only_links_.find(link->name) != only_links_.end())) &&
          links_ignored_everywhere_.find(link->name) == links_ignored_everywhere_.end()) {
          RCLCPP_WARN(
            get_logger(), "RobotBodyFilter: No collision element found for link %s of robot %s. This link will not be "
            "filtered out from laser scans.", link->name.c_str(), parsed_urdf_model_->getName().c_str());
        }
      }
    }

    shape_mask_->setIgnoreInContainsTest(ignore_in_contains_test);
    shape_mask_->setIgnoreInShadowTest(ignore_in_shadow_test);

    shape_mask_->updateInternalShapeLists();

    std::set<std::string> monitored_frames;
    for (const auto& shape_to_link : shapes_to_links_) {
      monitored_frames.insert(shape_to_link.second.link->name);
    }
    // Issue #6: Monitor sensor frame even if it is not a part of the model
    if (!sensor_frame_.empty()) {
      monitored_frames.insert(sensor_frame_);
    }

    tf_frames_watchdog_->setMonitoredFrames(monitored_frames);
  }
}

template<typename T>
void RobotBodyFilter<T>::clearRobotMask() {
  {
    std::lock_guard<std::mutex> guard(*model_mutex_);

    if (shape_mask_ != nullptr) {
      std::unordered_set<MultiShapeHandle> removed_multi_shapes;
      for (const auto& shape_to_link : shapes_to_links_) {
        const auto& multiShape = shape_to_link.second.multi_handle;
        if (removed_multi_shapes.find(multiShape) == removed_multi_shapes.end()) {
          shape_mask_->removeShape(multiShape, false);
          removed_multi_shapes.insert(multiShape);
        }
      }
      shape_mask_->updateInternalShapeLists();
    }

    shapes_to_links_.clear();
    shapes_ignored_in_bounding_sphere_.clear();
    shapes_ignored_in_bounding_box_.clear();
    transform_cache_.clear();
    transform_cache_after_scan_.clear();
    parsed_urdf_model_.reset();
  }

  if (tf_frames_watchdog_ != nullptr) {
    tf_frames_watchdog_->clear();
  }
}

template<typename T>
bool RobotBodyFilter<T>::hasModel() const {
  return !robot_description_string_.empty();
}

template<typename T>
typename RobotBodyFilter<T>::RequiredInterfaces RobotBodyFilter<T>::createNodeInterfaces() {
  // If the params interface given to the filter from FilterChain is the standard rclcpp::n_i::NodeParameters class,
  // misuse it to get the other missing interfaces we need (they are hidden inside of it under private members which
  // we extract using the "#define private public" hack at the top of this file.
  {
    auto params = std::dynamic_pointer_cast<rclcpp::node_interfaces::NodeParameters>(this->params_interface_);
    if (params != nullptr) {
      auto clock = std::dynamic_pointer_cast<rclcpp::node_interfaces::NodeClock>(params->node_clock_);
      if (clock != nullptr) {
        RCLCPP_INFO(get_logger(), "RobotBodyFilter is initialized using a hack via its parameters interface.");
        return RequiredInterfaces{
          clock->node_base_,
          params->node_clock_,
          this->logging_interface_,
          this->params_interface_,
          clock->node_services_,
          clock->node_topics_,
        };
      }
    }
  }

  // If the params interface is something different, we create our own nodehandle. This has a lot of downsides, but
  // it's the best we can do given the circumstances.
  own_node_handle_ = std::make_shared<rclcpp::Node>(this->getName());
  executor_->add_node(own_node_handle_);

  return RequiredInterfaces{
    own_node_handle_->get_node_base_interface(),
    own_node_handle_->get_node_clock_interface(),
    this->logging_interface_,
    this->params_interface_,
    own_node_handle_->get_node_services_interface(),
    own_node_handle_->get_node_topics_interface(),
  };
}

template<typename T>
void RobotBodyFilter<T>::publishDebugMarkers(const rclcpp::Time& scan_time) const {
  // assume model_mutex_ is locked

  if (publish_debug_contains_marker_) {
    visualization_msgs::msg::MarkerArray marker_array;
    std_msgs::msg::ColorRGBA color;
    color.g = 1.0;
    color.a = 0.5;
    createBodyVisualizationMsg(
      shape_mask_->getBodiesForContainsTest(), scan_time, color, marker_array);
    debug_contains_marker_publisher_->publish(marker_array);
  }

  if (publish_debug_shadow_marker_) {
    visualization_msgs::msg::MarkerArray marker_array;
    std_msgs::msg::ColorRGBA color;
    color.b = 1.0;
    color.a = 0.5;
    createBodyVisualizationMsg(
      shape_mask_->getBodiesForShadowTest(), scan_time, color, marker_array);
    debug_shadow_marker_publisher_->publish(marker_array);
  }

  if (publish_debug_bsphere_marker_) {
    visualization_msgs::msg::MarkerArray marker_array;
    std_msgs::msg::ColorRGBA color;
    color.g = 1.0;
    color.b = 1.0;
    color.a = 0.5;
    createBodyVisualizationMsg(
      shape_mask_->getBodiesForBoundingSphere(), scan_time, color, marker_array);
    debug_bsphere_marker_publisher_->publish(marker_array);
  }

  if (publish_debug_bbox_marker_) {
    visualization_msgs::msg::MarkerArray marker_array;
    std_msgs::msg::ColorRGBA color;
    color.r = 1.0;
    color.b = 1.0;
    color.a = 0.5;
    createBodyVisualizationMsg(
      shape_mask_->getBodiesForBoundingBox(), scan_time, color, marker_array);
    debug_bbox_marker_publisher_->publish(marker_array);
  }
}

template<typename T>
void RobotBodyFilter<T>::publishDebugPointClouds(
    const sensor_msgs::msg::PointCloud2& projected_point_cloud,
    const std::vector<RayCastingShapeMask::MaskValue>& point_mask) const {
  if (publish_debug_pcl_inside_) {
    sensor_msgs::msg::PointCloud2 inside_cloud;
    CREATE_FILTERED_CLOUD(
      projected_point_cloud, inside_cloud, keep_clouds_organized_,
      (point_mask[i] == RayCastingShapeMask::MaskValue::INSIDE));
    debug_point_cloud_inside_publisher_->publish(inside_cloud);
  }

  if (publish_debug_pcl_clip_) {
    sensor_msgs::msg::PointCloud2 clip_cloud;
    CREATE_FILTERED_CLOUD(
      projected_point_cloud, clip_cloud, keep_clouds_organized_,
      (point_mask[i] == RayCastingShapeMask::MaskValue::CLIP));
    debug_point_cloud_clip_publisher_->publish(clip_cloud);
  }

  if (publish_debug_pcl_shadow_) {
    sensor_msgs::msg::PointCloud2 shadow_cloud;
    CREATE_FILTERED_CLOUD(
      projected_point_cloud, shadow_cloud, keep_clouds_organized_,
      (point_mask[i] == RayCastingShapeMask::MaskValue::SHADOW));
    debug_point_cloud_shadow_publisher_->publish(shadow_cloud);
  }
}

template<typename T>
void RobotBodyFilter<T>::computeAndPublishBoundingSphere(
  const sensor_msgs::msg::PointCloud2& projected_point_cloud) const {
  if (!compute_bounding_sphere_ && !compute_debug_bounding_sphere_) {
    return;
  }

  // assume model_mutex_ is locked

  // when computing bounding spheres for publication, we want to publish them to the time of the
  // scan, so we need to set cache_lookup_between_scans_ratio_ again to zero
  if (cache_lookup_between_scans_ratio_ != 0.0) {
    cache_lookup_between_scans_ratio_ = 0.0;
    shape_mask_->updateBodyPoses();
  }

  const auto& scan_time = projected_point_cloud.header.stamp;
  std::vector<bodies::BoundingSphere> spheres;
  {
    visualization_msgs::msg::MarkerArray bounding_sphere_debug_msg;
    for (const auto& [shapeHandle, body] : shape_mask_->getBodiesForBoundingSphere()) {
      if (shapes_ignored_in_bounding_sphere_.find(shapeHandle) !=
          shapes_ignored_in_bounding_sphere_.end()) {
        continue;
      }

      bodies::BoundingSphere sphere;
      body->computeBoundingSphere(sphere);

      spheres.push_back(sphere);

      if (compute_debug_bounding_sphere_) {
        visualization_msgs::msg::Marker msg;
        msg.header.stamp = scan_time;
        msg.header.frame_id = filtering_frame_;

        msg.scale.x = msg.scale.y = msg.scale.z = sphere.radius * 2;

        msg.pose.position.x = sphere.center[0];
        msg.pose.position.y = sphere.center[1];
        msg.pose.position.z = sphere.center[2];
        msg.pose.orientation.w = 1;

        msg.color.g = 1.0;
        msg.color.a = 0.5;
        msg.type = visualization_msgs::msg::Marker::SPHERE;
        msg.action = visualization_msgs::msg::Marker::ADD;
        msg.ns = "bsphere/" + shapes_to_links_.at(shapeHandle).cache_key;
        msg.frame_locked = true;

        bounding_sphere_debug_msg.markers.push_back(msg);
      }
    }

    if (compute_debug_bounding_sphere_) {
      bounding_sphere_debug_marker_publisher_->publish(bounding_sphere_debug_msg);
    }
  }

  if (compute_bounding_sphere_) {
    bodies::BoundingSphere bounding_sphere;
    bodies::mergeBoundingSpheres(spheres, bounding_sphere);

    robot_body_filter::msg::SphereStamped bounding_sphere_msg;
    bounding_sphere_msg.header.stamp = scan_time;
    bounding_sphere_msg.header.frame_id = filtering_frame_;
    bounding_sphere_msg.sphere.radius = static_cast<float>(bounding_sphere.radius);
    bounding_sphere_msg.sphere.center = tf2::toMsg(bounding_sphere.center);

    bounding_sphere_publisher_->publish(bounding_sphere_msg);

    if (publish_bounding_sphere_marker_) {
      visualization_msgs::msg::Marker msg;
      msg.header.stamp = scan_time;
      msg.header.frame_id = filtering_frame_;

      msg.scale.x = msg.scale.y = msg.scale.z = bounding_sphere.radius * 2;

      msg.pose.position.x = bounding_sphere.center[0];
      msg.pose.position.y = bounding_sphere.center[1];
      msg.pose.position.z = bounding_sphere.center[2];
      msg.pose.orientation.w = 1;

      msg.color.g = 1.0;
      msg.color.a = 0.5;
      msg.type = visualization_msgs::msg::Marker::SPHERE;
      msg.action = visualization_msgs::msg::Marker::ADD;
      msg.ns = "bounding_sphere";
      msg.frame_locked = true;

      bounding_sphere_marker_publisher_->publish(msg);
    }

    if (publish_no_bounding_sphere_pointcloud_) {
      sensor_msgs::msg::PointCloud2 no_sphere_cloud;
      CREATE_FILTERED_CLOUD(
        projected_point_cloud, no_sphere_cloud, keep_clouds_organized_,
        ((Eigen::Vector3d(*x_it, *y_it, *z_it) - bounding_sphere.center).norm() > bounding_sphere.radius));
      scan_point_cloud_no_bounding_sphere_publisher_->publish(no_sphere_cloud);
    }
  }
}

template<typename T>
void RobotBodyFilter<T>::computeAndPublishBoundingBox(
  const sensor_msgs::msg::PointCloud2& projected_point_cloud) const {
  if (!compute_bounding_box_ && !compute_debug_bounding_box_) {
    return;
  }

  // assume model_mutex_ is locked

  // when computing bounding boxes for publication, we want to publish them to the time of the
  // scan, so we need to set cache_lookup_between_scans_ratio_ again to zero
  if (cache_lookup_between_scans_ratio_ != 0.0) {
    cache_lookup_between_scans_ratio_ = 0.0;
    shape_mask_->updateBodyPoses();
  }

  const auto& scan_time = projected_point_cloud.header.stamp;
  std::vector<bodies::AxisAlignedBoundingBox> boxes;

  {
    visualization_msgs::msg::MarkerArray bounding_box_debug_msg;
    for (const auto& [shapeHandle, body] : shape_mask_->getBodiesForBoundingBox()) {
      if (shapes_ignored_in_bounding_box_.find(shapeHandle) != shapes_ignored_in_bounding_box_.end()) {
        continue;
      }

      bodies::AxisAlignedBoundingBox box;
      body->computeBoundingBox(box);

      boxes.push_back(box);

      if (compute_debug_bounding_box_) {
        visualization_msgs::msg::Marker msg;
        msg.header.stamp = scan_time;
        msg.header.frame_id = filtering_frame_;

        // it is aligned to fixed frame, not necessarily robot frame
        tf2::toMsg(box.sizes(), msg.scale);
        msg.pose.position = tf2::toMsg(static_cast<Eigen::Vector3d>(box.center()));
        msg.pose.orientation.w = 1;

        msg.color.g = 1.0;
        msg.color.a = 0.5;
        msg.type = visualization_msgs::msg::Marker::CUBE;
        msg.action = visualization_msgs::msg::Marker::ADD;
        msg.ns = "bbox/" + shapes_to_links_.at(shapeHandle).cache_key;
        msg.frame_locked = true;

        bounding_box_debug_msg.markers.push_back(msg);
      }
    }

    if (compute_debug_bounding_box_) {
      bounding_box_debug_marker_publisher_->publish(bounding_box_debug_msg);
    }
  }

  if (compute_bounding_box_) {
    bodies::AxisAlignedBoundingBox box;
    bodies::mergeBoundingBoxes(boxes, box);
    const auto box_float = box.cast<float>();

    geometry_msgs::msg::PolygonStamped bounding_box_msg;

    bounding_box_msg.header.stamp = scan_time;
    bounding_box_msg.header.frame_id = filtering_frame_;

    bounding_box_msg.polygon.points.resize(2);
    tf2::toMsg(box.min(), bounding_box_msg.polygon.points[0]);
    tf2::toMsg(box.max(), bounding_box_msg.polygon.points[1]);

    bounding_box_publisher_->publish(bounding_box_msg);

    if (publish_bounding_box_marker_) {
      visualization_msgs::msg::Marker msg;
      msg.header.stamp = scan_time;
      msg.header.frame_id = filtering_frame_;

      // it is aligned to fixed frame and not necessarily to robot frame
      tf2::toMsg(box.sizes(), msg.scale);
      msg.pose.position = tf2::toMsg(static_cast<Eigen::Vector3d>(box.center()));
      msg.pose.orientation.w = 1;

      msg.color.r = 1.0;
      msg.color.a = 0.5;
      msg.type = visualization_msgs::msg::Marker::CUBE;
      msg.action = visualization_msgs::msg::Marker::ADD;
      msg.ns = "bounding_box";
      msg.frame_locked = true;

      bounding_box_marker_publisher_->publish(msg);
    }

    // compute and publish the scan_point_cloud with robot bounding box removed
    if (publish_no_bounding_box_pointcloud_) {
      pcl::PCLPointCloud2::Ptr bbox_crop_input(new pcl::PCLPointCloud2());
      pcl_conversions::toPCL(projected_point_cloud, *bbox_crop_input);

      pcl::CropBox<pcl::PCLPointCloud2> crop_box;
      crop_box.setNegative(true);
      crop_box.setInputCloud(bbox_crop_input);
      crop_box.setKeepOrganized(keep_clouds_organized_);

      crop_box.setMin(Eigen::Vector4f(box_float.min()[0], box_float.min()[1], box_float.min()[2], 0.0));
      crop_box.setMax(Eigen::Vector4f(box_float.max()[0], box_float.max()[1], box_float.max()[2], 0.0));

      pcl::PCLPointCloud2 pcl_output;
      crop_box.filter(pcl_output);

      sensor_msgs::msg::PointCloud2::SharedPtr box_filtered_cloud(new sensor_msgs::msg::PointCloud2());
      pcl_conversions::moveFromPCL(pcl_output, *box_filtered_cloud);
      box_filtered_cloud->header.stamp = scan_time;  // PCL strips precision of timestamp

      scan_point_cloud_no_bounding_box_publisher_->publish(*box_filtered_cloud);
    }
  }
}

template<typename T>
void RobotBodyFilter<T>::computeAndPublishOrientedBoundingBox(
  const sensor_msgs::msg::PointCloud2& projected_point_cloud) const {
  if (!compute_oriented_bounding_box_ && !compute_debug_oriented_bounding_box_) {
    return;
  }

  // assume model_mutex_ is locked

  // when computing bounding boxes for publication, we want to publish them to the time of the
  // scan, so we need to set cache_lookup_between_scans_ratio_ again to zero
  if (cache_lookup_between_scans_ratio_ != 0.0) {
    cache_lookup_between_scans_ratio_ = 0.0;
    shape_mask_->updateBodyPoses();
  }

  const auto& scan_time = projected_point_cloud.header.stamp;
  std::vector<bodies::OrientedBoundingBox> boxes;

  {
    visualization_msgs::msg::MarkerArray bounding_box_debug_msg;
    for (const auto& [shape_handle, body] : shape_mask_->getBodiesForBoundingBox()) {
      if (shapes_ignored_in_bounding_box_.find(shape_handle) != shapes_ignored_in_bounding_box_.end()) {
        continue;
      }

      bodies::OrientedBoundingBox box;
      body->computeBoundingBox(box);

      boxes.push_back(box);

      if (compute_debug_oriented_bounding_box_) {
        visualization_msgs::msg::Marker msg;
        msg.header.stamp = scan_time;
        msg.header.frame_id = filtering_frame_;

        tf2::toMsg(box.getExtents(), msg.scale);
        msg.pose.position = tf2::toMsg(static_cast<Eigen::Vector3d>(box.getPose().translation()));
        msg.pose.orientation = tf2::toMsg(Eigen::Quaterniond(box.getPose().linear()));

        msg.color.g = 1.0;
        msg.color.a = 0.5;
        msg.type = visualization_msgs::msg::Marker::CUBE;
        msg.action = visualization_msgs::msg::Marker::ADD;
        msg.ns = "obbox/" + shapes_to_links_.at(shape_handle).cache_key;
        msg.frame_locked = true;

        bounding_box_debug_msg.markers.push_back(msg);
      }
    }

    if (compute_debug_oriented_bounding_box_) {
      oriented_bounding_box_debug_marker_publisher_->publish(bounding_box_debug_msg);
    }
  }

  if (compute_oriented_bounding_box_) {
    bodies::OrientedBoundingBox box(Eigen::Isometry3d::Identity(), Eigen::Vector3d::Zero());
    bodies::mergeBoundingBoxesApprox(boxes, box);

    robot_body_filter::msg::OrientedBoundingBoxStamped bounding_box_msg;

    bounding_box_msg.header.stamp = scan_time;
    bounding_box_msg.header.frame_id = filtering_frame_;

    tf2::toMsg(box.getExtents(), bounding_box_msg.obb.extents);
    tf2::toMsg(box.getPose().translation(), bounding_box_msg.obb.pose.translation);
    bounding_box_msg.obb.pose.rotation = tf2::toMsg(Eigen::Quaterniond(box.getPose().linear()));

    oriented_bounding_box_publisher_->publish(bounding_box_msg);

    if (publish_oriented_bounding_box_marker_) {
      visualization_msgs::msg::Marker msg;
      msg.header.stamp = scan_time;
      msg.header.frame_id = filtering_frame_;

      tf2::toMsg(box.getExtents(), msg.scale);
      msg.pose.position = tf2::toMsg(static_cast<Eigen::Vector3d>(box.getPose().translation()));
      msg.pose.orientation = tf2::toMsg(Eigen::Quaterniond(box.getPose().linear()));

      msg.color.r = 1.0;
      msg.color.a = 0.5;
      msg.type = visualization_msgs::msg::Marker::CUBE;
      msg.action = visualization_msgs::msg::Marker::ADD;
      msg.ns = "oriented_bounding_box";
      msg.frame_locked = true;

      oriented_bounding_box_marker_publisher_->publish(msg);
    }

    // compute and publish the scan_point_cloud with robot bounding box removed
    if (publish_no_oriented_bounding_box_pointcloud_) {
      const pcl::PCLPointCloud2::Ptr bbox_crop_input(new pcl::PCLPointCloud2());
      pcl_conversions::toPCL(projected_point_cloud, *bbox_crop_input);

      pcl::CropBox<pcl::PCLPointCloud2> crop_box;
      crop_box.setNegative(true);
      crop_box.setInputCloud(bbox_crop_input);
      crop_box.setKeepOrganized(keep_clouds_organized_);

      const auto e = box.getExtents();
      crop_box.setMin(Eigen::Vector4f(-e.x() / 2, -e.y() / 2, -e.z() / 2, 0.0));
      crop_box.setMax(Eigen::Vector4f(e.x() / 2, e.y() / 2, e.z() / 2, 0.0));
      crop_box.setTranslation(box.getPose().translation().cast<float>());
      crop_box.setRotation(box.getPose().linear().eulerAngles(0, 1, 2).cast<float>());

      pcl::PCLPointCloud2 pcl_output;
      crop_box.filter(pcl_output);

      const sensor_msgs::msg::PointCloud2::SharedPtr box_filtered_cloud(new sensor_msgs::msg::PointCloud2());
      pcl_conversions::moveFromPCL(pcl_output, *box_filtered_cloud);
      box_filtered_cloud->header.stamp = scan_time;  // PCL strips precision of timestamp

      scan_point_cloud_no_oriented_bounding_box_publisher_->publish(*box_filtered_cloud);
    }
  }
}

template<typename T>
void RobotBodyFilter<T>::computeAndPublishLocalBoundingBox(
  const sensor_msgs::msg::PointCloud2& projected_point_cloud) const {
  if (!compute_local_bounding_box_ && !compute_debug_local_bounding_box_) {
    return;
  }

  // assume model_mutex_ is locked

  const auto& scan_time = projected_point_cloud.header.stamp;
  std::string err;
  try {
    if (!tf_buffer_->canTransform(
      local_bounding_box_frame_, filtering_frame_, scan_time,
      cras::remainingTime(scan_time, reachable_transform_timeout_, clock_), &err)) {
      // TODO: Originally was delayed-throttle
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *clock_, 3.0, "Cannot get transform %s->%s. Error is %s.",
        filtering_frame_.c_str(), local_bounding_box_frame_.c_str(), err.c_str());
      return;
    }
  } catch (tf2::TransformException& e) {
    // TODO: Originally was delayed-throttle
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *clock_, 3.0, "Cannot get transform %s->%s. Error is %s.",
      filtering_frame_.c_str(), local_bounding_box_frame_.c_str(), e.what());
    return;
  }

  const auto local_tf_msg = tf_buffer_->lookupTransform(
    local_bounding_box_frame_, filtering_frame_, scan_time);
  const Eigen::Isometry3d local_tf = tf2::transformToEigen(local_tf_msg.transform);

  std::vector<bodies::AxisAlignedBoundingBox> boxes;

  {
    visualization_msgs::msg::MarkerArray bounding_box_debug_msg;
    for (const auto& [shape_handle, body] : shape_mask_->getBodiesForBoundingBox()) {
      if (shapes_ignored_in_bounding_box_.find(shape_handle) != shapes_ignored_in_bounding_box_.end()) {
        continue;
      }

      bodies::AxisAlignedBoundingBox box;
      bodies::computeBoundingBoxAt(body, box, local_tf * body->getPose());

      boxes.push_back(box);

      if (compute_debug_local_bounding_box_) {
        visualization_msgs::msg::Marker msg;
        msg.header.stamp = scan_time;
        msg.header.frame_id = local_bounding_box_frame_;

        tf2::toMsg(box.sizes(), msg.scale);
        msg.pose.position = tf2::toMsg(static_cast<Eigen::Vector3d>(box.center()));
        msg.pose.orientation.w = 1;

        msg.color.g = 1.0;
        msg.color.a = 0.5;
        msg.type = visualization_msgs::msg::Marker::CUBE;
        msg.action = visualization_msgs::msg::Marker::ADD;
        msg.ns = "lbbox/" + shapes_to_links_.at(shape_handle).cache_key;
        msg.frame_locked = true;

        bounding_box_debug_msg.markers.push_back(msg);
      }
    }

    if (compute_debug_local_bounding_box_) {
      local_bounding_box_debug_marker_publisher_->publish(bounding_box_debug_msg);
    }
  }

  if (compute_local_bounding_box_) {
    bodies::AxisAlignedBoundingBox box;
    bodies::mergeBoundingBoxes(boxes, box);

    geometry_msgs::msg::PolygonStamped bounding_box_msg;

    bounding_box_msg.header.stamp = scan_time;
    bounding_box_msg.header.frame_id = local_bounding_box_frame_;

    bounding_box_msg.polygon.points.resize(2);
    tf2::toMsg(box.min(), bounding_box_msg.polygon.points[0]);
    tf2::toMsg(box.max(), bounding_box_msg.polygon.points[1]);

    local_bounding_box_publisher_->publish(bounding_box_msg);

    if (publish_local_bounding_box_marker_) {
      visualization_msgs::msg::Marker msg;
      msg.header.stamp = scan_time;
      msg.header.frame_id = local_bounding_box_frame_;

      tf2::toMsg(box.sizes(), msg.scale);
      msg.pose.position = tf2::toMsg(static_cast<Eigen::Vector3d>(box.center()));
      msg.pose.orientation.w = 1;

      msg.color.r = 1.0;
      msg.color.a = 0.5;
      msg.type = visualization_msgs::msg::Marker::CUBE;
      msg.action = visualization_msgs::msg::Marker::ADD;
      msg.ns = "local_bounding_box";
      msg.frame_locked = true;

      local_bounding_box_marker_publisher_->publish(msg);
    }

    // compute and publish the scan_point_cloud with robot bounding box removed
    if (publish_no_local_bounding_box_pointcloud_) {
      pcl::PCLPointCloud2::Ptr bbox_crop_input(new pcl::PCLPointCloud2());
      pcl_conversions::toPCL(projected_point_cloud, *bbox_crop_input);

      pcl::CropBox<pcl::PCLPointCloud2> crop_box;
      crop_box.setNegative(true);
      crop_box.setInputCloud(bbox_crop_input);
      crop_box.setKeepOrganized(keep_clouds_organized_);

      crop_box.setMin(Eigen::Vector4f(box.min()[0], box.min()[1], box.min()[2], 0.0));
      crop_box.setMax(Eigen::Vector4f(box.max()[0], box.max()[1], box.max()[2], 0.0));
      const Eigen::Isometry3d localTfInv = local_tf.inverse();
      crop_box.setTranslation(localTfInv.translation().cast<float>());
      crop_box.setRotation(localTfInv.linear().eulerAngles(0, 1, 2).cast<float>());

      pcl::PCLPointCloud2 pcl_output;
      crop_box.filter(pcl_output);

      sensor_msgs::msg::PointCloud2::SharedPtr box_filtered_cloud(new sensor_msgs::msg::PointCloud2());
      pcl_conversions::moveFromPCL(pcl_output, *box_filtered_cloud);
      box_filtered_cloud->header.stamp = scan_time;  // PCL strips precision of timestamp

      scan_point_cloud_no_local_bounding_box_publisher_->publish(*box_filtered_cloud);
    }
  }
}

template<typename T>
void RobotBodyFilter<T>::createBodyVisualizationMsg(
  const std::map<point_containment_filter::ShapeHandle, const bodies::Body*>& bodies, const rclcpp::Time& stamp,
  const std_msgs::msg::ColorRGBA& color, visualization_msgs::msg::MarkerArray& marker_array) const {
  // when computing the markers for publication, we want to publish them to the time of the
  // scan, so we need to set cache_lookup_between_scans_ratio_ again to zero
  if (cache_lookup_between_scans_ratio_ != 0.0) {
    cache_lookup_between_scans_ratio_ = 0.0;
    shape_mask_->updateBodyPoses();
  }

  for (const auto& [shapeHandle, body] : bodies) {
    visualization_msgs::msg::Marker msg;
    bodies::constructMarkerFromBody(body, msg);

    msg.header.stamp = stamp;
    msg.header.frame_id = filtering_frame_;

    msg.color = color;
    msg.action = visualization_msgs::msg::Marker::ADD;
    msg.ns = shapes_to_links_.at(shapeHandle).cache_key;
    msg.frame_locked = true;

    marker_array.markers.push_back(msg);
  }
}

template<typename T>
rcl_interfaces::msg::SetParametersResult RobotBodyFilter<T>::paramUpdateCallback(
    const std::vector<rclcpp::Parameter>&) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;
  result.reason = "No dynamic parameters are supported yet.";
  return result;
}

template<typename T>
void RobotBodyFilter<T>::onRobotModelMsg(const std_msgs::msg::String::ConstSharedPtr& msg) {
  if (!this->configured_ || should_stop_ || tf_frames_watchdog_ == nullptr) {
    return;
  }

  RCLCPP_INFO(
    get_logger(), "RobotBodyFilter: Reloading robot model because a new model was received. Filter operation stopped.");

  robot_description_string_ = msg->data;

  tf_frames_watchdog_->pause();
  this->configured_ = false;

  clearRobotMask();
  addRobotMaskFromUrdf(robot_description_string_);

  tf_frames_watchdog_->unpause();
  time_configured_ = clock_->now();
  this->configured_ = true;

  RCLCPP_INFO(get_logger(), "RobotBodyFilter: Robot model reloaded, resuming filter operation.");
}

template<typename T>
void RobotBodyFilter<T>::triggerModelReload(
  const std::shared_ptr<rmw_request_id_t>, const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
  if (!this->configured_ || should_stop_ || tf_frames_watchdog_ == nullptr) {
    return;
  }

  RCLCPP_INFO(get_logger(), "RobotBodyFilter: Reloading robot model because of trigger. Filter operation stopped.");

  tf_frames_watchdog_->pause();
  this->configured_ = false;

  clearRobotMask();
  addRobotMaskFromUrdf(robot_description_string_);

  tf_frames_watchdog_->unpause();
  time_configured_ = clock_->now();
  this->configured_ = true;

  RCLCPP_INFO(get_logger(), "RobotBodyFilter: Robot model reloaded, resuming filter operation.");
  res->success = true;
}

template<typename T>
RobotBodyFilter<T>::~RobotBodyFilter() {
  should_stop_ = true;
  this->configured_ = false;

  if (tf_frames_watchdog_ != nullptr) {
    tf_frames_watchdog_->stop();
  }

  // Stop the internal executor
  if (executor_ != nullptr) {
    executor_->cancel();
  }
  if (executor_thread_ != nullptr) {
    if (executor_thread_->joinable()) {
      executor_thread_->join();
    }
    executor_thread_.reset();
  }
  if (executor_ != nullptr) {
    executor_.reset();
  }

  clearRobotMask();
}

template<typename T>
ScaleAndPadding RobotBodyFilter<T>::getLinkInflationForContainsTest(const string& link_name) const {
  return getLinkInflationForContainsTest({link_name});
}

template<typename T>
ScaleAndPadding RobotBodyFilter<T>::getLinkInflationForContainsTest(const std::vector<std::string>& link_names) const {
  return getLinkInflation(link_names, default_contains_inflation_, per_link_contains_inflation_);
}

template<typename T>
ScaleAndPadding RobotBodyFilter<T>::getLinkInflationForShadowTest(const string& link_name) const {
  return getLinkInflationForShadowTest({link_name});
}

template<typename T>
ScaleAndPadding RobotBodyFilter<T>::getLinkInflationForShadowTest(const std::vector<std::string>& link_names) const {
  return getLinkInflation(link_names, default_shadow_inflation_, per_link_shadow_inflation_);
}

template<typename T>
ScaleAndPadding RobotBodyFilter<T>::getLinkInflationForBoundingSphere(const string& link_name) const {
  return getLinkInflationForBoundingSphere({link_name});
}

template<typename T>
ScaleAndPadding RobotBodyFilter<T>::getLinkInflationForBoundingSphere(
    const std::vector<std::string>& link_names) const {
  return getLinkInflation(link_names, default_bsphere_inflation_, per_link_bsphere_inflation_);
}

template<typename T>
ScaleAndPadding RobotBodyFilter<T>::getLinkInflationForBoundingBox(const string& link_name) const {
  return getLinkInflationForBoundingBox({link_name});
}

template<typename T>
ScaleAndPadding RobotBodyFilter<T>::getLinkInflationForBoundingBox(const std::vector<std::string>& link_names) const {
  return getLinkInflation(link_names, default_bbox_inflation_, per_link_bbox_inflation_);
}

template<typename T>
ScaleAndPadding RobotBodyFilter<T>::getLinkInflation(
    const std::vector<std::string>& link_names, const ScaleAndPadding& default_inflation,
    const std::map<std::string, ScaleAndPadding>& per_link_inflation) const {
  ScaleAndPadding result = default_inflation;

  for (const auto& link_name : link_names) {
    if (per_link_inflation.find(link_name) != per_link_inflation.end()) {
      result = per_link_inflation.at(link_name);
    }
  }

  return result;
}

ScaleAndPadding::ScaleAndPadding(const double scale, const double padding) : scale(scale), padding(padding) {}

bool ScaleAndPadding::operator==(const ScaleAndPadding& other) const {
  return scale == other.scale && padding == other.padding;
}

bool ScaleAndPadding::operator!=(const ScaleAndPadding& other) const {
  return !(*this == other);
}

template<typename T>
std::string RobotBodyFilter<T>::getLinkTfPrefix() const {
  if (link_tf_prefix_.empty()) {
    return "";
  }
  return link_tf_prefix_ + "/";
}
}

PLUGINLIB_EXPORT_CLASS(robot_body_filter::RobotBodyFilterLaserScan, filters::FilterBase<sensor_msgs::msg::LaserScan>)

PLUGINLIB_EXPORT_CLASS(
  robot_body_filter::RobotBodyFilterPointCloud2, filters::FilterBase<sensor_msgs::msg::PointCloud2>)
