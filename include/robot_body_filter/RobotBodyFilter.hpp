#pragma once

// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include <cstdint>
#include <cstdio>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cras_cpp_common/tf2_sensor_msgs.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <moveit/point_containment_filter/shape_mask.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/executor.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/node_interfaces/get_node_base_interface.hpp>
#include <rclcpp/node_interfaces/get_node_clock_interface.hpp>
#include <rclcpp/node_interfaces/get_node_logging_interface.hpp>
#include <rclcpp/node_interfaces/get_node_parameters_interface.hpp>
#include <rclcpp/node_interfaces/get_node_services_interface.hpp>
#include <rclcpp/node_interfaces/get_node_topics_interface.hpp>
#include <rclcpp/node_interfaces/node_interfaces.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>
#include <rclcpp/parameter.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <rmw/types.h>
#include <robot_body_filter/msg/oriented_bounding_box_stamped.hpp>
#include <robot_body_filter/msg/sphere_stamped.hpp>
#include <robot_body_filter/RayCastingShapeMask.hpp>
#include <robot_body_filter/TfFramesWatchdog.hpp>
#include <robot_body_filter/utils/filter_utils.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <urdf_model/link.h>
#include <urdf/model.h>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace robot_body_filter {

/**
* \brief Just a helper structure holding together a link, one of its collision elements,
 *       and the index of the collision element in the collision array of the link.
*/
struct CollisionBodyWithLink {
  urdf::CollisionSharedPtr collision;
  urdf::LinkSharedPtr link;
  size_t index_in_collision_array;
  MultiShapeHandle multi_handle;
  std::string cache_key;

  CollisionBodyWithLink() : index_in_collision_array(0), multi_handle({}), cache_key("__empty__") {}

  CollisionBodyWithLink(
    urdf::CollisionSharedPtr collision, urdf::LinkSharedPtr link,
    const size_t indexInCollisionArray, const MultiShapeHandle& multiHandle)
    : collision(collision), link(link), index_in_collision_array(indexInCollisionArray), multi_handle(multiHandle) {

    std::ostringstream stream;
    stream << link->name << "-" << indexInCollisionArray;
    cache_key = stream.str();
  }
};

struct ScaleAndPadding {
  double scale;
  double padding;

  explicit ScaleAndPadding(double scale = 1.0, double padding = 0.0);

  bool operator==(const ScaleAndPadding& other) const;

  bool operator!=(const ScaleAndPadding& other) const;
};

/** \brief Suffix added to link/collision names to distinguish their usage in contains tests only. */
static inline constexpr char kContainsSuffix[] = "::contains";
/** \brief Suffix added to link/collision names to distinguish their usage in shadow tests only. */
static inline constexpr char kShadowSuffix[] = "::shadow";
/** \brief Suffix added to link/collision names to distinguish their usage in bounding sphere computation only. */
static inline constexpr char kBsphereSuffix[] = "::bounding_sphere";
/** \brief Suffix added to link/collision names to distinguish their usage in bounding box computation only. */
static inline constexpr char kBboxSuffix[] = "::bounding_box";

/**
 * \brief Filter to remove robot's own body from laser scan.
 *
 * See readme for more information.
 *
 * \author Martin Pecka
 */
template<typename T>
class RobotBodyFilter : public ::robot_body_filter::FilterBase<T> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using RequiredInterfaces = rclcpp::node_interfaces::NodeInterfaces<
    rclcpp::node_interfaces::NodeBaseInterface,
    rclcpp::node_interfaces::NodeClockInterface,
    rclcpp::node_interfaces::NodeLoggingInterface,
    rclcpp::node_interfaces::NodeParametersInterface,
    rclcpp::node_interfaces::NodeServicesInterface,
    rclcpp::node_interfaces::NodeTopicsInterface>;

  RobotBodyFilter();

  ~RobotBodyFilter() override;

  bool update(const T& data_in, T& data_out) override = 0;

  /**
   * \brief Tell whether some robot model has been already received.
   */
  virtual bool hasModel() const;

protected:
  //! This filter needs a few more interfaces than FilterBase provides. This function should provide them (however they
  //! are created).
  virtual RequiredInterfaces createNodeInterfaces();

  //! Read config parameters loaded by FilterBase::configure(...)
  //! Parameters are described in the readme.
  bool configure() override;

  rclcpp::Logger get_logger() const {
    return this->logging_interface_->get_logger();
  }

  rclcpp::Executor::UniquePtr executor_;  //!< Executor handling the topics required by this filter.
  RequiredInterfaces node_interfaces_;  //!< Handles to various required node interfaces.
  rclcpp::Clock::SharedPtr clock_;  //!< The clock to use.

  volatile bool should_stop_;
  std::unique_ptr<std::thread> executor_thread_;  //!< Thread running the internal executor.

  /**
   * \brief If true, suppose that every point in the scan was captured at a
   *        different time instant. Otherwise, the scan is assumed to be taken at once.
   *
   * \note Always true for T = LaserScan.
   *
   * \note If this is true and T = PointCloud2, the processing pipeline expects
   *       the pointcloud to have fields int32 index, float32 stamps, and float32
   *       vp_x, vp_y and vp_z viewpoint positions. If one of these fields is missing,
   *       computeMask() throws runtime exception.
   */
  bool point_by_point_scan_;

  //! Whether to keep pointcloud organized or not (if not, invalid points are
  //! removed).
  bool keep_clouds_organized_;

  /**
   * \brief The interval between two consecutive model pose updates when
   *        processing a pointByPointScan. If set to zero, the model will be updated
   *        for each point separately (might be computationally exhaustive). If
   *        non-zero, it will only update the model once in this interval, which makes
   *        the masking algorithm a little bit less precise but more computationally
   *        affordable.
   */
  rclcpp::Duration model_pose_update_interval_;

  /**
   * \brief Fixed frame wrt the sensor frame.
   *        Usually base_link for stationary robots (or sensor frame if both
   *        robot and sensor are stationary). For mobile robots, it can be e.g.
   *        odom or map.
   */
  std::string fixed_frame_;

  /**
   * \brief Frame of the sensor. For LaserScan version, it is automatically
   *        read from the incoming data. For PointCloud2, you have to specify it
   *        explicitly because the pointcloud could have already been transformed e.g.
   *        to the fixed frame.
   */
  std::string sensor_frame_;

  /**
   * \brief Frame in which the filter is applied. For point-by-point scans, it
   *        has to be a fixed frame, otherwise, it can be the sensor frame.
   */
  std::string filtering_frame_;

  //! The minimum distance of points from the sensor to keep them (in meters).
  double min_distance_;

  //! The maximum distance of points from the sensor origin to apply this filter on (in meters).
  double max_distance_;

  //! The default inflation that is applied to the collision model for the purposes of checking if a point is contained
  //! by the robot model (scale 1.0 = no scaling, padding 0.0 = no padding). Every collision element is scaled
  //! individually with the scaling center in its origin. Padding is added individually to every collision element.
  ScaleAndPadding default_contains_inflation_;

  //! The default inflation that is applied to the collision model for the purposes of checking if a point is shadowed
  //! by the robot model (scale 1.0 = no scaling, padding 0.0 = no padding). Every collision element is scaled
  //! individually with the scaling center in its origin. Padding is added individually to every collision element.
  ScaleAndPadding default_shadow_inflation_;

  //! The default inflation that is applied to the collision model for the purposes of computing the bounding sphere.
  //! Every collision element is scaled individually with the scaling center in its origin. Padding is added
  //! individually to every collision element.
  ScaleAndPadding default_bsphere_inflation_;

  //! The default inflation that is applied to the collision model for the purposes of computing the bounding box.
  //! Every collision element is scaled individually with the scaling center in its origin. Padding is added
  //! individually to every collision element.
  ScaleAndPadding default_bbox_inflation_;

  //! Inflation that is applied to a collision element for the purposes of checking if a point is contained by the
  //! robot model (scale 1.0 = no scaling, padding 0.0 = no padding). Elements not present in this list are scaled and
  //! padded with default_contains_inflation_.
  std::map<std::string, ScaleAndPadding> per_link_contains_inflation_;

  //! Inflation that is applied to a collision element for the purposes of checking if a point is shadowed by the
  //! robot model (scale 1.0 = no scaling, padding 0.0 = no padding). Elements not present in this list are scaled and
  //! padded with default_shadow_inflation_.
  std::map<std::string, ScaleAndPadding> per_link_shadow_inflation_;

  //! Inflation that is applied to a collision element for the purposes of computing the bounding sphere.
  //! Elements not present in this list are scaled and padded with default_bsphere_inflation_.
  std::map<std::string, ScaleAndPadding> per_link_bsphere_inflation_;

  //! Inflation that is applied to a collision element for the purposes of computing the bounding box.
  //! Elements not present in this list are scaled and padded with default_bbox_inflation_.
  std::map<std::string, ScaleAndPadding> per_link_bbox_inflation_;

  //! Name of the topic where the robot model is published.
  std::string robot_description_topic_;

  //! Callback for parameter updates.
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

  std::set<std::string> links_ignored_in_bounding_sphere_;
  std::set<std::string> links_ignored_in_bounding_box_;
  std::set<std::string> links_ignored_in_contains_test_;
  std::set<std::string> links_ignored_in_shadow_test_;
  std::set<std::string> links_ignored_everywhere_;
  std::set<std::string> only_links_;

  //! Publisher of robot bounding sphere (relative to fixed frame).
  rclcpp::Publisher<robot_body_filter::msg::SphereStamped>::SharedPtr bounding_sphere_publisher_;
  //! Publisher of robot bounding box (relative to fixed frame).
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr bounding_box_publisher_;
  //! Publisher of robot bounding box (relative to fixed frame).
  rclcpp::Publisher<robot_body_filter::msg::OrientedBoundingBoxStamped>::SharedPtr oriented_bounding_box_publisher_;
  //! Publisher of robot bounding box (relative to defined local frame).
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr local_bounding_box_publisher_;
  //! Publisher of the bounding sphere marker.
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr bounding_sphere_marker_publisher_;
  //! Publisher of the bounding box marker.
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr bounding_box_marker_publisher_;
  //! Publisher of the oriented bounding box marker.
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr oriented_bounding_box_marker_publisher_;
  //! Publisher of the local bounding box marker.
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr local_bounding_box_marker_publisher_;
  //! Publisher of the debug bounding box markers.
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr bounding_box_debug_marker_publisher_;
  //! Publisher of the debug oriented bounding box markers.
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr oriented_bounding_box_debug_marker_publisher_;
  //! Publisher of the debug local bounding box markers.
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr local_bounding_box_debug_marker_publisher_;
  //! Publisher of the debug bounding sphere markers.
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr bounding_sphere_debug_marker_publisher_;

  //! Publisher of scan_point_cloud with robot bounding box cut out.
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_point_cloud_no_bounding_box_publisher_;
  //! Publisher of scan_point_cloud with robot oriented bounding box cut out.
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_point_cloud_no_oriented_bounding_box_publisher_;
  //! Publisher of scan_point_cloud with robot local bounding box cut out.
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_point_cloud_no_local_bounding_box_publisher_;
  //! Publisher of scan_point_cloud with robot bounding sphere cut out.
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scan_point_cloud_no_bounding_sphere_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_point_cloud_inside_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_point_cloud_clip_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debug_point_cloud_shadow_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_contains_marker_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_shadow_marker_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_bsphere_marker_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_bbox_marker_publisher_;

  //! Service server for reloading robot model.
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reload_robot_model_service_server_;

  //! Subscriber for robot model.
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr reload_robot_model_subscriber_;

  //! Whether to compute bounding sphere of the robot.
  bool compute_bounding_sphere_;
  //! Whether to compute debug bounding sphere of the robot.
  bool compute_debug_bounding_sphere_;
  //! Whether to compute bounding box of the robot.
  bool compute_bounding_box_;
  //! Whether to compute debug bounding box of the robot.
  bool compute_debug_bounding_box_;
  //! Whether to compute oriented bounding box of the robot.
  bool compute_oriented_bounding_box_;
  //! Whether to compute debug oriented bounding box of the robot.
  bool compute_debug_oriented_bounding_box_;
  //! Whether to compute local bounding box of the robot.
  bool compute_local_bounding_box_;
  //! Whether to compute debug local bounding box of the robot.
  bool compute_debug_local_bounding_box_;
  //! Whether to publish the bounding box marker.
  bool publish_bounding_box_marker_;
  //! Whether to publish the bounding box marker.
  bool publish_oriented_bounding_box_marker_;
  //! Whether to publish the bounding box marker.
  bool publish_local_bounding_box_marker_;
  //! Whether to publish the bounding sphere marker.
  bool publish_bounding_sphere_marker_;
  //! Whether to publish scan_point_cloud with robot bounding box cut out.
  bool publish_no_bounding_box_pointcloud_;
  //! Whether to publish scan_point_cloud with robot oriented bounding box cut out.
  bool publish_no_oriented_bounding_box_pointcloud_;
  //! Whether to publish scan_point_cloud with robot local bounding box cut out.
  bool publish_no_local_bounding_box_pointcloud_;
  //! Whether to publish scan_point_cloud with robot bounding sphere cut out.
  bool publish_no_bounding_sphere_pointcloud_;

  //! The frame in which local bounding box should be computed.
  std::string local_bounding_box_frame_;

  bool publish_debug_pcl_inside_;
  bool publish_debug_pcl_clip_;
  bool publish_debug_pcl_shadow_;
  bool publish_debug_contains_marker_;
  bool publish_debug_shadow_marker_;
  bool publish_debug_bsphere_marker_;
  bool publish_debug_bbox_marker_;

  //! Timeout for reachable transforms.
  rclcpp::Duration reachable_transform_timeout_;
  //! Timeout for unreachable transforms.
  rclcpp::Duration unreachable_transform_timeout_;

  //! Whether to process data when there are some unreachable frames.
  bool require_all_frames_reachable_;

  //! Tf prefix for link name
  std::string link_tf_prefix_;

  //! A mutex that has to be locked in order to work with shapes_to_links_ or tf_buffer_.
  std::shared_ptr<std::mutex> model_mutex_;

  //! The string representation of the currently used robot model.
  std::string robot_description_string_;

  //! tf buffer length
  rclcpp::Duration tf_buffer_length_;
  //! tf client
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  //! tf listener
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  //! Watchdog for unreachable frames.
  std::shared_ptr<TFFramesWatchdog> tf_frames_watchdog_;

  //! The time when the filter configuration has finished.
  rclcpp::Time time_configured_;

  //! Tool for masking out 3D bodies out of point clouds.
  std::unique_ptr<RayCastingShapeMask> shape_mask_;

  /// A map that correlates shapes in robot_shape_mask to collision links in URDF.
  /** Keys are shape handles from shape_mask_. */
  std::map<point_containment_filter::ShapeHandle, CollisionBodyWithLink> shapes_to_links_;

  std::set<point_containment_filter::ShapeHandle> shapes_ignored_in_bounding_sphere_;
  std::set<point_containment_filter::ShapeHandle> shapes_ignored_in_bounding_box_;

  /**
   * \brief Caches any link->fixedFrame transforms after a scan message is received. Is queried by robot_shape_mask.
   *        Keys are `CollisionBodyWithLink::cacheKey`.
   */
  std::map<std::string, std::shared_ptr<Eigen::Isometry3d>> transform_cache_;
  /**
   * \brief Caches any link->fixedFrame transforms at the time of scan end. Only used for pointByPoint scans.
   *        Is queried by robot_shape_mask. Keys are `CollisionBodyWithLink::cache_key`.
   */
  std::map<std::string, std::shared_ptr<Eigen::Isometry3d>> transform_cache_after_scan_;

  //! If the scan is pointByPoint, set this variable to the ratio between scan start and end time you're looking for
  //! with getShapeTransform().
  mutable double cache_lookup_between_scans_ratio_;

  //! Used in tests. If false, configure() waits until robot description becomes available. If true,
  //! configure() fails with std::runtime_exception if robot description is not available.
  bool fail_without_robot_description_ = false;

  /**
   * \brief Perform the actual computation of mask.
   * \param[in] projected_point_cloud The input pointcloud. For clouds with each point captured at different time, it
   *                                  needs a float32 "stamps" channel and viewpoint channels vp_x, vp_y and vp_z.
   *                                  The stamps channel contains timestamps relative to the time in header.
   * \param[out] point_mask Output mask of the points.
   * \param[in] sensor_frame Sensor frame id. Only needed for scans with all points
   *                         captured at the same time. Point-by-point scans read
   *                         sensor position from the viewpoint channels.
   * \return Whether the computation succeeded.
   */
  bool computeMask(const sensor_msgs::msg::PointCloud2& projected_point_cloud,
                   std::vector<RayCastingShapeMask::MaskValue>& point_mask,
                   const std::string& sensor_frame = "");

  /**
   * \brief Return the latest cached transform for the link corresponding to the given shape handle.
   *
   * You should call updateTransformCache before calling this function.
   *
   * \param[in] shape_handle The handle of the shape for which we want the transform.
   *                         The handle is from robot_shape_mask.
   * \param[out] transform Transform of the corresponding link (wrt filtering frame).
   * \return If the transform was found.
   */
  bool getShapeTransform(point_containment_filter::ShapeHandle shape_handle, Eigen::Isometry3d& transform) const;

  /**
   * \brief Update robot_shape_mask with the given URDF model.
   *
   * \param[in] urdf_model The robot's URDF loaded as a string.
   */
  void addRobotMaskFromUrdf(const std::string& urdf_model);

  /**
   * \brief Remove all parts of the robot mask and clear internal shape and TF buffers.
   *
   * Make sure no filtering happens when executing this function.
   */
  void clearRobotMask();

  /**
   * \brief Update the cache of link transforms relative to filtering frame.
   *
   * \param[in] time The time to get transforms for.
   * \param[in] after_scan_time The after scan time to get transforms for
   *                          (if zero time is passed, after scan transforms are not computed).
   */
  void updateTransformCache(const rclcpp::Time& time, const rclcpp::Time& after_scan_time = rclcpp::Time(0));

  /**
   * \brief Callback handling update of the parameters.
   *
   * \param parameters The updated config.
   */
  rcl_interfaces::msg::SetParametersResult paramUpdateCallback(const std::vector<rclcpp::Parameter>& parameters);

  /**
   * \brief Callback for ~reload_model service. Reloads the URDF from parameter.
   */
  void onRobotModelMsg(const std_msgs::msg::String::ConstSharedPtr& msg);

  /**
   * \brief Callback for ~reload_model service. Reloads the URDF from parameter.
   */
  void triggerModelReload(std::shared_ptr<rmw_request_id_t>,
                          std::shared_ptr<std_srvs::srv::Trigger::Request>,
                          std::shared_ptr<std_srvs::srv::Trigger::Response>);

  void createBodyVisualizationMsg(
    const std::map<point_containment_filter::ShapeHandle, const bodies::Body*>& bodies,
    const rclcpp::Time& stamp,
    const std_msgs::msg::ColorRGBA& color,
    visualization_msgs::msg::MarkerArray& marker_array) const;

  void publishDebugMarkers(const rclcpp::Time& scan_time) const;

  void publishDebugPointClouds(
    const sensor_msgs::msg::PointCloud2& projected_point_cloud,
    const std::vector<RayCastingShapeMask::MaskValue>& point_mask) const;

  /**
   * \brief Computation of the bounding sphere, debug spheres, and publishing of
   *        pointcloud without bounding sphere.
   */
  void computeAndPublishBoundingSphere(const sensor_msgs::msg::PointCloud2& projected_point_cloud) const;

  /**
   * \brief Computation of the bounding box, debug boxes, and publishing of
   *        pointcloud without bounding box.
   */
  void computeAndPublishBoundingBox(const sensor_msgs::msg::PointCloud2& projected_point_cloud) const;

  /**
   * \brief Computation of the oriented bounding box, debug boxes, and publishing of
   *        pointcloud without bounding box.
   */
  void computeAndPublishOrientedBoundingBox(const sensor_msgs::msg::PointCloud2& projected_point_cloud) const;

  /**
   * \brief Computation of the local bounding box, debug boxes, and publishing of
   *        pointcloud without bounding box.
   */
  void computeAndPublishLocalBoundingBox(const sensor_msgs::msg::PointCloud2& projected_point_cloud) const;

  ScaleAndPadding getLinkInflationForContainsTest(const std::string& link_name) const;

  ScaleAndPadding getLinkInflationForContainsTest(const std::vector<std::string>& link_names) const;

  ScaleAndPadding getLinkInflationForShadowTest(const std::string& link_name) const;

  ScaleAndPadding getLinkInflationForShadowTest(const std::vector<std::string>& link_names) const;

  ScaleAndPadding getLinkInflationForBoundingSphere(const std::string& link_name) const;

  ScaleAndPadding getLinkInflationForBoundingSphere(const std::vector<std::string>& link_names) const;

  ScaleAndPadding getLinkInflationForBoundingBox(const std::string& link_name) const;

  ScaleAndPadding getLinkInflationForBoundingBox(const std::vector<std::string>& link_names) const;

private:
  ScaleAndPadding getLinkInflation(
    const std::vector<std::string>& link_names, const ScaleAndPadding& default_inflation,
    const std::map<std::string, ScaleAndPadding>& per_link_inflation) const;

  std::string getLinkTfPrefix() const;

  rclcpp::Node::SharedPtr own_node_handle_;

  std::unique_ptr<urdf::Model> parsed_urdf_model_;
};

class RobotBodyFilterLaserScan : public RobotBodyFilter<sensor_msgs::msg::LaserScan> {
public:
  //! Apply the filter.
  bool update(const sensor_msgs::msg::LaserScan& input_scan, sensor_msgs::msg::LaserScan& filtered_scan) override;

protected:
  bool configure() override;

  laser_geometry::LaserProjection laser_projector_;

  // in RobotBodyFilterLaserScan::update we project the scan to a pointcloud with viewpoints
  const std::unordered_map<std::string, cras::CloudChannelType> channels_to_transform_ {
    {"vp_", cras::CloudChannelType::POINT}
  };
};

class RobotBodyFilterPointCloud2 : public RobotBodyFilter<sensor_msgs::msg::PointCloud2> {
public:
  //! Apply the filter.
  bool update(const sensor_msgs::msg::PointCloud2& input_cloud, sensor_msgs::msg::PointCloud2& filtered_cloud) override;

protected:
  bool configure() override;

  /** \brief Frame into which the output data should be transformed. */
  std::string output_frame_;

  std::unordered_map<std::string, cras::CloudChannelType> channels_to_transform_;
};

}  // namespace robot_body_filter
