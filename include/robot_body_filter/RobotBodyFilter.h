#ifndef ROBOT_BODY_FILTER_ROBOTSELFFILTER_H_
#define ROBOT_BODY_FILTER_ROBOTSELFFILTER_H_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/filters/crop_box.h>

#include <rclcpp/rclcpp.hpp>
// #include <robot_body_filter/utils/filter_utils.hpp>
// #include <robot_body_filter/robot_body_filter/msg/oriented_bounding_box_stamped.hpp>
#include <robot_body_filter/utils/tf2_sensor_msgs.h>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <robot_body_filter/RayCastingShapeMask.h>
#include <moveit/occupancy_map_monitor/occupancy_map_updater.h>
#include <moveit/robot_model/aabb.h>
#include <urdf/model.h>
#include <laser_geometry/laser_geometry.hpp>
#include <geometric_shapes/mesh_operations.h>
// #include <geometry_msgs/Point32.h>
// #include <geometry_msgs/PolygonStamped.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
// #include <dynamic_reconfigure/Config.h>
// #include <robot_body_filter/SphereStamped.h>
// #include <robot_body_filter/OrientedBoundingBoxStamped.h>
// #include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/string.hpp>

#include <robot_body_filter/TfFramesWatchdog.h>

// Remove?
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <filters/filter_base.hpp>
#include <geometric_shapes/body_operations.h>
// #include <geometry_msgs/msg/detail/point32__struct.hpp>
// #include <geometry_msgs/msg/detail/point_stamped__struct.hpp>
// #include <geometry_msgs/msg/detail/polygon_stamped__struct.hpp>

#include <visualization_msgs/msg/detail/marker__struct.hpp>

namespace robot_body_filter {
/**
* \brief Just a helper structure holding together a link, one of its collision elements,
 * and the index of the collision element in the collision array of the link.
 */
struct CollisionBodyWithLink {
  urdf::CollisionSharedPtr collision;
  urdf::LinkSharedPtr link;
  size_t indexInCollisionArray;
  MultiShapeHandle multiHandle;
  std::string cacheKey;

  CollisionBodyWithLink() :
      indexInCollisionArray(0), cacheKey("__empty__") {
  }

  CollisionBodyWithLink(urdf::CollisionSharedPtr collision,
                        urdf::LinkSharedPtr link,
                        const size_t indexInCollisionArray,
                        const MultiShapeHandle& multiHandle):
      collision(collision), link(link), indexInCollisionArray(indexInCollisionArray),
      multiHandle(multiHandle)
  {
    std::ostringstream stream;
    stream << link->name << "-" << indexInCollisionArray;
    this->cacheKey = stream.str();
  }
};

struct ScaleAndPadding
{
  double scale;
  double padding;
  ScaleAndPadding(double scale = 1.0, double padding = 0.0);

  bool operator==(const ScaleAndPadding& other) const;
  bool operator!=(const ScaleAndPadding& other) const;
};

/** \brief Suffix added to link/collision names to distinguish their usage in contains tests only. */
static const std::string CONTAINS_SUFFIX = "::contains";
/** \brief Suffix added to link/collision names to distinguish their usage in shadow tests only. */
static const std::string SHADOW_SUFFIX = "::shadow";
/** \brief Suffix added to link/collision names to distinguish their usage in bounding sphere computation only. */
static const std::string BSPHERE_SUFFIX = "::bounding_sphere";
/** \brief Suffix added to link/collision names to distinguish their usage in bounding box computation only. */
static const std::string BBOX_SUFFIX = "::bounding_box";

/**
 * \brief Filter to remove robot's own body from laser scan.
 *
 * See readme for more information.
 *
 * \author Martin Pecka
 */
template<typename T>
class RobotBodyFilter : public filters::FilterBase<T> {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  RobotBodyFilter(std::shared_ptr<rclcpp::Node> inputNode = std::make_shared<rclcpp::Node>("robot_body_filter"));
  ~RobotBodyFilter() override;

  void DeclareParameters();
  //! Read config parameters loaded by FilterBase::configure(string, NodeHandle)
  //! Parameters are described in the readme.
  bool configure() override;

  bool update(const T& data_in, T& data_out) override = 0;

protected:
  //! Handle of the node this filter runs in.
  rclcpp::Node::SharedPtr nodeHandle;
  rclcpp::Node privateNodeHandle;

  /** \brief If true, suppose that every point in the scan was captured at a
   * different time instant. Otherwise, the scan is assumed to be taken at once.
   *
   * \note Always true for T = LaserScan.
   *
   * \note If this is true and T = PointCloud2, the processing pipeline expects
   * the pointcloud to have fields int32 index, float32 stamps, and float32
   * vp_x, vp_y and vp_z viewpoint positions. If one of these fields is missing,
   * computeMask() throws runtime exception.
   */
  bool pointByPointScan;

  //! Whether to keep pointcloud organized or not (if not, invalid points are
  //! removed).
  bool keepCloudsOrganized;

  /** \brief The interval between two consecutive model pose updates when
   * processing a pointByPointScan. If set to zero, the model will be updated
   * for each point separately (might be computationally exhaustive). If
   * non-zero, it will only update the model once in this interval, which makes
   * the masking algorithm a little bit less precise but more computationally
   * affordable.
   */
  rclcpp::Duration modelPoseUpdateInterval;

  /** \brief Fixed frame wrt the sensor frame.
   * Usually base_link for stationary robots (or sensor frame if both
   * robot and sensor are stationary). For mobile robots, it can be e.g.
   * odom or map. */
  std::string fixedFrame;

  /** \brief Frame of the sensor. For LaserScan version, it is automatically
   * read from the incoming data. For PointCloud2, you have to specify it
   * explicitly because the pointcloud could have already been transformed e.g.
   * to the fixed frame.
   */
  std::string sensorFrame;

  /** \brief Frame in which the filter is applied. For point-by-point scans, it
   * has to be a fixed frame, otherwise, it can be the sensor frame. */
  std::string filteringFrame;

  //! The minimum distance of points from the sensor to keep them (in meters).
  double minDistance;

  //! The maximum distance of points from the sensor origin to apply this filter on (in meters).
  double maxDistance;

  //! The default inflation that is applied to the collision model for the purposes of checking if a point is contained
  //! by the robot model (scale 1.0 = no scaling, padding 0.0 = no padding). Every collision element is scaled
  //! individually with the scaling center in its origin. Padding is added individually to every collision element.
  ScaleAndPadding defaultContainsInflation;

  //! The default inflation that is applied to the collision model for the purposes of checking if a point is shadowed
  //! by the robot model (scale 1.0 = no scaling, padding 0.0 = no padding). Every collision element is scaled
  //! individually with the scaling center in its origin. Padding is added individually to every collision element.
  ScaleAndPadding defaultShadowInflation;

  //! The default inflation that is applied to the collision model for the purposes of computing the bounding sphere.
  //! Every collision element is scaled individually with the scaling center in its origin. Padding is added
  //! individually to every collision element.
  ScaleAndPadding defaultBsphereInflation;

  //! The default inflation that is applied to the collision model for the purposes of computing the bounding box.
  //! Every collision element is scaled individually with the scaling center in its origin. Padding is added
  //! individually to every collision element.
  ScaleAndPadding defaultBboxInflation;

  //! Inflation that is applied to a collision element for the purposes of checking if a point is contained by the
  //! robot model (scale 1.0 = no scaling, padding 0.0 = no padding). Elements not present in this list are scaled and
  //! padded with defaultContainsInflation.
  std::map<std::string, ScaleAndPadding> perLinkContainsInflation;

  //! Inflation that is applied to a collision element for the purposes of checking if a point is shadowed by the
  //! robot model (scale 1.0 = no scaling, padding 0.0 = no padding). Elements not present in this list are scaled and
  //! padded with defaultShadowInflation.
  std::map<std::string, ScaleAndPadding> perLinkShadowInflation;

  //! Inflation that is applied to a collision element for the purposes of computing the bounding sphere.
  //! Elements not present in this list are scaled and padded with defaultBsphereInflation.
  std::map<std::string, ScaleAndPadding> perLinkBsphereInflation;

  //! Inflation that is applied to a collision element for the purposes of computing the bounding box.
  //! Elements not present in this list are scaled and padded with defaultBboxInflation.
  std::map<std::string, ScaleAndPadding> perLinkBboxInflation;

  //! Name of the parameter where the robot model can be found.
  std::string robotDescriptionParam;

  //! Subscriber for robot_description updates.
  std::shared_ptr<rclcpp::AsyncParametersClient> robotDescriptionUpdatesListener;
  // Callback group for dynamic updates
  rclcpp::CallbackGroup::SharedPtr AsyncCallbackGroup;

  //! Name of the field in the dynamic reconfigure message that contains robot model.
  std::string robotDescriptionUpdatesFieldName;

  std::set<std::string> linksIgnoredInBoundingSphere;
  std::set<std::string> linksIgnoredInBoundingBox;
  std::set<std::string> linksIgnoredInContainsTest;
  std::set<std::string> linksIgnoredInShadowTest;
  std::set<std::string> linksIgnoredEverywhere;
  std::set<std::string> onlyLinks;

  //! Publisher of robot bounding sphere (relative to fixed frame).
  // ros::Publisher boundingSpherePublisher;
  // They use a custom msg type, we can just use an unstamped std shape_msg for simplicity
  rclcpp::Publisher<shape_msgs::msg::SolidPrimitive>::SharedPtr boundingSpherePublisher;
  // //! Publisher of robot bounding box (relative to fixed frame).
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr boundingBoxPublisher;
  // //! Publisher of robot bounding box (relative to fixed frame).
  rclcpp::Publisher<shape_msgs::msg::SolidPrimitive>::SharedPtr orientedBoundingBoxPublisher;
  // //! Publisher of robot bounding box (relative to defined local frame).
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr localBoundingBoxPublisher;
  // //! Publisher of the bounding sphere marker.
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr boundingSphereMarkerPublisher;
  // //! Publisher of the bounding box marker.
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr boundingBoxMarkerPublisher;
  // //! Publisher of the oriented bounding box marker.
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr orientedBoundingBoxMarkerPublisher;
  //! Publisher of the local bounding box marker.
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr localBoundingBoxMarkerPublisher;
  //! Publisher of the debug bounding box markers.
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr boundingBoxDebugMarkerPublisher;
  //! Publisher of the debug oriented bounding box markers.
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr orientedBoundingBoxDebugMarkerPublisher;
  //! Publisher of the debug local bounding box markers.
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr localBoundingBoxDebugMarkerPublisher;
  //! Publisher of the debug bounding sphere markers.
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr boundingSphereDebugMarkerPublisher;

  //! Publisher of scan_point_cloud with robot bounding box cut out.
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scanPointCloudNoBoundingBoxPublisher;
  //! Publisher of scan_point_cloud with robot oriented bounding box cut out.
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scanPointCloudNoOrientedBoundingBoxPublisher;
  //! Publisher of scan_point_cloud with robot local bounding box cut out.
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scanPointCloudNoLocalBoundingBoxPublisher;
  //! Publisher of scan_point_cloud with robot bounding sphere cut out.
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr scanPointCloudNoBoundingSpherePublisher;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debugPointCloudInsidePublisher;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debugPointCloudClipPublisher;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr debugPointCloudShadowPublisher;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debugContainsMarkerPublisher;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debugShadowMarkerPublisher;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debugBsphereMarkerPublisher;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debugBboxMarkerPublisher;

  //! Service server for reloading robot model.
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reloadRobotModelServiceServer;

  //! Whether to compute bounding sphere of the robot.
  bool computeBoundingSphere;
  //! Whether to compute debug bounding sphere of the robot.
  bool computeDebugBoundingSphere;
  //! Whether to compute bounding box of the robot.
  bool computeBoundingBox;
  //! Whether to compute debug bounding box of the robot.
  bool computeDebugBoundingBox;
  //! Whether to compute oriented bounding box of the robot.
  bool computeOrientedBoundingBox;
  //! Whether to compute debug oriented bounding box of the robot.
  bool computeDebugOrientedBoundingBox;
  //! Whether to compute local bounding box of the robot.
  bool computeLocalBoundingBox;
  //! Whether to compute debug local bounding box of the robot.
  bool computeDebugLocalBoundingBox;
  //! Whether to publish the bounding box marker.
  bool publishBoundingBoxMarker;
  //! Whether to publish the bounding box marker.
  bool publishOrientedBoundingBoxMarker;
  //! Whether to publish the bounding box marker.
  bool publishLocalBoundingBoxMarker;
  //! Whether to publish the bounding sphere marker.
  bool publishBoundingSphereMarker;
  //! Whether to publish scan_point_cloud with robot bounding box cut out.
  bool publishNoBoundingBoxPointcloud;
  //! Whether to publish scan_point_cloud with robot oriented bounding box cut out.
  bool publishNoOrientedBoundingBoxPointcloud;
  //! Whether to publish scan_point_cloud with robot local bounding box cut out.
  bool publishNoLocalBoundingBoxPointcloud;
  //! Whether to publish scan_point_cloud with robot bounding sphere cut out.
  bool publishNoBoundingSpherePointcloud;

  //! The frame in which local bounding box should be computed.
  std::string localBoundingBoxFrame;

  bool publishDebugPclInside;
  bool publishDebugPclClip;
  bool publishDebugPclShadow;
  bool publishDebugContainsMarker;
  bool publishDebugShadowMarker;
  bool publishDebugBsphereMarker;
  bool publishDebugBboxMarker;

  //! Timeout for reachable transforms.
  rclcpp::Duration reachableTransformTimeout;
  //! Timeout for unreachable transforms.
  rclcpp::Duration unreachableTransformTimeout;

  //! Whether to process data when there are some unreachable frames.
  bool requireAllFramesReachable;

  //! A mutex that has to be locked in order to work with shapesToLinks or tfBuffer.
  std::shared_ptr<std::mutex> modelMutex;

  //! tf buffer length
  rclcpp::Duration tfBufferLength;
  //! tf client
  std::shared_ptr<tf2_ros::Buffer> tfBuffer;
  //! tf listener
  std::shared_ptr<tf2_ros::TransformListener> tfListener;

  //! Watchdog for unreachable frames.
  std::shared_ptr<TFFramesWatchdog> tfFramesWatchdog;

  //! The time when the filter configuration has finished.
  rclcpp::Time timeConfigured;

  //! Tool for masking out 3D bodies out of point clouds.
  std::unique_ptr<RayCastingShapeMask> shapeMask;

  /// A map that correlates shapes in robot_shape_mask to collision links in URDF.
  /** Keys are shape handles from shapeMask. */
  std::map<point_containment_filter::ShapeHandle, CollisionBodyWithLink> shapesToLinks;

  std::set<point_containment_filter::ShapeHandle> shapesIgnoredInBoundingSphere;
  std::set<point_containment_filter::ShapeHandle> shapesIgnoredInBoundingBox;

  //! Caches any link->fixedFrame transforms after a scan message is received. Is queried by robot_shape_mask. Keys are CollisionBodyWithLink#cacheKey.
  std::map<std::string, std::shared_ptr<Eigen::Isometry3d> > transformCache;
  //! Caches any link->fixedFrame transforms at the time of scan end. Only used for pointByPoint scans. Is queried by robot_shape_mask. Keys are CollisionBodyWithLink#cacheKey.
  std::map<std::string, std::shared_ptr<Eigen::Isometry3d> > transformCacheAfterScan;

  //! If the scan is pointByPoint, set this variable to the ratio between scan start and end time you're looking for with getShapeTransform().
  mutable double cacheLookupBetweenScansRatio;

  //! Used in tests. If false, configure() waits until robot description becomes available. If true,
  //! configure() fails with std::runtime_exception if robot description is not available.
  bool failWithoutRobotDescription = false;

  /**
   * \brief Perform the actual computation of mask.
   * \param projectedPointCloud The input pointcloud. For clouds with each
   *                            point captured at different time, it needs
   *                            a float32 "stamps" channel and viewpoint
   *                            channels vp_x, vp_y and vp_z. The stamps channel
   *                            contains timestamps relative to the time in
   *                            header.
   * \param mask Output mask of the points.
   * \param sensorFrame Sensor frame id. Only needed for scans with all points
   *                    captured at the same time. Point-by-point scans read
   *                    sensor position from the viewpoint channels.
   * \return Whether the computation succeeded.
   */
  bool computeMask(const sensor_msgs::msg::PointCloud2& projectedPointCloud,
                   std::vector<RayCastingShapeMask::MaskValue>& mask,
                   const std::string& sensorFrame = "");

  /** \brief Return the latest cached transform for the link corresponding to the given shape handle.
   *
   * You should call updateTransformCache before calling this function.
   *
   * \param shapeHandle The handle of the shape for which we want the transform. The handle is from robot_shape_mask.
   * \param[out] transform Transform of the corresponding link (wrt filtering frame).
   * \return If the transform was found.
   */
  bool getShapeTransform(point_containment_filter::ShapeHandle shapeHandle, Eigen::Isometry3d& transform) const;

  /** \brief Update robot_shape_mask with the given URDF model.
   *
   * \param urdfModel The robot's URDF loaded as a string.
   */
  void addRobotMaskFromUrdf(const std::string& urdfModel);

  /**
   * \brief Remove all parts of the robot mask and clear internal shape and TF buffers.
   *
   * Make sure no filtering happens when executing this function.
   */
  void clearRobotMask();

  /** \brief Update the cache of link transforms relative to filtering frame.
   *
   * \param time The time to get transforms for.
   * \param afterScantime The after scan time to get transforms for (if zero time is passed, after scan transforms are not computed).
   */
  void updateTransformCache(const rclcpp::Time& time, const rclcpp::Time& afterScanTime = rclcpp::Time(0));

  /**
   * \brief Callback handling update of the robot_description parameter using dynamic reconfigure.
   *
   * \param msg The new configuration.
   */
  void robotDescriptionUpdated(const std_msgs::msg::String::SharedPtr msg);

  /**
   * \brief Callback for ~reload_model service. Reloads the URDF from parameter.
   * \return Success.
   */
  bool triggerModelReload(std_srvs::srv::Trigger_Request&, std_srvs::srv::Trigger_Response&);

  void createBodyVisualizationMsg(
      const std::map<point_containment_filter::ShapeHandle, const bodies::Body *>& bodies,
      const rclcpp::Time& stamp, const std_msgs::msg::ColorRGBA& color,
      visualization_msgs::msg::MarkerArray& markerArray) const;

  void publishDebugMarkers(const rclcpp::Time& scanTime) const;
  void publishDebugPointClouds(
      const sensor_msgs::msg::PointCloud2& projectedPointCloud,
      const std::vector<RayCastingShapeMask::MaskValue>& pointMask) const;
  /**
   * \brief Computation of the bounding sphere, debug spheres, and publishing of
   * pointcloud without bounding sphere.
   */
  void computeAndPublishBoundingSphere(const sensor_msgs::msg::PointCloud2& projectedPointCloud) const;

  /**
   * \brief Computation of the bounding box, debug boxes, and publishing of
   * pointcloud without bounding box.
   */
  void computeAndPublishBoundingBox(const sensor_msgs::msg::PointCloud2& projectedPointCloud) const;

  /**
   * \brief Computation of the oriented bounding box, debug boxes, and publishing of 
   * pointcloud without bounding box.
   */
  void computeAndPublishOrientedBoundingBox(const sensor_msgs::msg::PointCloud2& projectedPointCloud) const;

  /**
   * \brief Computation of the local bounding box, debug boxes, and publishing of
   * pointcloud without bounding box.
   */
  void computeAndPublishLocalBoundingBox(const sensor_msgs::msg::PointCloud2& projectedPointCloud) const;

  ScaleAndPadding getLinkInflationForContainsTest(const std::string& linkName) const;
  ScaleAndPadding getLinkInflationForContainsTest(const std::vector<std::string>& linkNames) const;
  ScaleAndPadding getLinkInflationForShadowTest(const std::string& linkName) const;
  ScaleAndPadding getLinkInflationForShadowTest(const std::vector<std::string>& linkNames) const;
  ScaleAndPadding getLinkInflationForBoundingSphere(const std::string& linkName) const;
  ScaleAndPadding getLinkInflationForBoundingSphere(const std::vector<std::string>& linkNames) const;
  ScaleAndPadding getLinkInflationForBoundingBox(const std::string& linkName) const;
  ScaleAndPadding getLinkInflationForBoundingBox(const std::vector<std::string>& linkNames) const;

private:
  ScaleAndPadding getLinkInflation(const std::vector<std::string>& linkNames, const ScaleAndPadding& defaultInflation, const std::map<std::string, ScaleAndPadding>& perLinkInflation) const;
};

class RobotBodyFilterLaserScan : public RobotBodyFilter<sensor_msgs::msg::LaserScan>
{
public:
  void DeclareParameters();
  //! Apply the filter.
  bool update(const sensor_msgs::msg::LaserScan& inputScan, sensor_msgs::msg::LaserScan& filteredScan) override;

  bool configure() override;

protected:
  laser_geometry::LaserProjection laserProjector;

  // in RobotBodyFilterLaserScan::update we project the scan to a pointcloud with viewpoints
  const std::unordered_map<std::string, CloudChannelType> channelsToTransform { {"vp_", CloudChannelType::POINT} };
};

class RobotBodyFilterPointCloud2 : public RobotBodyFilter<sensor_msgs::msg::PointCloud2>
{
public:
  void DeclareParameters();
  //! Apply the filter.
  bool update(const sensor_msgs::msg::PointCloud2& inputCloud, sensor_msgs::msg::PointCloud2& filteredCloud) override;

  bool configure() override;

protected:
  /** \brief Frame into which the output data should be transformed. */
  std::string outputFrame;

  std::unordered_map<std::string, CloudChannelType> channelsToTransform;
};

}

#endif //ROBOT_BODY_FILTER_ROBOTSELFFILTER_H_
