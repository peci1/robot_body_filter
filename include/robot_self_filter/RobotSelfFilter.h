#ifndef ROBOT_SELF_FILTER_ROBOTSELFFILTER_H_
#define ROBOT_SELF_FILTER_ROBOTSELFFILTER_H_

#include <memory>
#include <mutex>
#include <set>
#include <thread>
#include <utility>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/filters/crop_box.h>

#include <ros/ros.h>
#include <robot_self_filter/utils/filter_utils.hpp>
#include <sensor_msgs/LaserScan.h>
#include <robot_self_filter/RayCastingShapeMask.h>
#include <moveit/occupancy_map_monitor/occupancy_map_updater.h>
#include <moveit/robot_model/aabb.h>
#include <urdf/model.h>
#include <laser_geometry/laser_geometry.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PolygonStamped.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <dynamic_reconfigure/Config.h>
#include <robot_self_filter/SphereStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_srvs/Trigger.h>

#include <robot_self_filter/TfFramesWatchdog.h>

namespace robot_self_filter {
/**
* \brief Just a helper structure holding together a link, one of its collision elements,
 * and the index of the collision element in the collision array of the link.
*/
struct CollisionBodyWithLink {
  urdf::CollisionSharedPtr collision;
  urdf::LinkSharedPtr link;
  size_t indexInCollisionArray;
  std::string cacheKey;

  CollisionBodyWithLink() :
      indexInCollisionArray(0), cacheKey("__empty__") {
  }

  CollisionBodyWithLink(urdf::CollisionSharedPtr collision,
                        urdf::LinkSharedPtr link,
                        const size_t indexInCollisionArray):
      collision(collision), link(link),
      indexInCollisionArray(indexInCollisionArray)
  {
    std::ostringstream stream;
    stream << link->name << "-" << indexInCollisionArray;
    this->cacheKey = stream.str();
  }
};

/**
 * \brief Filter to remove robot's own body from laser scans.
 *
 * Subscribed topics:
 * - /tf2, /tf2_static: transforms
 * - dynamic_robot_model_server/parameter_updates: Dynamic reconfigure topic on
 *       which updated robot model can be published. The model is read from a
 *       field defined by parameter dynamic_robot_description_field_name.
 *
 * Published topics:
 * - scan_point_cloud_no_bbox: Point cloud with point inside bounding box removed.
 * - scan_point_cloud_no_bsphere: Point cloud with point inside bounding sphere removed.
 * - robot_bounding_sphere: Bounding sphere of the robot body.
 * - robot_bounding_box: Bounding box of the robot body.
 * - robot_bounding_sphere_debug: Marker array containing the bounding sphere for
 *       each collision.
 * - robot_bounding_box_debug: Marker array containing the bounding box for
 *       each collision.
 *
 * Provided services:
 * - ~reload_model (std_srvs/Trigger): Call this service to re-read the URDF
 *       model from parameter server.
 *
 * \author Martin Pecka
 */
template<typename T>
class RobotSelfFilter : public ::robot_self_filter::FilterBase<T> {
public:
  RobotSelfFilter();
  ~RobotSelfFilter() override = default;

  //! Read config parameters loaded by FilterBase::configure(string, NodeHandle)
  /**
   * point_by_point_scan (only the PointCloud2 version):  If true, suppose that
   *     every point in the scan was captured at a different time instant.
   *     Otherwise, the scan is assumed to be taken at once. If this is true,
   *     the processing pipeline expects the pointcloud to have fields
   *     int32 index, float32 stamps, and float32 vp_x, vp_y and vp_z viewpoint
   *     positions. If one of these fields is missing, computeMask() throws
   *     runtime exception. Default: false.
   * fixed_frame: The fixed frame. Usually base_link for stationary robots
   *              (or sensor frame if both robot and sensor are stationary).
   *              For mobile robots, it can be e.g. odom or map.
   *              Default: base_link.
   * sensor_frame: Frame of the sensor. For LaserScan version, it is
   *               automatically read from the incoming data (this parameter has
   *               no effect). For PointCloud2, you have to specify it
   *               explicitly because the pointcloud could have already been
   *               transformed e.g. to the fixed frame. Default: "laser"
   * min_distance: The minimum distance of points from the laser to keep them
   *               (in meters). Default: 0.0 m
   * max_distance: The maximum distance of points from the laser to keep them
   *               (in meters). Set to zero to disable this limit. Default: 0.0 m
   * inflation_scale: A scale that is applied to the collision model for the
   *                  purposes of collision checking. Default: 1.0
   * inflation_padding: Padding to be added to the collision model for the
   *                    purposes of collision checking (meters). Default: 0.0 m
   * robot_description_param: Name of the parameter where the robot model can be
   *                          found. Default: robot_description
   * keep_clouds_organized: Whether to keep pointclouds organized if they were.
   *                        Organized cloud has height > 1. Default: true.
   * tf_buffer_length: Duration for which transforms will be stored in TF buffer.
   *                   Default: 60.0 seconds.
   * reachable_transform_timeout: How long to wait while getting reachable TF.
   *                              Default: 1.0 second.
   * unreachable_transform_timeout: How long to wait while getting unreachable
   *                                TF. Default: 0.1 second.
   * compute_bounding_sphere: Whether to compute bounding sphere. Default: false.
   * compute_bounding_box: Whether to compute bounding box. Default: false.
   * compute_debug_bounding_sphere: Whether to compute and publish debug
   *                                bounding sphere (marker array of spheres for
   *                                each collision). Default: false.
   * compute_debug_bounding_box: Whether to compute and publish debug
   *                             bounding box (marker array of boxes for
   *                             each collision). Default: false.
   * publish_no_bounding_box_pointcloud: Whether to compute and publish
   *                                     pointcloud from which points in the
   *                                     bounding box are removed. Will be
   *                                     published on scan_point_cloud_no_bbox.
   *                                     Implies compute_bounding_box.
   *                                     Default: false.
   * publish_no_bounding_sphere_pointcloud: Whether to compute and publish
   *                                        pointcloud from which points in the
   *                                        bounding sphere are removed. Will be
   *                                        published on scan_point_cloud_no_bsphere.
   *                                        Implies compute_bounding_sphere.
   *                                        Default: false.
   * links_ignored_in_bounding_sphere: List of links to be ignored in bounding
   *                                   sphere computation. Can be either names of
   *                                   links or a combination of link name and
   *                                   collision index, e.g. link::1,
   *                                   or link name and collision name, e.g.
   *                                   link::my_collision.
   *                                   Default: []
   * links_ignored_in_bounding_box: List of links to be ignored in bounding
   *                                box computation. Can be either names of
   *                                links or a combination of link name and
   *                                collision index, e.g. link::1,
   *                                or link name and collision name, e.g.
   *                                link::my_collision.
   *                                Default: []
   * links_ignored_in_contains_test: List of links to be ignored when testing
   *                                 if a point is inside robot body. Can be
   *                                 either names of links or a combination of
   *                                 link name and collision index, e.g.
   *                                 link::1, or link name and
   *                                 collision name, e.g. link::my_collision.
   *                                 Default: []
   * links_ignored_in_shadow_test: List of links to be ignored when testing
   *                               if a point is shadowed by robot body. Can be
   *                               either names of links or a combination of
   *                               link name and collision index, e.g.
   *                               link::1, or link name and collision
   *                               name, e.g. link::my_collision. It is
   *                               essential that this list contains the sensor
   *                               link - otherwise all points would be shadowed
   *                               by the sensor itself. Default: ['laser']
   * links_ignored_everywhere: List of links to be completely ignored. Can be
   *                               either names of links or a combination of
   *                               link name and collision index, e.g.
   *                               link::1, or link name and collision
   *                               name, e.g. link::my_collision. Default: []
   * dynamic_robot_description_field_name: If robot model is published by dynamic
   *                                       reconfigure, this is the name of the
   *                                       Config message field which holds the
   *                                       robot model: Default: robot_model
   */
  bool configure() override;

  bool update(const T& data_in, T& data_out) override = 0;

protected:

  //! Handle of the node this filter runs in.
  ros::NodeHandle nodeHandle;
  ros::NodeHandle privateNodeHandle;

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

  //! The minimum distance of points from the sensor to keep them (in meters).
  double minDistance;

  //! The maximum distance of points from the sensor origin to apply this filter on (in meters).
  double maxDistance;

  /// A scale that is applied to the collision model for the purposes of collision checking (1.0 = no scaling).
  /** Every collision element is scaled individually with the scaling center in its origin. */
  double inflationScale;

  /// A constant padding to be added to the collision model for the purposes of collision checking (in meters).
  /** It is added individually to every collision element. */
  double inflationPadding;

  //! Name of the parameter where the robot model can be found.
  std::string robotDescriptionParam;

  //! Subscriber for robot_description updates.
  ros::Subscriber robotDescriptionUpdatesListener;
  //! Name of the field in the dynamic reconfigure message that contains robot model.
  std::string robotDescriptionUpdatesFieldName;

  std::set<std::string> linksIgnoredInBoundingSphere;
  std::set<std::string> linksIgnoredInBoundingBox;
  std::set<std::string> linksIgnoredInContainsTest;
  std::set<std::string> linksIgnoredInShadowTest;
  std::set<std::string> linksIgnoredEverywhere;

  //! Publisher of robot bounding sphere (relative to fixed frame).
  ros::Publisher boundingSpherePublisher;
  //! Publisher of robot bounding box (relative to fixed frame).
  ros::Publisher boundingBoxPublisher;
  //! Publisher of the debug bounding box markers.
  ros::Publisher boundingBoxDebugMarkerPublisher;
  ros::Publisher boundingSphereDebugMarkerPublisher;

  //! Publisher of scan_point_cloud with robot bounding box cut out.
  ros::Publisher scanPointCloudNoBoundingBoxPublisher;
  //! Publisher of scan_point_cloud with robot bounding sphere cut out.
  ros::Publisher scanPointCloudNoBoundingSpherePublisher;
  ros::Publisher debugPointCloudInsidePublisher;
  ros::Publisher debugPointCloudClipPublisher;
  ros::Publisher debugPointCloudShadowPublisher;
  ros::Publisher debugContainsMarkerPublisher;
  ros::Publisher debugShadowMarkerPublisher;

  //! Service server for reloading robot model.
  ros::ServiceServer reloadRobotModelServiceServer;

  //! Whether to compute bounding sphere of the robot.
  bool computeBoundingSphere;
  //! Whether to compute debug bounding sphere of the robot.
  bool computeDebugBoundingSphere;
  //! Whether to compute bounding box of the robot.
  bool computeBoundingBox;
  //! Whether to compute debug bounding box of the robot.
  bool computeDebugBoundingBox;
  //! Whether to publish scan_point_cloud with robot bounding box cut out.
  bool publishNoBoundingBoxPointcloud;
  //! Whether to publish scan_point_cloud with robot bounding sphere cut out.
  bool publishNoBoundingSpherePointcloud;

  bool publishDebugPclInside;
  bool publishDebugPclClip;
  bool publishDebugPclShadow;
  bool publishDebugContainsMarker;
  bool publishDebugShadowMarker;

  //! Timeout for reachable transforms.
  ros::Duration reachableTransformTimeout;
  //! Timeout for unreachable transforms.
  ros::Duration unreachableTransformTimeout;

  //! A mutex that has to be locked in order to work with shapesToLinks or tfBuffer.
  std::shared_ptr<std::mutex> modelMutex;

  //! tf buffer length
  ros::Duration tfBufferLength;
  //! tf client
  std::shared_ptr<tf2_ros::Buffer> tfBuffer;
  //! tf listener
  std::unique_ptr<tf2_ros::TransformListener> tfListener;

  //! Watchdog for unreachable frames.
  std::shared_ptr<TFFramesWatchdog> tfFramesWatchdog;

  //! The time when the filter configuration has finished.
  ros::Time timeConfigured;

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
  double cacheLookupBetweenScansRatio;

  /**
   * \brief Perform the actual computation of mask.
   * \param projectedPointCloud The input pointcloud. Needs at least the
*                               int32 INDEX channel, and for clouds with each
   *                            point captured at different time, it also needs
   *                            a float32 "stamps" channel and viewpoint
   *                            channels vp_x, vp_y and vp_z.
   * \param mask Output mask of the points. Indices to the mask are taken from
   *             the INDEX channel.
   * \return Whether the computation succeeded.
   */
  bool computeMask(const sensor_msgs::PointCloud2& projectedPointCloud, std::vector<RayCastingShapeMask::MaskValue>& mask);

  /** \brief Return the latest cached transform for the link corresponding to the given shape handle.
   *
   * You should call updateTransformCache before calling this function.
   *
   * \param shapeHandle The handle of the shape for which we want the transform. The handle is from robot_shape_mask.
   * \param[out] transform Transform of the corresponding link (wrt robot_frame).
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

  /** \brief Update the cache of link transforms relative to robot_frame.
   *
   * \param time The time to get transforms for.
   * \param afterScantime The after scan time to get transforms for (if zero time is passed, after scan transforms are not computed).
   */
  void updateTransformCache(const ros::Time& time, const ros::Time& afterScanTime = ros::Time(0));

  /**
   * \brief Callback handling update of the robot_description parameter using dynamic reconfigure.
   *
   * \param newConfig The updated config.
   */
  void robotDescriptionUpdated(dynamic_reconfigure::ConfigConstPtr newConfig);

  /**
   * \brief Callback for ~reload_model service. Reloads the URDF from parameter.
   * \return Success.
   */
  bool triggerModelReload(std_srvs::TriggerRequest&, std_srvs::TriggerResponse&);

  void createBodyVisualizationMsg(
      const std::map<point_containment_filter::ShapeHandle, bodies::Body*>& bodies,
      const ros::Time& stamp, const std_msgs::ColorRGBA& color,
      visualization_msgs::MarkerArray& markerArray) const;

  void publishDebugMarkers(const ros::Time& scanTime) const;
  void publishDebugPointClouds(
      const sensor_msgs::PointCloud2& projectedPointCloud,
      const std::vector<RayCastingShapeMask::MaskValue> &pointMask) const;
  /**
   * \brief Computation of the bounding sphere, debug spheres, and publishing of
   * pointcloud without bounding sphere.
   */
  void computeAndPublishBoundingSphere(const sensor_msgs::PointCloud2& projectedPointCloud) const;

  /**
   * \brief Computation of the bounding box, debug boxes, and publishing of
   * pointcloud without bounding box.
   */
  void computeAndPublishBoundingBox(const sensor_msgs::PointCloud2& projectedPointCloud) const;
};

class RobotSelfFilterLaserScan : public RobotSelfFilter<sensor_msgs::LaserScan>
{
public:
  //! Apply the filter.
  bool update(const sensor_msgs::LaserScan &inputScan, sensor_msgs::LaserScan &filteredScan) override;

  virtual bool configure();
};

class RobotSelfFilterPointCloud2 : public RobotSelfFilter<sensor_msgs::PointCloud2>
{
public:
  //! Apply the filter.
  bool update(const sensor_msgs::PointCloud2 &inputCloud, sensor_msgs::PointCloud2 &filteredCloud) override;

  virtual bool configure();
};

}

#endif //ROBOT_SELF_FILTER_ROBOTSELFFILTER_H_
