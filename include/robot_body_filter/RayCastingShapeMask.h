#ifndef ROBOT_BODY_FILTER_RAYCASTINGSHAPEMASK_H
#define ROBOT_BODY_FILTER_RAYCASTINGSHAPEMASK_H

#include <vector>
#include <map>

#include <moveit/point_containment_filter/shape_mask.h>

#include <robot_body_filter/utils/bodies.h>
#include <robot_body_filter/utils/cloud.h>

namespace robot_body_filter
{

class RayCastingShapeMask : protected point_containment_filter::ShapeMask
{
public:

  enum class MaskValue : std::uint8_t {
    INSIDE = 0, //!< The point is inside robot body.
    OUTSIDE = 1, //!< The point is outside robot body and is not a shadow.
    CLIP = 2, //!< The point is outside measurement range.
    SHADOW = 3, //!< Line segment sensor-point intersects the robot body and the point is not INSIDE.
  };

  explicit RayCastingShapeMask(
      const TransformCallback& transformCallback,
      double minSensorDist = 0.0, double maxSensorDist = 1e10);

  virtual ~RayCastingShapeMask();

  point_containment_filter::ShapeHandle addShape(
      const shapes::ShapeConstPtr& shape, double scale = 1.0,
      double padding = 0.0, bool updateInternalStructures = true);
  void removeShape(point_containment_filter::ShapeHandle handle,
      bool updateInternalStructures = true);

  void setTransformCallback(const TransformCallback& transform_callback);

  /**
   * \brief Get the map of bounding spheres of all registered shapes.
   * \return The map of bounding spheres.
   */
  std::map<point_containment_filter::ShapeHandle, bodies::BoundingSphere>
  getBoundingSpheres() const;

  /**
   * \brief Get the map of axis-aligned bounding boxes of all registered shapes.
   * \return The map of bounding boxes.
   */
  std::map<point_containment_filter::ShapeHandle, bodies::AxisAlignedBoundingBox>
  getAxisAlignedBoundingBoxes() const;

  /**
   * \brief Get the bounding sphere containing all registered shapes.
   * \return The bounding sphere of the mask.
   */
  bodies::BoundingSphere getBoundingSphere() const;

  /**
   * \brief Update the poses of bodies_ and recompute corresponding bspheres_.
   * \note Calls transform_callback_
   */
  void updateBodyPoses();

  /**
   * \brief Update internal structures. Should be called whenever a shape is
   * added, removed or ignored. By default, it is called automatically by the
   * methods that affect the list of shapes. However, for efficiency, the caller
   * can tell these methods to not update the shape lists and call this when a
   * batch operation is finished.
   */
  void updateInternalShapeLists();

  /** \brief Decide whether the points in data are either INSIDE the robot,
   * OUTSIDE of it, SHADOWed by the robot body, or CLIPped by min/max sensor
   * measurement distance. INSIDE points can also be viewed as being SHADOW
   * points, but in this algorithm, INSIDE has precedence.
   *
   * \param [in] data The input pointcloud. It has to be in the same frame into
   *                  which transform_callback_ transforms the body parts.
   * \param [out] mask The mask value of all the points. Ordered by point index.
   * \param [in] sensorPos Position of the sensor in the pointcloud frame.
   *
   * \note Internally calls updateBodyPoses() to update link transforms.
   * \note Updates bspheres_/boundingBoxes to contain the bounding spheres/boxes of links.
  */
  void maskContainmentAndShadows(
      const Cloud& data,
      std::vector<MaskValue>& mask,
      const Eigen::Vector3f& sensorPos = Eigen::Vector3f::Zero());

  /** \brief Decide whether the point is either INSIDE the robot,
   * OUTSIDE of it, SHADOWed by the robot body, or CLIPped by min/max sensor
   * measurement distance. INSIDE points can also be viewed as being SHADOW
   * points, but in this algorithm, INSIDE has precedence.
   *
   * \param [in] data The input point. It has to be in the same frame
   *                  into which transform_callback_ transforms the body parts.
   * \param [out] mask The mask value of the given point.
   * \param [in] sensorPos Position of the sensor in the pointcloud frame.
   *
   *
   * \note Internally calls updateBodyPoses() to update link transforms.
   * \note Updates bspheres_/boundingBoxes to contain the bounding spheres/boxes of links.
  */
  void maskContainmentAndShadows(
      const Eigen::Vector3f& data,
      MaskValue& mask,
      const Eigen::Vector3f& sensorPos = Eigen::Vector3d::Zero(),
      bool updateBodyPoses = true);

  /**
   * \brief Set the shapes to be ignored when doing test for INSIDE in maskContainmentAndShadows.
   * \param ignoreInContainsTest The shapes to be ignored.
   * \param updateInternalStructures If false, the caller is responsible for calling updateInternalShapeLists().
   */
  void setIgnoreInContainsTest(
      std::set<point_containment_filter::ShapeHandle> ignoreInContainsTest,
      bool updateInternalStructures = true);

  /**
   * \brief Set the shapes to be ignored when doing test for SHADOW in maskContainmentAndShadows.
   * \param ignoreInShadowTest The shapes to be ignored.
   * \param updateInternalStructures If false, the caller is responsible for calling updateInternalShapeLists().
   * \note E.g. the sensor collision shape should be listed here.
   */
  void setIgnoreInShadowTest(
      std::set<point_containment_filter::ShapeHandle> ignoreInShadowTest,
      bool updateInternalStructures = true);

  std::map<point_containment_filter::ShapeHandle, bodies::Body*> getBodies() const;
  std::map<point_containment_filter::ShapeHandle, bodies::Body*> getBodiesForContainsTest() const;
  std::map<point_containment_filter::ShapeHandle, bodies::Body*> getBodiesForShadowTest() const;

protected:

  /**
   * \brief Update the poses of bodies_ and recompute corresponding bspheres_
   *        and boundingBoxes.
   * \note Calls transform_callback_
   * \note The caller has to hold a lock to shapes_mutex_.
   */
  void updateBodyPosesNoLock();

  /** \brief Decide whether the point is either INSIDE the robot,
   * OUTSIDE of it, SHADOWed by the robot body, or CLIPped by min/max sensor
   * measurement distance. INSIDE points can also be viewed as being SHADOW
   * points, but in this algorithm, INSIDE has precedence.
   *
   * \param [in] data The input point. It has to be in the same frame
   *                  into which transform_callback_ transforms the body parts.
   * \param [out] mask The mask value of the given point.
   * \param [in] sensorPos Position of the sensor in the pointcloud frame.
   * \param [in] boundingSphereForContainsTest Sphere containing all the shapes
   *                                           that should be taken into account
   *                                           during the contains test.
   *
   * \note Contrasting to maskContainmentAndShadows(), this method doesn't
   *       update link poses and expects them to be correctly updated by a prior
   *       call to updateBodyPoses(). It also doesn't lock shapes_mutex_.
  */
  void classifyPointNoLock(
      const Eigen::Vector3f& data,
      MaskValue &mask,
      const Eigen::Vector3f& sensorPos,
      const bodies::BoundingSphere &boundingSphereForContainsTest);

  /**
   * \brief Get the bounding sphere containing all registered shapes.
   * \return The bounding sphere of the mask.
   */
  bodies::BoundingSphere getBoundingSphereNoLock() const;

  /**
   * \brief Get the bounding sphere containing all shapes that are to be used
   *        in the contains test.
   * \return The bounding sphere of the mask.
   */
  bodies::BoundingSphere getBoundingSphereForContainsTestNoLock() const;

  double minSensorDist; //!< Minimum sensing distance of the sensor.
  double maxSensorDist; //!< Maximum sensing distance of the sensor.

  /** \brief Shapes to be ignored when doing test for INSIDE in maskContainmentAndShadows. */
  std::set<point_containment_filter::ShapeHandle> ignoreInContainsTest;
  /** \brief Shapes to be ignored when doing test for SHADOW in maskContainmentAndShadows.
   *  E.g. the sensor collision shape should be listed here. */
  std::set<point_containment_filter::ShapeHandle> ignoreInShadowTest;

  class RayCastingShapeMaskPIMPL;
  std::unique_ptr<RayCastingShapeMaskPIMPL> data;

  std::vector<size_t> bspheresBodyIndices;
  std::vector<bodies::BoundingSphere> bspheresForContainsTest;
  std::vector<bodies::AxisAlignedBoundingBox> boundingBoxes;
};

}

#endif //ROBOT_BODY_FILTER_RAYCASTINGSHAPEMASK_H
