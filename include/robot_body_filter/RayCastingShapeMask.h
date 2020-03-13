#ifndef ROBOT_BODY_FILTER_RAYCASTINGSHAPEMASK_H
#define ROBOT_BODY_FILTER_RAYCASTINGSHAPEMASK_H

#include <vector>
#include <map>

#include <moveit/point_containment_filter/shape_mask.h>

#include <robot_body_filter/utils/bodies.h>
#include <robot_body_filter/utils/cloud.h>
#include <sstream>  // has to be there, otherwise we encounter build problems
#include <robot_body_filter/RayCastingShapeMask.h>
#include <geometric_shapes/body_operations.h>
#include <ros/console.h>

namespace robot_body_filter
{

/**
 * \brief This class manages a set of bodies in space and is able to test point(cloud)s against this
 * set, whether the points are inside the set, shadowed by the set when viewed from a given sensor,
 * or clipped by the given sensor clipping distance.
 *
 * The mask works in the so-called filtering frame, which is any arbitrary TF frame.
 * The TransformCallback you need to provide should transform all bodies to the filtering frame.
 * All points passed to the masking functions are also expected to be in the filtering frame.
 *
 * Important tip: do not forget to include the sensor's body in setIgnoreInShadowTest() call,
 * otherwise all points will be shadowed by the sensor's body.
 *
 * Due to limitations of the C++ API, there are some methods of the parent ShapeMask class that are
 * forbidden to be used:
 *  - ShapeHandle addShape(const shapes::ShapeConstPtr&, double, double);
 *  - void removeShape(ShapeHandle handle);
 *  - void maskContainment(const sensor_msgs::PointCloud2&, const Eigen::Vector3d&, double, double,
 *                         std::vector<int>&);
 *  - int getMaskContainment(double, double, double) const;
 *  - int getMaskContainment(const Eigen::Vector3d&) const;
 *  - void setTransformCallback(const TransformCallback&); (this class shadows it and the shadow
 *    has to be used)
 *
 * Also, constants ShapeMask::INSIDE, ShapeMask::OUTSIDE, ShapeMask::CLIP should be avoided and
 * their RayCastingShapeMask:: counterparts should be used. In any case, the ShapeMask:: constants
 * are binary-compatible with the corresponding RayCastingShapeMask:: constants, so if you don't
 * need shadow filtering, you can keep using the ShapeMask:: constants (although it is strongly
 * discouraged).
 */
class RayCastingShapeMask : protected point_containment_filter::ShapeMask
{
public:

  enum class MaskValue : std::uint8_t {
    INSIDE = 0, //!< The point is inside robot body.
    OUTSIDE = 1, //!< The point is outside robot body and is not a shadow.
    CLIP = 2, //!< The point is outside measurement range.
    SHADOW = 3, //!< Line segment sensor-point intersects the robot body and the point is not INSIDE.
  };

  /**
   * \brief
   * \param transformCallback
   * \param minSensorDist
   * \param maxSensorDist
   * \param doClipping
   * \param doContainsTest
   * \param doShadowTest
   */
  explicit RayCastingShapeMask(
      const TransformCallback& transformCallback,
      double minSensorDist = 0.0, double maxSensorDist = 1e10,
      bool doClipping = true, bool doContainsTest = true,
      bool doShadowTest = true);

  virtual ~RayCastingShapeMask();

  /**
   * \brief Add the given shape to the set of filtered bodies. The internally created body will be
   * transformed by the TransformCallback to which the handle of this shape will be passed.
   * \param shape The shape to be filtered.
   * \param scale Scale of the shape.
   * \param padding Padding of the shape.
   * \param updateInternalStructures Set to true if only adding a single shape. If adding a batch of
   *        shapes, set this to false and call updateInternalStructures() manually at the end of the
   *        batch.
   * \param name Optional name of the shape. Used when reporting problems with the shape transforms.
   * \return A handle of the shape. It can be used for removing the shape, and will also be passed
   *         as the first argument of TransformCallback when asking for the transform of this
   *         shape's body.
   * \sa updateInternalShapeLists()
   */
  point_containment_filter::ShapeHandle addShape(
      const shapes::ShapeConstPtr& shape, double scale = 1.0,
      double padding = 0.0, bool updateInternalStructures = true,
      const std::string& name = "");

  /**
   * \brief Remove the shape identified by the given handle from the filtering mask.
   * \param handle The handle returned by addShape().
   * \param updateInternalStructures Set to true if only removing a single shape. If removing a
   *        batch of shapes, set this to false and call updateInternalStructures() manually at the
   *        end of the batch.
   * \sa updateInternalShapeLists()
   */
  void removeShape(point_containment_filter::ShapeHandle handle,
      bool updateInternalStructures = true);

  /**
   * \brief Set the callback which is called whenever a pose of a body needs to be updated.
   * \param transform_callback The callback. First argument is the handle of the shape whose
   *        corresponding body is being updated. The second argument is the resulting transform.
   *        Return false if getting the transform failed, otherwise return true.
   */
  void setTransformCallback(const TransformCallback& transform_callback);

  /**
   * \brief Get the map of bounding spheres of all registered shapes.
   * \return The map of bounding spheres.
   *
   * \note Is only updated during updateBodyPoses().
   */
  std::map<point_containment_filter::ShapeHandle, bodies::BoundingSphere>
  getBoundingSpheres() const;

  /**
   * \brief Get the bounding sphere containing all registered shapes.
   * \return The bounding sphere of the mask.
   *
   * \note Is only updated during updateBodyPoses().
   */
  bodies::BoundingSphere getBoundingSphere() const;

  /**
   * \brief Update the poses of bodies and recompute corresponding bounding
   *        spheres.
   * \note Calls the TransformCallback given in constructor or set by
   *       setTransformCallback(). One call per registered shape will be made.
   */
  void updateBodyPoses();

  /**
   * \brief Update internal structures. Should be called whenever a shape is
   * added, removed or ignored. By default, it is called automatically by the
   * methods that affect the list of shapes. However, for efficiency, the caller
   * can tell these methods to not update the shape lists and call this when a
   * batch operation is finished.
   *
   * \sa addShape()
   * \sa removeShape()
   */
  void updateInternalShapeLists();

  /** \brief Decide whether the points in data are either INSIDE the robot,
   * OUTSIDE of it, SHADOWed by the robot body, or CLIPped by min/max sensor
   * measurement distance. INSIDE points can also be viewed as being SHADOW
   * points, but in this algorithm, INSIDE has precedence.
   *
   * \param [in] data The input pointcloud. It has to be in the same frame into
   *                  which transform_callback_ transforms the body parts. The
   *                  frame_id of the cloud is ignored.
   * \param [out] mask The mask value of all the points. Ordered by point index.
   * \param [in] sensorPos Position of the sensor in the pointcloud frame.
   *
   * \note Internally calls updateBodyPoses() to update link transforms.
   * \note Updates bspheres_/boundingBoxes to contain the bounding spheres/boxes of links.
  */
  void maskContainmentAndShadows(
      const Cloud& data,
      std::vector<MaskValue>& mask,
      const Eigen::Vector3d& sensorPos = Eigen::Vector3d::Zero());

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
   * \note Internally calls updateBodyPoses() to update link transforms.
   * \note Updates bspheres_/boundingBoxes to contain the bounding spheres/boxes of links.
  */
  void maskContainmentAndShadows(
      const Eigen::Vector3f& data,
      MaskValue& mask,
      const Eigen::Vector3d& sensorPos = Eigen::Vector3d::Zero(),
      bool updateBodyPoses = true);

  /**
   * \brief Set the shapes to be ignored when doing test for INSIDE in maskContainmentAndShadows.
   * \param ignoreInContainsTest The shapes to be ignored.
   * \param updateInternalStructures If false, the caller is responsible for calling
   *        updateInternalShapeLists().
   * \sa updateInternalShapeLists()
   */
  void setIgnoreInContainsTest(
      std::set<point_containment_filter::ShapeHandle> ignoreInContainsTest,
      bool updateInternalStructures = true);

  /**
   * \brief Set the shapes to be ignored when doing test for SHADOW in maskContainmentAndShadows.
   * \param ignoreInShadowTest The shapes to be ignored.
   * \param updateInternalStructures If false, the caller is responsible for calling
   *        updateInternalShapeLists().
   * \note E.g. the sensor collision shape should be listed here.
   * \sa updateInternalShapeLists()
   */
  void setIgnoreInShadowTest(
      std::set<point_containment_filter::ShapeHandle> ignoreInShadowTest,
      bool updateInternalStructures = true);

  /**
   * \brief Provides the map of shape handle to corresponding body (for all added shapes).
   * \return Map shape_handle->body* .
   * \note The returned bodies should not be changed.
   */
  std::map<point_containment_filter::ShapeHandle, bodies::Body*> getBodies() const;

  /**
   * \brief Provides the map of shape handle to corresponding body (for all shapes except those
   * specified in setIgnoreInContainsTest()).
   * \return Map shape_handle->body* .
   * \note The returned bodies should not be changed.
   */
  std::map<point_containment_filter::ShapeHandle, bodies::Body*> getBodiesForContainsTest() const;

  /**
   * \brief Provides the map of shape handle to corresponding body (for all shapes except those
   * specified in setIgnoreInShadowTest()).
   * \return Map shape_handle->body* .
   * \note The returned bodies should not be changed.
   */
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
      const Eigen::Vector3d& data,
      MaskValue &mask,
      const Eigen::Vector3d& sensorPos,
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

  bool doClipping = true; //!< Classify for CLIP during masking.
  bool doContainsTest = true; //!< Classify for INSIDE during masking.
  bool doShadowTest = true; //!< Classify for SHADOW during masking.

  class RayCastingShapeMaskPIMPL;
  std::unique_ptr<RayCastingShapeMaskPIMPL> data; //!< Implementation-private data.

  /**
   * \brief Contains indices of bodies (as listed in this->bodies_) which correspond to bounding
   * spheres in this->bspheres_. Indices in this vector correspond to indices in bspheres_. Values
   * of this vector correspond to indices in bodies_.
   */
  std::vector<size_t> bspheresBodyIndices;
  /** \brief Bounding spheres to be used for classifying INSIDE points. */
  std::vector<bodies::BoundingSphere> bspheresForContainsTest;
};

}

#endif //ROBOT_BODY_FILTER_RAYCASTINGSHAPEMASK_H
