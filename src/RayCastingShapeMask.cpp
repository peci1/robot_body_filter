/* HACK HACK HACK */
/* We want to subclass ShapeMask and use its members. */
// Upstream solution proposed in https://github.com/ros-planning/moveit/pull/1457
#include <sstream>  // has to be there, otherwise we encounter build problems
#define private protected
#include <moveit/point_containment_filter/shape_mask.h>
#undef private
/* HACK END HACK */

#include <robot_body_filter/RayCastingShapeMask.h>

#include <geometric_shapes/body_operations.h>

#include <ros/console.h>

namespace robot_body_filter
{

struct RayCastingShapeMask::RayCastingShapeMaskPIMPL
{
  std::set<SeeShape, SortBodies> bodiesForContainsTest;
  std::set<SeeShape, SortBodies> bodiesForShadowTest;
  std::map<point_containment_filter::ShapeHandle, std::string> shapeNames;
};

RayCastingShapeMask::RayCastingShapeMask(
    const TransformCallback& transformCallback,
    const double minSensorDist, const double maxSensorDist,
    const bool doClipping, const bool doContainsTest, const bool doShadowTest)
    : ShapeMask(transformCallback),
      minSensorDist(minSensorDist),
      maxSensorDist(maxSensorDist),
      doClipping(doClipping),
      doContainsTest(doContainsTest),
      doShadowTest(doShadowTest)
{
  this->data = std::make_unique<RayCastingShapeMaskPIMPL>();
}

RayCastingShapeMask::~RayCastingShapeMask() = default;

std::map<point_containment_filter::ShapeHandle, bodies::BoundingSphere>
RayCastingShapeMask::getBoundingSpheres() const
{
  boost::mutex::scoped_lock _(this->shapes_lock_);
  std::map<point_containment_filter::ShapeHandle, bodies::BoundingSphere> map;

  size_t bodyIndex = 0, sphereIndex = 0;
  for (const auto& seeShape : this->bodies_) {
    if (this->bspheresBodyIndices[sphereIndex] == bodyIndex) {
      map[seeShape.handle] = this->bspheres_[sphereIndex];
      sphereIndex++;
    }
    bodyIndex++;
  }

  return map;
}

bodies::BoundingSphere RayCastingShapeMask::getBoundingSphere() const
{
  boost::mutex::scoped_lock _(this->shapes_lock_);
  return this->getBoundingSphereNoLock();
}

bodies::BoundingSphere RayCastingShapeMask::getBoundingSphereNoLock() const
{
  bodies::BoundingSphere result;
  bodies::mergeBoundingSpheres(this->bspheres_, result);
  return result;
}

bodies::BoundingSphere RayCastingShapeMask::getBoundingSphereForContainsTestNoLock() const
{
  bodies::BoundingSphere result;
  bodies::mergeBoundingSpheres(this->bspheresForContainsTest, result);
  return result;
}

void RayCastingShapeMask::updateBodyPoses()
{
  boost::mutex::scoped_lock _(this->shapes_lock_);
  this->updateBodyPosesNoLock();
}

void RayCastingShapeMask::updateBodyPosesNoLock()
{
  this->bspheres_.resize(this->bodies_.size());
  this->bspheresBodyIndices.resize(this->bodies_.size());
  this->bspheresForContainsTest.resize(this->bodies_.size());

  Eigen::Isometry3d transform;
  point_containment_filter::ShapeHandle shapeHandle;
  bodies::Body* body;
  size_t i = 0, j = 0, k = 0;

  for (const auto& seeShape : this->bodies_)
  {
    shapeHandle = seeShape.handle;
    body = seeShape.body;

    if (!this->transform_callback_(shapeHandle, transform))
    {
      if (body == nullptr)
        ROS_ERROR_STREAM_DELAYED_THROTTLE_NAMED(3, "shape_mask",
            "Missing transform for shape with handle " << shapeHandle
            << " without a body");
      else {
        std::string name;
        if (this->data->shapeNames.find(shapeHandle) != this->data->shapeNames.end())
          name = this->data->shapeNames.at(shapeHandle);

        if (name.empty())
          ROS_ERROR_STREAM_DELAYED_THROTTLE_NAMED(3, "shape_mask",
              "Missing transform for shape " << body->getType()
              << " with handle " << shapeHandle);
        else
          ROS_ERROR_STREAM_DELAYED_THROTTLE_NAMED(3, "shape_mask",
              "Missing transform for shape " << name << " (" << body->getType() << ")");
      }
    }
    else
    {
      body->setPose(transform);

      this->bspheresBodyIndices[j] = i;
      body->computeBoundingSphere(this->bspheres_[j]);

      if (this->ignoreInContainsTest.find(shapeHandle) == this->ignoreInContainsTest.end())
      {
        this->bspheresForContainsTest[k] = this->bspheres_[j];
        k++;
      }

      j++;
    }

    i++;
  }
}

void RayCastingShapeMask::maskContainmentAndShadows(
    const Cloud& data, std::vector<RayCastingShapeMask::MaskValue>& mask,
    const Eigen::Vector3d& sensorPos)
{
  boost::mutex::scoped_lock _(this->shapes_lock_);

  const auto np = num_points(data);
  mask.resize(np);

  this->updateBodyPosesNoLock();

  // compute a sphere that bounds the entire robot
  const auto bound = this->getBoundingSphereForContainsTestNoLock();

  // we now decide which points we keep
  CloudConstIter iter_x(data, "x");
  CloudConstIter iter_y(data, "y");
  CloudConstIter iter_z(data, "z");

  // Cloud iterators are not incremented in the for loop, because of the pragma
  // Comment out below parallelization as it can result in very high CPU consumption
  //#pragma omp parallel for schedule(dynamic)
  for (size_t i = 0; i < np; ++i)
  {
    const Eigen::Vector3d pt(static_cast<double>(*(iter_x + i)),
                             static_cast<double>(*(iter_y + i)),
                             static_cast<double>(*(iter_z + i)));
    this->classifyPointNoLock(pt, mask[i], sensorPos, bound);
  }
}

void RayCastingShapeMask::maskContainmentAndShadows(const Eigen::Vector3f& data,
    RayCastingShapeMask::MaskValue& mask, const Eigen::Vector3d& sensorPos,
    const bool updateBodyPoses)
{
  boost::mutex::scoped_lock _(this->shapes_lock_);

  if (updateBodyPoses)
    this->updateBodyPosesNoLock();

  // compute a sphere that bounds the entire robot
  const bodies::BoundingSphere bound = this->getBoundingSphereForContainsTestNoLock();

  this->classifyPointNoLock(data.cast<double>(), mask, sensorPos, bound);
}

void RayCastingShapeMask::classifyPointNoLock(const Eigen::Vector3d& data,
    RayCastingShapeMask::MaskValue &mask, const Eigen::Vector3d& sensorPos,
    const bodies::BoundingSphere &boundingSphereForContainsTest)
{
  mask = MaskValue::OUTSIDE;

  // direction from measured point to sensor
  Eigen::Vector3d dir(sensorPos - data);
  const auto distance = dir.norm();

  if (this->doClipping && (distance < this->minSensorDist || (this->maxSensorDist > 0.0 && distance > this->maxSensorDist))) {
    // check if the point is inside measurement range
    mask = MaskValue::CLIP;
    return;
  }

  // check if it is inside the scaled body
  const auto radiusSquared = pow(boundingSphereForContainsTest.radius, 2);
  if (this->doContainsTest && (boundingSphereForContainsTest.center - data).squaredNorm() < radiusSquared)
  {
    for (const auto &seeShape : this->data->bodiesForContainsTest)
    {
      if (seeShape.body->containsPoint(data))
      {
        mask = MaskValue::INSIDE;
        return;
      }
    }
  }

  if (this->doShadowTest) {
    // point is not inside the robot, check if it is a shadow point
    dir /= distance;
    EigenSTL::vector_Vector3d intersections;
    for (const auto &seeShape : this->data->bodiesForShadowTest) {
      // get the 1st intersection of ray pt->sensor
      intersections.clear(); // intersectsRay doesn't clear the vector...
      if (bodies::intersectsRay(seeShape.body, data, dir, &intersections, 1)) {
        // is the intersection between point and sensor?
        if (dir.dot(sensorPos - intersections[0]) >= 0.0) {
          mask = MaskValue::SHADOW;
          return;
        }
      }
    }
  }
}

void RayCastingShapeMask::setIgnoreInContainsTest(
    std::set<point_containment_filter::ShapeHandle> ignoreInContainsTest,
    const bool updateInternalStructures)
{
  this->ignoreInContainsTest = std::move(ignoreInContainsTest);
  if (updateInternalStructures)
    this->updateInternalShapeLists();
}

void RayCastingShapeMask::setIgnoreInShadowTest(
    std::set<point_containment_filter::ShapeHandle> ignoreInShadowTest,
    const bool updateInternalStructures)
{
  this->ignoreInShadowTest = std::move(ignoreInShadowTest);
  if (updateInternalStructures)
    this->updateInternalShapeLists();
}

point_containment_filter::ShapeHandle RayCastingShapeMask::addShape(
    const shapes::ShapeConstPtr &shape, const double scale, const double padding,
    const bool updateInternalStructures, const std::string& name)
{
  const auto result = ShapeMask::addShape(shape, scale, padding);
  this->data->shapeNames[result] = name;
  if (updateInternalStructures)
    this->updateInternalShapeLists();
  return result;
}

void RayCastingShapeMask::removeShape(
    point_containment_filter::ShapeHandle handle,
    const bool updateInternalStructures)
{
  ShapeMask::removeShape(handle);
  this->data->shapeNames.erase(handle);
  if (updateInternalStructures)
    this->updateInternalShapeLists();
}

void RayCastingShapeMask::setTransformCallback(
    const point_containment_filter::ShapeMask::TransformCallback &transform_callback)
{
  ShapeMask::setTransformCallback(transform_callback);
}

void RayCastingShapeMask::updateInternalShapeLists()
{
  boost::mutex::scoped_lock _(this->shapes_lock_);

  this->data->bodiesForContainsTest.clear();
  this->data->bodiesForShadowTest.clear();

  for (const auto& seeShape : this->bodies_) {
    if (this->ignoreInContainsTest.find(seeShape.handle) == this->ignoreInContainsTest.end())
      this->data->bodiesForContainsTest.insert(seeShape);
    if (this->ignoreInShadowTest.find(seeShape.handle) == this->ignoreInShadowTest.end())
      this->data->bodiesForShadowTest.insert(seeShape);
  }
}

std::map<point_containment_filter::ShapeHandle,
         bodies::Body *> RayCastingShapeMask::getBodies() const
{
  boost::mutex::scoped_lock _(this->shapes_lock_);
  std::map<point_containment_filter::ShapeHandle, bodies::Body *> result;

  for (const auto& seeShape: this->bodies_)
    result[seeShape.handle] = seeShape.body;

  return result;
}

std::map<point_containment_filter::ShapeHandle,
         bodies::Body *> RayCastingShapeMask::getBodiesForContainsTest() const
{
  boost::mutex::scoped_lock _(this->shapes_lock_);
  std::map<point_containment_filter::ShapeHandle, bodies::Body *> result;

  for (const auto& seeShape: this->data->bodiesForContainsTest)
    result[seeShape.handle] = seeShape.body;

  return result;
}

std::map<point_containment_filter::ShapeHandle,
         bodies::Body *> RayCastingShapeMask::getBodiesForShadowTest() const
{
  boost::mutex::scoped_lock _(this->shapes_lock_);
  std::map<point_containment_filter::ShapeHandle, bodies::Body *> result;

  for (const auto& seeShape: this->data->bodiesForShadowTest)
    result[seeShape.handle] = seeShape.body;

  return result;
}

}