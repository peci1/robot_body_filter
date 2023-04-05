/* HACK HACK HACK */
/* We want to subclass ShapeMask and use its private members. */
#include <sstream>  // has to be there, otherwise we encounter build problems
#define private protected
#include <moveit/point_containment_filter/shape_mask.h>
#undef private
/* HACK END HACK */

#include <list>

#include <robot_body_filter/RayCastingShapeMask.h>

#include <geometric_shapes/body_operations.h>

#include <ros/console.h>

namespace robot_body_filter
{

struct RayCastingShapeMask::RayCastingShapeMaskPIMPL
{
  std::set<SeeShape, SortBodies> bodiesForContainsTest;
  std::set<SeeShape, SortBodies> bodiesForShadowTest;
  std::set<SeeShape, SortBodies> bodiesForBsphere;
  std::set<SeeShape, SortBodies> bodiesForBbox;
  std::map<point_containment_filter::ShapeHandle, std::string> shapeNames;

  typedef std::tuple<MultiShapeHandle, SeeShape, SeeShape, SeeShape, SeeShape> MultiBodyTuple;
  std::list<MultiBodyTuple> multiBodies;

  std::map<point_containment_filter::ShapeHandle, MultiShapeHandle>
  shapesToMultiShapes;

  bodies::BoundingSphere boundingSphere;
  bodies::BoundingSphere boundingSphereForContainsTest;
};

RayCastingShapeMask::RayCastingShapeMask(
    const TransformCallback& transformCallback,
    const double minSensorDist, const double maxSensorDist,
    const bool doClipping, const bool doContainsTest, const bool doShadowTest,
    const double maxShadowDist)
    : ShapeMask(transformCallback),
      minSensorDist(minSensorDist),
      maxSensorDist(maxSensorDist),
      doClipping(doClipping),
      doContainsTest(doContainsTest),
      doShadowTest(doShadowTest),
      maxShadowDist(maxShadowDist)
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
    if (sphereIndex >= this->bspheresBodyIndices.size()) {
      break;
    }
    if (this->bspheresBodyIndices[sphereIndex] == bodyIndex) {
      map[seeShape.handle] = this->bspheres_[sphereIndex];
      sphereIndex++;
    }
    bodyIndex++;
  }

  return map;
}

std::map<point_containment_filter::ShapeHandle, bodies::BoundingSphere>
RayCastingShapeMask::getBoundingSpheresForContainsTest() const
{
  boost::mutex::scoped_lock _(this->shapes_lock_);
  std::map<point_containment_filter::ShapeHandle, bodies::BoundingSphere> map;

  size_t bodyIndex = 0, sphereIndex = 0;
  for (const auto& seeShape : this->bodies_) {
    if (sphereIndex >= this->bspheresForContainsTestBodyIndices.size()) {
      break;
    }
    if (this->bspheresForContainsTestBodyIndices[sphereIndex] == bodyIndex) {
      map[seeShape.handle] = this->bspheresForContainsTest[sphereIndex];
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
  return this->data->boundingSphere;
}

bodies::BoundingSphere RayCastingShapeMask::getBoundingSphereForContainsTestNoLock() const
{
  return this->data->boundingSphereForContainsTest;
}

void RayCastingShapeMask::updateBodyPoses()
{
  boost::mutex::scoped_lock _(this->shapes_lock_);
  this->updateBodyPosesNoLock();
}

void RayCastingShapeMask::updateBodyPosesNoLock()
{
  auto transform = Eigen::Isometry3d::Identity();
  point_containment_filter::ShapeHandle containsHandle;
  bodies::Body* containsBody;
  bodies::Body* shadowBody;
  bodies::Body* bsphereBody;
  bodies::Body* bboxBody;
  std::set<const bodies::Body*> validBodies;

  for (const auto& multiBody : this->data->multiBodies)
  {
    containsHandle = std::get<0>(multiBody).contains;
    containsBody = std::get<1>(multiBody).body;
    shadowBody = std::get<2>(multiBody).body;
    bsphereBody = std::get<3>(multiBody).body;
    bboxBody = std::get<4>(multiBody).body;

    if (this->transform_callback_(containsHandle, transform))
    {
      containsBody->setPose(transform);
      validBodies.insert(containsBody);

      if (containsBody != shadowBody)
      {
        shadowBody->setPose(transform);
        validBodies.insert(shadowBody);
      }

      if (bsphereBody != containsBody && bsphereBody != shadowBody)
      {
        bsphereBody->setPose(transform);
        validBodies.insert(bsphereBody);
      }

      if (bboxBody != containsBody && bboxBody != shadowBody && bboxBody != bsphereBody)
      {
        bboxBody->setPose(transform);
        validBodies.insert(bboxBody);
      }
    }
    else
    {
      if (containsBody == nullptr)
        ROS_ERROR_STREAM_DELAYED_THROTTLE_NAMED(3, "shape_mask",
            "Missing transform for shape with handle " << containsHandle
            << " without a body");
      else {
        std::string name;
        if (this->data->shapeNames.find(containsHandle) != this->data->shapeNames.end())
          name = this->data->shapeNames.at(containsHandle);

        if (name.empty())
          ROS_ERROR_STREAM_DELAYED_THROTTLE_NAMED(3, "shape_mask",
              "Missing transform for shape " << containsBody->getType()
              << " with handle " << containsHandle);
        else
          ROS_ERROR_STREAM_DELAYED_THROTTLE_NAMED(3, "shape_mask",
              "Missing transform for shape " << name << " (" << containsBody->getType() << ")");
      }
    }
  }

  // TODO: prevent frequent reallocations of memory
  this->bspheres_.resize(this->bodies_.size());
  this->bspheresBodyIndices.resize(this->bodies_.size());
  this->bspheresForContainsTest.resize(this->bodies_.size());
  this->bspheresForContainsTestBodyIndices.resize(this->bodies_.size());

  size_t bodyIdx = 0, validBodyIdx = 0, validContainsTestIdx = 0;
  for (const auto& seeShape : this->bodies_)
  {
    const auto& shapeHandle = seeShape.handle;
    const auto body = seeShape.body;
    const auto& multiShape = this->data->shapesToMultiShapes.at(shapeHandle);
    if (validBodies.find(body) != validBodies.end())
    {
      this->bspheresBodyIndices[validBodyIdx] = bodyIdx;
      body->computeBoundingSphere(this->bspheres_[validBodyIdx]);

      if (shapeHandle == multiShape.contains &&
        this->ignoreInContainsTest.find(multiShape) == this->ignoreInContainsTest.end())
      {
        this->bspheresForContainsTestBodyIndices[validContainsTestIdx] = bodyIdx;
        this->bspheresForContainsTest[validContainsTestIdx] = this->bspheres_[validBodyIdx];
        validContainsTestIdx++;
      }

      validBodyIdx++;
    }

    bodyIdx++;
  }
  this->bspheres_.resize(validBodyIdx);
  this->bspheresForContainsTest.resize(validContainsTestIdx);
  this->bspheresBodyIndices.resize(validBodyIdx);
  this->bspheresForContainsTestBodyIndices.resize(validContainsTestIdx);

  bodies::mergeBoundingSpheres(this->bspheres_, this->data->boundingSphere);
  bodies::mergeBoundingSpheres(this->bspheresForContainsTest,
                               this->data->boundingSphereForContainsTest);
}

void RayCastingShapeMask::maskContainmentAndShadows(
    const Cloud& data, std::vector<RayCastingShapeMask::MaskValue>& mask,
    const Eigen::Vector3d& sensorPos)
{
  boost::mutex::scoped_lock _(this->shapes_lock_);

  const auto np = num_points(data);
  mask.resize(np);

  this->updateBodyPosesNoLock();

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
    this->classifyPointNoLock(pt, mask[i], sensorPos);
  }
}

void RayCastingShapeMask::maskContainmentAndShadows(const Eigen::Vector3f& data,
    RayCastingShapeMask::MaskValue& mask, const Eigen::Vector3d& sensorPos,
    const bool updateBodyPoses)
{
  if (data.hasNaN())
  {
    mask = MaskValue::OUTSIDE;
    return;
  }

  boost::mutex::scoped_lock _(this->shapes_lock_);

  if (updateBodyPoses)
    this->updateBodyPosesNoLock();

  this->classifyPointNoLock(data.cast<double>(), mask, sensorPos);
}

void RayCastingShapeMask::classifyPointNoLock(const Eigen::Vector3d& data,
    RayCastingShapeMask::MaskValue &mask, const Eigen::Vector3d& sensorPos)
{
  mask = MaskValue::OUTSIDE;

  if (data.hasNaN()) {
    return;
  }

  // direction from measured point to sensor
  Eigen::Vector3d dir(sensorPos - data);
  const auto distance = dir.norm();

  if (this->doClipping && (distance < this->minSensorDist ||
    (this->maxSensorDist > 0.0 && distance > this->maxSensorDist)))
  {
    // check if the point is inside measurement range
    mask = MaskValue::CLIP;
    return;
  }

  // check if it is inside the scaled body
  const auto radiusSquared = pow(this->data->boundingSphereForContainsTest.radius, 2);
  if (this->doContainsTest &&
    (this->data->boundingSphereForContainsTest.center - data).squaredNorm() < radiusSquared)
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

  if (this->doShadowTest && (this->maxShadowDist <= 0.0 || distance <= this->maxShadowDist)) {
    // point is not inside the robot, check if it is a shadow point
    dir /= distance;
    EigenSTL::vector_Vector3d intersections;
    for (const auto &seeShape : this->data->bodiesForShadowTest) {
      // get the 1st intersection of ray pt->sensor
      intersections.clear(); // intersectsRay doesn't clear the vector...
      if (seeShape.body->intersectsRay(data, dir, &intersections, 1)) {
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
    std::unordered_set<MultiShapeHandle> ignoreInContainsTest,
    const bool updateInternalStructures)
{
  this->ignoreInContainsTest = std::move(ignoreInContainsTest);
  if (updateInternalStructures)
    this->updateInternalShapeLists();
}

void RayCastingShapeMask::setIgnoreInShadowTest(
    std::unordered_set<MultiShapeHandle> ignoreInShadowTest,
    const bool updateInternalStructures)
{
  this->ignoreInShadowTest = std::move(ignoreInShadowTest);
  if (updateInternalStructures)
    this->updateInternalShapeLists();
}

MultiShapeHandle RayCastingShapeMask::addShape(
    const shapes::ShapeConstPtr &shape, const double scale, const double padding,
    const bool updateInternalStructures, const std::string& name)
{
  return this->addShape(shape, scale, padding, scale, padding,
                        scale, padding, scale, padding, updateInternalStructures, name);
}

MultiShapeHandle RayCastingShapeMask::addShape(
    const shapes::ShapeConstPtr &shape, const double containsScale, const double containsPadding,
    const double shadowScale, const double shadowPadding, const double bsphereScale, const double bspherePadding,
    const double bboxScale, const double bboxPadding, const bool updateInternalStructures, const std::string& name)
{
  MultiShapeHandle result;

  result.contains = ShapeMask::addShape(shape, containsScale, containsPadding);
  this->data->shapeNames[result.contains] = name;

  result.shadow = result.contains;
  if ((std::abs(containsScale - shadowScale) > 1e-6 ||
      std::abs(containsPadding - shadowPadding) > 1e-6))
  {
    result.shadow = ShapeMask::addShape(shape, shadowScale, shadowPadding);
    this->data->shapeNames[result.shadow] = name;
  }

  result.bsphere = result.contains;
  if ((std::abs(containsScale - bsphereScale) > 1e-6 ||
      std::abs(containsPadding - bspherePadding) > 1e-6))
  {
    result.bsphere = ShapeMask::addShape(shape, bsphereScale, bspherePadding);
    this->data->shapeNames[result.bsphere] = name;
  }

  result.bbox = result.contains;
  if ((std::abs(containsScale - bboxScale) > 1e-6 ||
      std::abs(containsPadding - bboxPadding) > 1e-6))
  {
    result.bbox = ShapeMask::addShape(shape, bboxScale, bboxPadding);
    this->data->shapeNames[result.bbox] = name;
  }

  auto containsSeeShape = *this->used_handles_.at(result.contains);
  auto shadowSeeShape = *this->used_handles_.at(result.shadow);
  auto bsphereSeeShape = *this->used_handles_.at(result.bsphere);
  auto bboxSeeShape = *this->used_handles_.at(result.bbox);
  this->data->multiBodies.emplace_back(result, containsSeeShape, shadowSeeShape, bsphereSeeShape, bboxSeeShape);

  this->data->shapesToMultiShapes[result.contains] = result;
  this->data->shapesToMultiShapes[result.shadow] = result;
  this->data->shapesToMultiShapes[result.bsphere] = result;
  this->data->shapesToMultiShapes[result.bbox] = result;

  if (updateInternalStructures)
    this->updateInternalShapeLists();
  return result;
}

void RayCastingShapeMask::removeShape(const MultiShapeHandle& handle,
    const bool updateInternalStructures)
{
  this->data->multiBodies.remove_if([handle](RayCastingShapeMaskPIMPL::MultiBodyTuple t){
    return std::get<0>(t) == handle;
  });

  ShapeMask::removeShape(handle.contains);
  this->data->shapeNames.erase(handle.contains);
  this->data->shapesToMultiShapes.erase(handle.contains);

  if (handle.contains != handle.shadow)
  {
    ShapeMask::removeShape(handle.shadow);
    this->data->shapeNames.erase(handle.shadow);
    this->data->shapesToMultiShapes.erase(handle.shadow);
  }

  if (handle.contains != handle.bsphere)
  {
    ShapeMask::removeShape(handle.bsphere);
    this->data->shapeNames.erase(handle.bsphere);
    this->data->shapesToMultiShapes.erase(handle.bsphere);
  }

  if (handle.contains != handle.bbox)
  {
    ShapeMask::removeShape(handle.bbox);
    this->data->shapeNames.erase(handle.bbox);
    this->data->shapesToMultiShapes.erase(handle.bbox);
  }

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
  this->data->bodiesForBsphere.clear();
  this->data->bodiesForBbox.clear();

  for (const auto& multiBody : this->data->multiBodies) {
    const auto handle = std::get<0>(multiBody);
    const auto containsSeeShape = std::get<1>(multiBody);
    const auto shadowSeeShape = std::get<2>(multiBody);
    const auto bsphereSeeShape = std::get<3>(multiBody);
    const auto bboxSeeShape = std::get<4>(multiBody);

    if (this->ignoreInContainsTest.find(handle) == this->ignoreInContainsTest.end())
      this->data->bodiesForContainsTest.insert(containsSeeShape);
    if (this->ignoreInShadowTest.find(handle) == this->ignoreInShadowTest.end())
      this->data->bodiesForShadowTest.insert(shadowSeeShape);
    if (this->ignoreInBsphere.find(handle) == this->ignoreInBsphere.end())
      this->data->bodiesForBsphere.insert(bsphereSeeShape);
    if (this->ignoreInBbox.find(handle) == this->ignoreInBbox.end())
      this->data->bodiesForBbox.insert(bboxSeeShape);
  }
}

std::map<point_containment_filter::ShapeHandle, const bodies::Body*>
RayCastingShapeMask::getBodies() const
{
  boost::mutex::scoped_lock _(this->shapes_lock_);
  std::map<point_containment_filter::ShapeHandle, const bodies::Body*> result;

  for (const auto& seeShape: this->bodies_)
    result[seeShape.handle] = seeShape.body;

  return result;
}

std::map<point_containment_filter::ShapeHandle, const bodies::Body*>
RayCastingShapeMask::getBodiesForContainsTest() const
{
  boost::mutex::scoped_lock _(this->shapes_lock_);
  std::map<point_containment_filter::ShapeHandle, const bodies::Body*> result;

  for (const auto& seeShape: this->data->bodiesForContainsTest)
    result[seeShape.handle] = seeShape.body;

  return result;
}

std::map<point_containment_filter::ShapeHandle, const bodies::Body*>
RayCastingShapeMask::getBodiesForShadowTest() const
{
  boost::mutex::scoped_lock _(this->shapes_lock_);
  std::map<point_containment_filter::ShapeHandle, const bodies::Body*> result;

  for (const auto& seeShape: this->data->bodiesForShadowTest)
    result[seeShape.handle] = seeShape.body;

  return result;
}

std::map<point_containment_filter::ShapeHandle, const bodies::Body*>
RayCastingShapeMask::getBodiesForBoundingSphere() const
{
  boost::mutex::scoped_lock _(this->shapes_lock_);
  std::map<point_containment_filter::ShapeHandle, const bodies::Body*> result;

  for (const auto& seeShape: this->data->bodiesForBsphere)
    result[seeShape.handle] = seeShape.body;

  return result;
}

std::map<point_containment_filter::ShapeHandle, const bodies::Body*>
RayCastingShapeMask::getBodiesForBoundingBox() const
{
  boost::mutex::scoped_lock _(this->shapes_lock_);
  std::map<point_containment_filter::ShapeHandle, const bodies::Body*> result;

  for (const auto& seeShape: this->data->bodiesForBbox)
    result[seeShape.handle] = seeShape.body;

  return result;
}

bool MultiShapeHandle::operator==(const MultiShapeHandle& other) const
{
  return this->contains == other.contains && this->shadow == other.shadow &&
         this->bsphere == other.bsphere && this->bbox == other.bbox;
}

bool MultiShapeHandle::operator!=(const MultiShapeHandle& other) const
{
  return !(*this == other);
}

}