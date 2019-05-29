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
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

namespace robot_body_filter
{

ros::NodeHandle nh;
ros::Publisher pub;
visualization_msgs::Marker msg;

struct RayCastingShapeMask::RayCastingShapeMaskPIMPL
{
  std::set<SeeShape, SortBodies> bodiesForContainsTest;
  std::set<SeeShape, SortBodies> bodiesForShadowTest;
};

RayCastingShapeMask::RayCastingShapeMask(
    const TransformCallback& transformCallback,
    const double minSensorDist, const double maxSensorDist)
    : ShapeMask(transformCallback),
      minSensorDist(minSensorDist),
      maxSensorDist(maxSensorDist)
{
  this->data = std::make_unique<RayCastingShapeMaskPIMPL>();
  pub = nh.advertise<visualization_msgs::Marker>("test", 10);
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
  msg.header.frame_id = "odom";
  msg.header.stamp = ros::Time::now();
  msg.action = visualization_msgs::Marker::ADD;
  msg.pose.orientation.w = 1;
  msg.frame_locked = true;
  msg.lifetime = ros::Duration(1.0);
  msg.ns = "test";
  msg.type = visualization_msgs::Marker::LINE_LIST;
  msg.scale.x = 0.001;
  msg.color.a = 1.0;

  pub.publish(msg);
  msg.points.clear();
  msg.colors.clear();
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
        ROS_ERROR_STREAM_NAMED("shape_mask",
            "Missing transform for shape with handle " << shapeHandle
            << " without a body");
      else
        ROS_ERROR_STREAM_NAMED("shape_mask", "Missing transform for shape "
            << body->getType() << " with handle " << shapeHandle);
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
    const Eigen::Vector3f& sensorPos)
{
  boost::mutex::scoped_lock _(this->shapes_lock_);

  const auto np = num_points(data);
  mask.resize(np);

  this->updateBodyPosesNoLock();

  // compute a sphere that bounds the entire robot
  const bodies::BoundingSphere bound = this->getBoundingSphereForContainsTestNoLock();

  // we now decide which points we keep
  CloudConstIter iter_x(data, "x");
  CloudConstIter iter_y(data, "y");
  CloudConstIter iter_z(data, "z");

  // Cloud iterators are not incremented in the for loop, because of the pragma
  // Comment out below parallelization as it can result in very high CPU consumption
  //#pragma omp parallel for schedule(dynamic)
  for (size_t i = 0; i < np; ++i)
  {
    const Eigen::Vector3f pt(*(iter_x + i), *(iter_y + i), *(iter_z + i));
    this->classifyPointNoLock(pt, mask[i], sensorPos, bound);
  }
}

void RayCastingShapeMask::maskContainmentAndShadows(const Eigen::Vector3f& data,
    RayCastingShapeMask::MaskValue& mask, const Eigen::Vector3f& sensorPos,
    const bool updateBodyPoses)
{
  boost::mutex::scoped_lock _(this->shapes_lock_);

  if (updateBodyPoses)
    this->updateBodyPosesNoLock();

  // compute a sphere that bounds the entire robot
  const bodies::BoundingSphere bound = this->getBoundingSphereForContainsTestNoLock();

  this->classifyPointNoLock(data, mask, sensorPos, bound);
}
geometry_msgs::Point fn(const Eigen::Vector3f& data) {
  geometry_msgs::Point result;
  result.x = data.x();
  result.y = data.y();
  result.z = data.z();
  return result;
}
void RayCastingShapeMask::classifyPointNoLock(const Eigen::Vector3f& data,
    RayCastingShapeMask::MaskValue &mask, const Eigen::Vector3f& sensorPos,
    const bodies::BoundingSphere &boundingSphereForContainsTest)
{
  mask = MaskValue::OUTSIDE;

  // direction from measured point to sensor
  Eigen::Vector3d dir(sensorPos.cast<double>() - data.cast<double>());
  const auto distance = dir.norm();

  if (distance < this->minSensorDist || (this->maxSensorDist > 0.0 && distance > this->maxSensorDist)) {
    // check if the point is inside measurement range
    mask = MaskValue::CLIP;
    return;
  }

  // check if it is inside the scaled body
  const auto radiusSquared = pow(boundingSphereForContainsTest.radius, 2);
  if ((boundingSphereForContainsTest.center - data.cast<double>()).squaredNorm() < radiusSquared)
  {
    for (const auto &seeShape : this->data->bodiesForContainsTest)
    {
      if (seeShape.body->containsPoint(data.cast<double>()))
      {
        mask = MaskValue::INSIDE;
        return;
      }
    }
  }

  // point is not inside the robot, check if it is a shadow point
  dir /= distance;
  EigenSTL::vector_Vector3d intersections;

  std_msgs::ColorRGBA red;
  std_msgs::ColorRGBA green;
  std_msgs::ColorRGBA blue;
  std_msgs::ColorRGBA white;
  std_msgs::ColorRGBA yellow;

  red.r = red.a = 1.0;
  green.g = green.a = 1.0;
  blue.b = blue.a = 1.0;
  white.b = white.a = white.r = white.g = 1.0;
  yellow.a = yellow.r = yellow.g = 1.0;

  for (const auto &seeShape : this->data->bodiesForShadowTest)
  {
    auto print = seeShape.handle == 16 && fabs(data.x() - 0.86) < 0.1 &&
                 fabs(data.y() - -0.09) < 0.1 && fabs(data.z() - 0.39) < 0.1;
    // get the 1st intersection of ray pt->sensor
    intersections.clear();  // intersectsRay doesn't clear the vector...
//    if (print) {
//      ROS_WARN_STREAM(seeShape.body->getDimensions()[0] << " " << seeShape.body->getDimensions()[1] << " " << seeShape.body->getDimensions()[2] << "---\n" << seeShape.body->getPose().linear() << "$$$\n" << seeShape.body->getPose().translation() << "***\n");
//      ROS_WARN_STREAM(data << "&&&\n" << dir);
//    }
    if (bodies::intersectsRay(seeShape.body, data.cast<double>(), dir, &intersections, 1))
    {
//      if (print) ROS_WARN_STREAM(intersections[0] << "^^^\n");
      // is the intersection between point and sensor?
      if (dir.dot(sensorPos.cast<double>() - intersections[0]) >= 0.0)
      {
        mask = MaskValue::SHADOW;
        msg.points.push_back(fn(data));
        msg.points.push_back(fn(intersections[0].cast<float>()));
        msg.points.push_back(fn(intersections[0].cast<float>()));
        msg.points.push_back(fn(sensorPos));
        msg.colors.push_back(green);
        msg.colors.push_back(green);
        msg.colors.push_back(white);
        msg.colors.push_back(white);
        return;
      } else {
//        ROS_WARN("no collision");
      }
    } else {

    }
  }
  msg.points.push_back(fn(data));
  msg.points.push_back(fn(sensorPos));
  msg.colors.push_back(red);
  msg.colors.push_back(red);
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
    const bool updateInternalStructures)
{
  const auto result = ShapeMask::addShape(shape, scale, padding);
  if (updateInternalStructures)
    this->updateInternalShapeLists();
  return result;
}

void RayCastingShapeMask::removeShape(
    point_containment_filter::ShapeHandle handle,
    const bool updateInternalStructures)
{
  ShapeMask::removeShape(handle);
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