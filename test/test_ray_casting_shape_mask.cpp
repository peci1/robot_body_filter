#include "gtest/gtest.h"

/* We want to subclass ShapeMask and use its members. */
// Upstream solution proposed in https://github.com/ros-planning/moveit/pull/1457
#include <sstream>  // has to be there, otherwise we encounter build problems
#define private protected
#include <moveit/point_containment_filter/shape_mask.h>
#undef private

#include <robot_body_filter/RayCastingShapeMask.h>
#include <robot_body_filter/utils/shapes.h>
#include <urdf_model/model.h>
#include "utils.cpp"

using namespace robot_body_filter;
using namespace point_containment_filter;

class TestMask : public RayCastingShapeMask
{
  public:
  TestMask(const point_containment_filter::ShapeMask::TransformCallback &transformCallback,
                   double minSensorDist,
                   double maxSensorDist,
                   bool doClipping,
                   bool doContainsTest,
                   bool doShadowTest) :
    RayCastingShapeMask(transformCallback, minSensorDist, maxSensorDist, doClipping, doContainsTest,
        doShadowTest)
  {

  }

  friend class RayCastingShapeMask_Basic_Test;
  friend class RayCastingShapeMask_Bspheres_Test;
  friend class RayCastingShapeMask_ClassifyPoint_Test;
};

TEST(RayCastingShapeMask, Basic)
{
  ros::Time::init();
  ros::Time::setNow(ros::Time(1));

  auto cb = [] (ShapeHandle h, Eigen::Isometry3d& t) -> bool
  {
    t = Eigen::Isometry3d::Identity();
    return true;
  };
  TestMask mask(cb, 1.0, 10.0, true, false, false);

  EXPECT_EQ(cb, mask.transform_callback_);
  EXPECT_TRUE(mask.doClipping);
  EXPECT_FALSE(mask.doContainsTest);
  EXPECT_FALSE(mask.doShadowTest);
  EXPECT_DOUBLE_EQ(1.0, mask.minSensorDist);
  EXPECT_DOUBLE_EQ(10.0, mask.maxSensorDist);
  EXPECT_TRUE(mask.bodies_.empty());
  EXPECT_TRUE(mask.getBodiesForContainsTest().empty());
  EXPECT_TRUE(mask.getBodiesForShadowTest().empty());
  EXPECT_TRUE(mask.used_handles_.empty());
  EXPECT_TRUE(mask.bspheres_.empty());
  EXPECT_TRUE(mask.ignoreInContainsTest.empty());
  EXPECT_TRUE(mask.ignoreInShadowTest.empty());

  shapes::ShapeConstPtr shape(new shapes::Box(1.0, 2.0, 3.0));
  auto handle = mask.addShape(shape, 1.0, 0.0, true, "box");
  EXPECT_EQ(1, mask.bodies_.size());
  EXPECT_EQ(1, mask.getBodiesForContainsTest().size());
  EXPECT_EQ(1, mask.getBodiesForShadowTest().size());
  EXPECT_EQ(1, mask.used_handles_.size());
  // bspheres are only updated by calling updateBodyPoses()
  EXPECT_EQ(0, mask.bspheres_.size());
  EXPECT_EQ(0, mask.bspheresBodyIndices.size());
  EXPECT_EQ(0, mask.bspheresForContainsTest.size());

  mask.setIgnoreInContainsTest({handle});
  EXPECT_EQ(1, mask.bodies_.size());
  EXPECT_EQ(0, mask.getBodiesForContainsTest().size());
  EXPECT_EQ(1, mask.getBodiesForShadowTest().size());

  mask.setIgnoreInShadowTest({handle});
  EXPECT_EQ(1, mask.bodies_.size());
  EXPECT_EQ(0, mask.getBodiesForContainsTest().size());
  EXPECT_EQ(0, mask.getBodiesForShadowTest().size());

  mask.setIgnoreInContainsTest({});
  mask.setIgnoreInShadowTest({});
  EXPECT_EQ(1, mask.bodies_.size());
  EXPECT_EQ(1, mask.getBodiesForContainsTest().size());
  EXPECT_EQ(1, mask.getBodiesForShadowTest().size());

  mask.removeShape(handle);
  EXPECT_TRUE(mask.bodies_.empty());
  EXPECT_TRUE(mask.getBodiesForContainsTest().empty());
  EXPECT_TRUE(mask.getBodiesForShadowTest().empty());
  EXPECT_TRUE(mask.used_handles_.empty());
  EXPECT_TRUE(mask.ignoreInContainsTest.empty());
  EXPECT_TRUE(mask.ignoreInShadowTest.empty());

  // test adding without updating internal structures
  handle = mask.addShape(shape, 1.0, 0.0, false, "box");
  EXPECT_EQ(1, mask.bodies_.size());
  EXPECT_EQ(0, mask.getBodiesForContainsTest().size());
  EXPECT_EQ(0, mask.getBodiesForShadowTest().size());

  mask.updateInternalShapeLists();
  EXPECT_EQ(1, mask.bodies_.size());
  EXPECT_EQ(1, mask.getBodiesForContainsTest().size());
  EXPECT_EQ(1, mask.getBodiesForShadowTest().size());


  auto cb2 = [] (ShapeHandle h, Eigen::Isometry3d& t) -> bool
  {
    t = Eigen::Isometry3d::Identity();
    return false;
  };
  mask.setTransformCallback(cb2);
  EXPECT_NE(cb, mask.transform_callback_);
  EXPECT_EQ(cb2, mask.transform_callback_);
}

TEST(RayCastingShapeMask, Bspheres)
{
  ros::Time::init();
  ros::Time::setNow(ros::Time(1));

  auto cb = [](ShapeHandle h, Eigen::Isometry3d &t) -> bool
  {
    t = Eigen::Isometry3d::Identity();
    return true;
  };
  TestMask mask(cb, 1.0, 10.0, true, true, true);

  shapes::ShapeConstPtr shape1(new shapes::Box(1.0, 2.0, 3.0));
  const auto handle1 = mask.addShape(shape1, 1.0, 0.0, false, "box");
  shapes::ShapeConstPtr shape2(new shapes::Sphere(2.0));
  const auto handle2 = mask.addShape(shape2, 2.0, 0.5, true, "sphere");

  mask.updateBodyPoses();

  auto bspheres = mask.getBoundingSpheres();
  ASSERT_EQ(2, bspheres.size());
  ASSERT_NE(bspheres.end(), bspheres.find(handle1));
  ASSERT_NE(bspheres.end(), bspheres.find(handle2));
  EXPECT_NEAR(sqrt(0.5 * 0.5 + 1.0 * 1.0 + 1.5 * 1.5), bspheres[handle1].radius, 1e-9);
  EXPECT_DOUBLE_EQ(0.0, bspheres[handle1].center.x());
  EXPECT_DOUBLE_EQ(0.0, bspheres[handle1].center.y());
  EXPECT_DOUBLE_EQ(0.0, bspheres[handle1].center.z());
  EXPECT_NEAR(4.5, bspheres[handle2].radius, 1e-9);
  EXPECT_DOUBLE_EQ(0.0, bspheres[handle2].center.x());
  EXPECT_DOUBLE_EQ(0.0, bspheres[handle2].center.y());
  EXPECT_DOUBLE_EQ(0.0, bspheres[handle2].center.z());

  auto bsphere = mask.getBoundingSphere();
  EXPECT_NEAR(4.5, bsphere.radius, 1e-9);
  EXPECT_DOUBLE_EQ(0.0, bsphere.center.x());
  EXPECT_DOUBLE_EQ(0.0, bsphere.center.y());
  EXPECT_DOUBLE_EQ(0.0, bsphere.center.z());

  auto bsphereForContainsTest = mask.getBoundingSphereForContainsTestNoLock();
  EXPECT_NEAR(bsphere.radius, bsphereForContainsTest.radius, 1e-9);
  EXPECT_DOUBLE_EQ(bsphere.center.x(), bsphereForContainsTest.center.x());
  EXPECT_DOUBLE_EQ(bsphere.center.y(), bsphereForContainsTest.center.y());
  EXPECT_DOUBLE_EQ(bsphere.center.z(), bsphereForContainsTest.center.z());

  auto cb2 = [](ShapeHandle h, Eigen::Isometry3d &t) -> bool
  {
    t = Eigen::Isometry3d::Identity();
    t.translate(Eigen::Vector3d(1.0, 2.0, 3.0));
    return true;
  };
  mask.setTransformCallback(cb2);
  mask.updateBodyPoses();

  bspheres = mask.getBoundingSpheres();
  ASSERT_EQ(2, bspheres.size());
  ASSERT_NE(bspheres.end(), bspheres.find(handle1));
  ASSERT_NE(bspheres.end(), bspheres.find(handle2));
  EXPECT_NEAR(sqrt(0.5 * 0.5 + 1.0 * 1.0 + 1.5 * 1.5), bspheres[handle1].radius, 1e-9);
  EXPECT_DOUBLE_EQ(1.0, bspheres[handle1].center.x());
  EXPECT_DOUBLE_EQ(2.0, bspheres[handle1].center.y());
  EXPECT_DOUBLE_EQ(3.0, bspheres[handle1].center.z());
  EXPECT_NEAR(4.5, bspheres[handle2].radius, 1e-9);
  EXPECT_DOUBLE_EQ(1.0, bspheres[handle2].center.x());
  EXPECT_DOUBLE_EQ(2.0, bspheres[handle2].center.y());
  EXPECT_DOUBLE_EQ(3.0, bspheres[handle2].center.z());

  bsphere = mask.getBoundingSphere();
  EXPECT_NEAR(4.5, bsphere.radius, 1e-9);
  EXPECT_DOUBLE_EQ(1.0, bsphere.center.x());
  EXPECT_DOUBLE_EQ(2.0, bsphere.center.y());
  EXPECT_DOUBLE_EQ(3.0, bsphere.center.z());

  bsphereForContainsTest = mask.getBoundingSphereForContainsTestNoLock();
  EXPECT_NEAR(bsphere.radius, bsphereForContainsTest.radius, 1e-9);
  EXPECT_DOUBLE_EQ(bsphere.center.x(), bsphereForContainsTest.center.x());
  EXPECT_DOUBLE_EQ(bsphere.center.y(), bsphereForContainsTest.center.y());
  EXPECT_DOUBLE_EQ(bsphere.center.z(), bsphereForContainsTest.center.z());

  // make the sphere's position unresolvable
  auto cb3 = [handle2](ShapeHandle h, Eigen::Isometry3d &t) -> bool
  {
    t = Eigen::Isometry3d::Identity();
    return h != handle2;
  };
  mask.setTransformCallback(cb3);
  mask.updateBodyPoses();

  bspheres = mask.getBoundingSpheres();
  ASSERT_EQ(1, bspheres.size());
  ASSERT_NE(bspheres.end(), bspheres.find(handle1));
  ASSERT_EQ(bspheres.end(), bspheres.find(handle2));
  EXPECT_NEAR(sqrt(0.5 * 0.5 + 1.0 * 1.0 + 1.5 * 1.5), bspheres[handle1].radius, 1e-9);
  EXPECT_DOUBLE_EQ(0.0, bspheres[handle1].center.x());
  EXPECT_DOUBLE_EQ(0.0, bspheres[handle1].center.y());
  EXPECT_DOUBLE_EQ(0.0, bspheres[handle1].center.z());

  bsphere = mask.getBoundingSphere();
  EXPECT_NEAR(sqrt(0.5 * 0.5 + 1.0 * 1.0 + 1.5 * 1.5), bsphere.radius, 1e-9);
  EXPECT_DOUBLE_EQ(0.0, bsphere.center.x());
  EXPECT_DOUBLE_EQ(0.0, bsphere.center.y());
  EXPECT_DOUBLE_EQ(0.0, bsphere.center.z());

  bsphereForContainsTest = mask.getBoundingSphereForContainsTestNoLock();
  EXPECT_NEAR(bsphere.radius, bsphereForContainsTest.radius, 1e-9);
  EXPECT_DOUBLE_EQ(bsphere.center.x(), bsphereForContainsTest.center.x());
  EXPECT_DOUBLE_EQ(bsphere.center.y(), bsphereForContainsTest.center.y());
  EXPECT_DOUBLE_EQ(bsphere.center.z(), bsphereForContainsTest.center.z());

  mask.setIgnoreInContainsTest({handle1});
  mask.updateBodyPoses();

  bsphere = mask.getBoundingSphere();
  EXPECT_NEAR(sqrt(0.5 * 0.5 + 1.0 * 1.0 + 1.5 * 1.5), bsphere.radius, 1e-9);
  EXPECT_DOUBLE_EQ(0.0, bsphere.center.x());
  EXPECT_DOUBLE_EQ(0.0, bsphere.center.y());
  EXPECT_DOUBLE_EQ(0.0, bsphere.center.z());

  bsphereForContainsTest = mask.getBoundingSphereForContainsTestNoLock();
  EXPECT_DOUBLE_EQ(0.0, bsphereForContainsTest.radius);
  EXPECT_DOUBLE_EQ(0.0, bsphereForContainsTest.center.x());
  EXPECT_DOUBLE_EQ(0.0, bsphereForContainsTest.center.y());
  EXPECT_DOUBLE_EQ(0.0, bsphereForContainsTest.center.z());

  // test the case when all transforms are unavailable
  auto cb4 = [handle2](ShapeHandle h, Eigen::Isometry3d &t) -> bool
  {
    t = Eigen::Isometry3d::Identity();
    return false;
  };
  mask.setTransformCallback(cb4);
  mask.updateBodyPoses();

  bspheres = mask.getBoundingSpheres();
  ASSERT_EQ(0, bspheres.size());
  ASSERT_EQ(bspheres.end(), bspheres.find(handle1));
  ASSERT_EQ(bspheres.end(), bspheres.find(handle2));

  bsphere = mask.getBoundingSphere();
  EXPECT_DOUBLE_EQ(0, bsphere.radius);
  EXPECT_DOUBLE_EQ(0.0, bsphere.center.x());
  EXPECT_DOUBLE_EQ(0.0, bsphere.center.y());
  EXPECT_DOUBLE_EQ(0.0, bsphere.center.z());

  bsphereForContainsTest = mask.getBoundingSphereForContainsTestNoLock();
  EXPECT_NEAR(bsphere.radius, bsphereForContainsTest.radius, 1e-9);
  EXPECT_DOUBLE_EQ(bsphere.center.x(), bsphereForContainsTest.center.x());
  EXPECT_DOUBLE_EQ(bsphere.center.y(), bsphereForContainsTest.center.y());
  EXPECT_DOUBLE_EQ(bsphere.center.z(), bsphereForContainsTest.center.z());
}

TEST(RayCastingShapeMask, UpdateBodyPoses)
{
  ros::Time::init();
  ros::Time::setNow(ros::Time(1));

  auto fooCb = [](ShapeHandle h, Eigen::Isometry3d &t) -> bool
  {
    return true;
  };
  TestMask mask(fooCb, 1.0, 10.0, true, true, true);

  shapes::ShapeConstPtr shape1(new shapes::Box(1.0, 2.0, 3.0));
  const auto handle1 = mask.addShape(shape1, 1.0, 0.0, false, "box");
  shapes::ShapeConstPtr shape2(new shapes::Sphere(2.0));
  const auto handle2 = mask.addShape(shape2, 2.0, 0.5, true, "sphere");

  size_t numCalled = 0;
  const Eigen::Isometry3d t1 = randomPose();
  const Eigen::Isometry3d t2 = randomPose();
  auto cb = [&](ShapeHandle h, Eigen::Isometry3d &t) -> bool
  {
    if (h == handle1)
      t = t1;
    else if (h == handle2)
      t = t2;
    else
      ADD_FAILURE();

    numCalled++;
    return true;
  };
  mask.setTransformCallback(cb);

  EXPECT_EQ(0, numCalled);
  mask.updateBodyPoses();
  EXPECT_EQ(2, numCalled);
  expectTransformsDoubleEq(t1, mask.getBodies()[handle1]->getPose());
  expectTransformsDoubleEq(t2, mask.getBodies()[handle2]->getPose());

  const Eigen::Isometry3d t3 = randomPose();
  const Eigen::Isometry3d t4 = randomPose();
  auto cb2 = [&](ShapeHandle h, Eigen::Isometry3d &t) -> bool
  {
    if (h == handle1)
      t = t3;
    else if (h == handle2)
      t = t4;
    else
      ADD_FAILURE();

    numCalled++;
    return true;
  };
  numCalled = 0;
  mask.setTransformCallback(cb2);

  EXPECT_EQ(0, numCalled);
  mask.updateBodyPoses();
  EXPECT_EQ(2, numCalled);
  expectTransformsDoubleEq(t3, mask.getBodies()[handle1]->getPose());
  expectTransformsDoubleEq(t4, mask.getBodies()[handle2]->getPose());
}

TEST(RayCastingShapeMask, ClassifyPoint)
{
  ros::Time::init();
  ros::Time::setNow(ros::Time(1));

  auto fooCb = [](ShapeHandle h, Eigen::Isometry3d &t) -> bool
  {
    return true;
  };
  TestMask mask(fooCb, 0.1, 10.0, false, false, false);

  shapes::ShapeConstPtr shape1(new shapes::Box(2.0, 2.0, 2.0));
  const auto handle1 = mask.addShape(shape1, 1.0, 0.0, false, "box");
  shapes::ShapeConstPtr shape2(new shapes::Sphere(1.375));
  const auto handle2 = mask.addShape(shape2, 1.0, 0.0, false, "sphere");
  shapes::ShapeConstPtr shapeSensor(new shapes::Box(1.0, 1.0, 1.0));
  const auto handleSensor = mask.addShape(shapeSensor, 0.1, 0.0, true, "sensor");

  const Eigen::Vector3d sensorPos(-1.5, 0.0, 0.0);

  auto cb = [&](ShapeHandle h, Eigen::Isometry3d &t) -> bool
  {
    t = Eigen::Isometry3d::Identity();
    if (h == handleSensor) {
      t.translate(sensorPos);
    }
    return true;
  };
  mask.setTransformCallback(cb);

  mask.updateBodyPoses();
  auto bsphere = mask.getBoundingSphereForContainsTestNoLock();

  RayCastingShapeMask::MaskValue val;

  // This is a test set of points.
  // For an overview, open test_ray_casting_shape_mask.blend in Blender 2.80+.
  Eigen::Vector3d pointSensor(sensorPos);
  Eigen::Vector3d pointSensor2(-1.47, 0, 0);
  Eigen::Vector3d pointClipMin(-1.42, 0, 0);
  Eigen::Vector3d pointClipMax(10, 0, 0);
  Eigen::Vector3d pointInBox(0.95, 0.95, 0.95);
  Eigen::Vector3d pointInSphere(1.35, 0, 0);
  Eigen::Vector3d pointInBoth(0, 0, 0);
  Eigen::Vector3d pointShadowBox(-0.25, -2, 2);
  Eigen::Vector3d pointShadowSphere(-0.560762, 0, 1.83871);
  Eigen::Vector3d pointShadowBoth(-sensorPos);
  Eigen::Vector3d pointOutside(-3, 0, 0);

  // doClipping, doContainsTest and doShadowTest are all false, so only OUTSIDE is possible
  mask.classifyPointNoLock(pointSensor, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(pointSensor2, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(pointClipMin, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(pointClipMax, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(pointInBox, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(pointInSphere, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(pointInBoth, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(pointShadowBox, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(pointShadowSphere, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(pointShadowBoth, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(pointOutside, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);

  mask.doClipping = true;
  mask.doContainsTest = false;
  mask.doShadowTest = false;
  mask.classifyPointNoLock(pointSensor, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::CLIP, val);
  mask.classifyPointNoLock(pointSensor2, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::CLIP, val);
  mask.classifyPointNoLock(pointClipMin, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::CLIP, val);
  mask.classifyPointNoLock(pointClipMax, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::CLIP, val);
  mask.classifyPointNoLock(pointInBox, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(pointInSphere, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(pointInBoth, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(pointShadowBox, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(pointShadowSphere, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(pointShadowBoth, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(pointOutside, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);

  mask.doClipping = false;
  mask.doContainsTest = true;
  mask.doShadowTest = false;
  mask.classifyPointNoLock(pointSensor, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::INSIDE, val);
  mask.classifyPointNoLock(pointSensor2, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::INSIDE, val);
  mask.classifyPointNoLock(pointClipMin, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(pointClipMax, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(pointInBox, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::INSIDE, val);
  mask.classifyPointNoLock(pointInSphere, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::INSIDE, val);
  mask.classifyPointNoLock(pointInBoth, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::INSIDE, val);
  mask.classifyPointNoLock(pointShadowBox, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(pointShadowSphere, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(pointShadowBoth, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(pointOutside, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);

  // shadow filtering with sensor body not excluded - everything is shadowed by the sensor body
  mask.doClipping = false;
  mask.doContainsTest = false;
  mask.doShadowTest = true;
  mask.classifyPointNoLock(pointSensor, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val); // no ray, so no shadow
  mask.classifyPointNoLock(pointSensor2, val, sensorPos, bsphere);
  // the sensor is not considered to shadow points inside itself
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(pointClipMin, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, val);
  mask.classifyPointNoLock(pointClipMax, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, val);
  mask.classifyPointNoLock(pointInBox, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, val);
  mask.classifyPointNoLock(pointInSphere, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, val);
  mask.classifyPointNoLock(pointInBoth, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, val);
  mask.classifyPointNoLock(pointShadowBox, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, val);
  mask.classifyPointNoLock(pointShadowSphere, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, val);
  mask.classifyPointNoLock(pointShadowBoth, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, val);
  mask.classifyPointNoLock(pointOutside, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, val);

  mask.setIgnoreInShadowTest({handleSensor});
  mask.doClipping = false;
  mask.doContainsTest = false;
  mask.doShadowTest = true;
  mask.classifyPointNoLock(pointSensor, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(pointSensor2, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(pointClipMin, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(pointClipMax, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, val);
  mask.classifyPointNoLock(pointInBox, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, val);
  mask.classifyPointNoLock(pointInSphere, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, val);
  mask.classifyPointNoLock(pointInBoth, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, val);
  mask.classifyPointNoLock(pointShadowBox, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, val);
  mask.classifyPointNoLock(pointShadowSphere, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, val);
  mask.classifyPointNoLock(pointShadowBoth, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, val);
  mask.classifyPointNoLock(pointOutside, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);

  mask.doClipping = true;
  mask.doContainsTest = true;
  mask.doShadowTest = false;
  mask.classifyPointNoLock(pointSensor, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::CLIP, val);
  mask.classifyPointNoLock(pointSensor2, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::CLIP, val);
  mask.classifyPointNoLock(pointClipMin, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::CLIP, val);
  mask.classifyPointNoLock(pointClipMax, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::CLIP, val);
  mask.classifyPointNoLock(pointInBox, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::INSIDE, val);
  mask.classifyPointNoLock(pointInSphere, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::INSIDE, val);
  mask.classifyPointNoLock(pointInBoth, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::INSIDE, val);
  mask.classifyPointNoLock(pointShadowBox, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(pointShadowSphere, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(pointShadowBoth, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(pointOutside, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);

  mask.doClipping = true;
  mask.doContainsTest = false;
  mask.doShadowTest = true;
  mask.classifyPointNoLock(pointSensor, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::CLIP, val);
  mask.classifyPointNoLock(pointSensor2, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::CLIP, val);
  mask.classifyPointNoLock(pointClipMin, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::CLIP, val);
  mask.classifyPointNoLock(pointClipMax, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::CLIP, val);
  mask.classifyPointNoLock(pointInBox, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, val);
  mask.classifyPointNoLock(pointInSphere, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, val);
  mask.classifyPointNoLock(pointInBoth, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, val);
  mask.classifyPointNoLock(pointShadowBox, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, val);
  mask.classifyPointNoLock(pointShadowSphere, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, val);
  mask.classifyPointNoLock(pointShadowBoth, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, val);
  mask.classifyPointNoLock(pointOutside, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);

  mask.doClipping = false;
  mask.doContainsTest = true;
  mask.doShadowTest = true;
  mask.classifyPointNoLock(pointSensor, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::INSIDE, val);
  mask.classifyPointNoLock(pointSensor2, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::INSIDE, val);
  mask.classifyPointNoLock(pointClipMin, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(pointClipMax, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, val);
  mask.classifyPointNoLock(pointInBox, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::INSIDE, val);
  mask.classifyPointNoLock(pointInSphere, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::INSIDE, val);
  mask.classifyPointNoLock(pointInBoth, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::INSIDE, val);
  mask.classifyPointNoLock(pointShadowBox, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, val);
  mask.classifyPointNoLock(pointShadowSphere, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, val);
  mask.classifyPointNoLock(pointShadowBoth, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, val);
  mask.classifyPointNoLock(pointOutside, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);

  mask.doClipping = true;
  mask.doContainsTest = true;
  mask.doShadowTest = true;
  mask.classifyPointNoLock(pointSensor, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::CLIP, val);
  mask.classifyPointNoLock(pointSensor2, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::CLIP, val);
  mask.classifyPointNoLock(pointClipMin, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::CLIP, val);
  mask.classifyPointNoLock(pointClipMax, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::CLIP, val);
  mask.classifyPointNoLock(pointInBox, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::INSIDE, val);
  mask.classifyPointNoLock(pointInSphere, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::INSIDE, val);
  mask.classifyPointNoLock(pointInBoth, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::INSIDE, val);
  mask.classifyPointNoLock(pointShadowBox, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, val);
  mask.classifyPointNoLock(pointShadowSphere, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, val);
  mask.classifyPointNoLock(pointShadowBoth, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, val);
  mask.classifyPointNoLock(pointOutside, val, sensorPos, bsphere);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);

  // test that the containment filter depends on the correct bounding sphere
  auto bsphereSmall = bsphere;
  bsphereSmall.radius /= 10.0;
  mask.classifyPointNoLock(pointInBox, val, sensorPos, bsphereSmall);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, val);
  mask.classifyPointNoLock(pointOutside, val, sensorPos, bsphereSmall);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);

  // test that the containment filter works even when the bounding sphere is too large
  auto bsphereBig = bsphere;
  bsphereBig.radius *= 10.0;
  mask.classifyPointNoLock(pointInBox, val, sensorPos, bsphereBig);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::INSIDE, val);
  mask.classifyPointNoLock(pointOutside, val, sensorPos, bsphereBig);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  
  random_numbers::RandomNumberGenerator rng;
  Eigen::Vector3d pointInside;
  for (size_t i = 0; i < 100; ++i)
  {
    if (mask.getBodies()[handle1]->samplePointInside(rng, 10, pointInside)) {
      mask.classifyPointNoLock(pointInside, val, sensorPos, bsphere);
      EXPECT_EQ(RayCastingShapeMask::MaskValue::INSIDE, val);
    }
    if (mask.getBodies()[handle2]->samplePointInside(rng, 10, pointInside)) {
      mask.classifyPointNoLock(pointInside, val, sensorPos, bsphere);
      EXPECT_EQ(RayCastingShapeMask::MaskValue::INSIDE, val);
    }
  }
}

TEST(RayCastingShapeMask, Mask)
{
  ros::Time::init();
  ros::Time::setNow(ros::Time(1));

  auto fooCb = [](ShapeHandle h, Eigen::Isometry3d &t) -> bool
  {
    return true;
  };
  TestMask mask(fooCb, 0.1, 10.0, true, true, true);

  shapes::ShapeConstPtr shape1(new shapes::Box(2.0, 2.0, 2.0));
  const auto handle1 = mask.addShape(shape1, 1.0, 0.0, false, "box");
  shapes::ShapeConstPtr shape2(new shapes::Sphere(1.375));
  const auto handle2 = mask.addShape(shape2, 1.0, 0.0, false, "sphere");
  shapes::ShapeConstPtr shapeSensor(new shapes::Box(1.0, 1.0, 1.0));
  const auto handleSensor = mask.addShape(shapeSensor, 0.1, 0.0, true, "sensor");

  const Eigen::Vector3d sensorPos(-1.5, 0.0, 0.0);

  auto cb = [&](ShapeHandle h, Eigen::Isometry3d &t) -> bool
  {
    t = Eigen::Isometry3d::Identity();
    if (h == handleSensor)
    {
      t.translate(sensorPos);
    }
    return true;
  };
  mask.setTransformCallback(cb);
  mask.setIgnoreInShadowTest({handleSensor});

  RayCastingShapeMask::MaskValue val;

  // This is a test set of points.
  // For an overview, open test_ray_casting_shape_mask.blend in Blender 2.80+.
  Eigen::Vector3f pointSensor(sensorPos.x(), sensorPos.y(), sensorPos.z());
  Eigen::Vector3f pointSensor2(-1.47, 0, 0);
  Eigen::Vector3f pointClipMin(-1.42, 0, 0);
  Eigen::Vector3f pointClipMax(10, 0, 0);
  Eigen::Vector3f pointInBox(0.95, 0.95, 0.95);
  Eigen::Vector3f pointInSphere(1.35, 0, 0);
  Eigen::Vector3f pointInBoth(0, 0, 0);
  Eigen::Vector3f pointShadowBox(-0.25, -2, 2);
  Eigen::Vector3f pointShadowSphere(-0.560762, 0, 1.83871);
  Eigen::Vector3f pointShadowBoth(-sensorPos.x(), 0, 0);
  Eigen::Vector3f pointOutside(-3, 0, 0);

  mask.maskContainmentAndShadows(pointSensor, val, sensorPos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::CLIP, val);
  mask.maskContainmentAndShadows(pointSensor2, val, sensorPos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::CLIP, val);
  mask.maskContainmentAndShadows(pointClipMin, val, sensorPos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::CLIP, val);
  mask.maskContainmentAndShadows(pointClipMax, val, sensorPos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::CLIP, val);
  mask.maskContainmentAndShadows(pointInBox, val, sensorPos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::INSIDE, val);
  mask.maskContainmentAndShadows(pointInSphere, val, sensorPos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::INSIDE, val);
  mask.maskContainmentAndShadows(pointInBoth, val, sensorPos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::INSIDE, val);
  mask.maskContainmentAndShadows(pointShadowBox, val, sensorPos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, val);
  mask.maskContainmentAndShadows(pointShadowSphere, val, sensorPos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, val);
  mask.maskContainmentAndShadows(pointShadowBoth, val, sensorPos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, val);
  mask.maskContainmentAndShadows(pointOutside, val, sensorPos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);

  Cloud cloud;
  CloudModifier mod(cloud);
  mod.setPointCloud2FieldsByString(1, "xyz");
  mod.resize(11);
  CloudIter it_x(cloud, "x");
  CloudIter it_y(cloud, "y");
  CloudIter it_z(cloud, "z");

  Eigen::Vector3f p;
  p = pointSensor; *it_x = p.x(); *it_y = p.y(); *it_z = p.z(); ++it_x; ++it_y; ++it_z;
  p = pointSensor2; *it_x = p.x(); *it_y = p.y(); *it_z = p.z(); ++it_x; ++it_y; ++it_z;
  p = pointClipMin; *it_x = p.x(); *it_y = p.y(); *it_z = p.z(); ++it_x; ++it_y; ++it_z;
  p = pointClipMax; *it_x = p.x(); *it_y = p.y(); *it_z = p.z(); ++it_x; ++it_y; ++it_z;
  p = pointInBox; *it_x = p.x(); *it_y = p.y(); *it_z = p.z(); ++it_x; ++it_y; ++it_z;
  p = pointInSphere; *it_x = p.x(); *it_y = p.y(); *it_z = p.z(); ++it_x; ++it_y; ++it_z;
  p = pointInBoth; *it_x = p.x(); *it_y = p.y(); *it_z = p.z(); ++it_x; ++it_y; ++it_z;
  p = pointShadowBox; *it_x = p.x(); *it_y = p.y(); *it_z = p.z(); ++it_x; ++it_y; ++it_z;
  p = pointShadowSphere; *it_x = p.x(); *it_y = p.y(); *it_z = p.z(); ++it_x; ++it_y; ++it_z;
  p = pointShadowBoth; *it_x = p.x(); *it_y = p.y(); *it_z = p.z(); ++it_x; ++it_y; ++it_z;
  p = pointOutside; *it_x = p.x(); *it_y = p.y(); *it_z = p.z(); ++it_x; ++it_y; ++it_z;

  std::vector<RayCastingShapeMask::MaskValue> vals;
  vals.push_back(RayCastingShapeMask::MaskValue::INSIDE); // be adverse and pass garbage
  mask.maskContainmentAndShadows(cloud, vals, sensorPos);

  ASSERT_EQ(11, vals.size());
  EXPECT_EQ(RayCastingShapeMask::MaskValue::CLIP, vals[0]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::CLIP, vals[1]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::CLIP, vals[2]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::CLIP, vals[3]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::INSIDE, vals[4]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::INSIDE, vals[5]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::INSIDE, vals[6]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, vals[7]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, vals[8]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, vals[9]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, vals[10]);
}

TEST(RayCastingShapeMask, MaskPerformancePoints)
{
  ros::Time::init();
  ros::Time::setNow(ros::Time(1));

  auto fooCb = [](ShapeHandle h, Eigen::Isometry3d &t) -> bool
  {
    return true;
  };
  TestMask mask(fooCb, 0.1, 10.0, true, true, true);

  shapes::ShapeConstPtr shape1(new shapes::Box(2.0, 2.0, 2.0));
  const auto handle1 = mask.addShape(shape1, 1.0, 0.0, false, "box");
  shapes::ShapeConstPtr shape2(new shapes::Sphere(1.375));
  const auto handle2 = mask.addShape(shape2, 1.0, 0.0, false, "sphere");
  shapes::ShapeConstPtr shapeSensor(new shapes::Box(1.0, 1.0, 1.0));
  const auto handleSensor = mask.addShape(shapeSensor, 0.1, 0.0, true, "sensor");

  const Eigen::Vector3d sensorPos(-1.5, 0.0, 0.0);

  auto cb = [&](ShapeHandle h, Eigen::Isometry3d &t) -> bool
  {
    t = Eigen::Isometry3d::Identity();
    if (h == handleSensor)
    {
      t.translate(sensorPos);
    }
    return true;
  };
  mask.setTransformCallback(cb);
  mask.setIgnoreInShadowTest({handleSensor});

#if RELEASE_BUILD == 1
  const size_t numPoints = 10000000;
#else
  const size_t numPoints = 100000;
#endif

  Cloud cloud;
  CloudModifier mod(cloud);
  mod.setPointCloud2FieldsByString(1, "xyz");
  mod.resize(numPoints);
  CloudIter it_x(cloud, "x");
  CloudIter it_y(cloud, "y");
  CloudIter it_z(cloud, "z");

  // generate a bunch of points close the the bodies so that all filtering parts get activated
  random_numbers::RandomNumberGenerator rng;
  Eigen::Vector3d p;
  auto body1 = mask.getBodies()[handle1];
  auto body2 = mask.getBodies()[handle2];
  for (size_t i = 0; i < numPoints / 2; ++i)
  {
    while (!body1->samplePointInside(rng, 10, p)) {}
    p = 2 * p;
    *it_x = p.x(); *it_y = p.y(); *it_z = p.z();
    ++it_x; ++it_y; ++it_z;

    while (!body2->samplePointInside(rng, 10, p)) {}
    p = 2 * p;
    *it_x = p.x(); *it_y = p.y(); *it_z = p.z();
    ++it_x; ++it_y; ++it_z;
  }

  std::vector<RayCastingShapeMask::MaskValue> vals;

  ros::WallTime start = ros::WallTime::now();
  mask.maskContainmentAndShadows(cloud, vals, sensorPos);
  ros::WallTime end = ros::WallTime::now();

  ASSERT_EQ(numPoints, vals.size());
#if RELEASE_BUILD == 1
  EXPECT_GT(2.0, (end - start).toSec());
#else
  EXPECT_GT(1.5, (end - start).toSec());
#endif
}

TEST(RayCastingShapeMask, MaskPerformanceBodies)
{
  ros::Time::init();
  ros::Time::setNow(ros::Time(1));

  auto cb = [](ShapeHandle h, Eigen::Isometry3d &t) -> bool
  {
    t = randomPose();
    t.translation().normalize();
    t.translation() *= 10.0; // so that we don't get too far behind the clipping plane
    return true;
  };
  TestMask mask(cb, 0.1, 10.0, true, true, true);

#if RELEASE_BUILD == 1
  const size_t numBodies = 1000000;
#else
  const size_t numBodies = 20000;
#endif

  for (size_t i = 0; i < numBodies / 2; ++i)
  {
    shapes::ShapeConstPtr shape1(new shapes::Box(2.0, 2.0, 2.0));
    mask.addShape(shape1, 1.0, 0.0, false, "box");
    shapes::ShapeConstPtr shape2(new shapes::Sphere(1.375));
    mask.addShape(shape2, 1.0, 0.0, false, "sphere");
  }
  mask.updateInternalShapeLists();

  const Eigen::Vector3d sensorPos(0.0, 0.0, 0.0);

  const size_t numPoints = 10;

  Cloud cloud;
  CloudModifier mod(cloud);
  mod.setPointCloud2FieldsByString(1, "xyz");
  mod.resize(numPoints);
  CloudIter it_x(cloud, "x");
  CloudIter it_y(cloud, "y");
  CloudIter it_z(cloud, "z");

  // generate a bunch of points close the the bodies so that all filtering parts get activated
  Eigen::Vector3d p;
  for (size_t i = 0; i < numPoints; ++i)
  {
    p = Eigen::Vector3d::Random();
    p.normalize();
    p *= 10.0;
    *it_x = p.x(); *it_y = p.y(); *it_z = p.z();
    ++it_x; ++it_y; ++it_z;
  }

  std::vector<RayCastingShapeMask::MaskValue> vals;

  ros::WallTime start = ros::WallTime::now();
  mask.maskContainmentAndShadows(cloud, vals, sensorPos);
  ros::WallTime end = ros::WallTime::now();

  ASSERT_EQ(numPoints, vals.size());
#if RELEASE_BUILD == 1
  EXPECT_GT(1.0, (end - start).toSec());
#else
  EXPECT_GT(2.0, (end - start).toSec());
#endif
}

TEST(RayCastingShapeMask, MaskPerformanceBodiesMesh)
{
  ros::Time::init();
  ros::Time::setNow(ros::Time(1));

  auto cb = [](ShapeHandle h, Eigen::Isometry3d &t) -> bool
  {
    t = randomPose();
    t.translation().normalize();
    t.translation() *= 10.0; // so that we don't get too far behind the clipping plane
    return true;
  };
  TestMask mask(cb, 0.1, 10.0, true, true, true);

#if RELEASE_BUILD == 1
  const size_t numBodies = 10000;
#else
  const size_t numBodies = 5000;
#endif

  auto g = urdf::Mesh();
  g.scale = {1.0, 2.0, 3.0};
  g.filename = "package://robot_body_filter/test/box.dae";
  for (size_t i = 0; i < numBodies; ++i)
  {
    const auto shape = robot_body_filter::constructShape(g);
    mask.addShape(shape, 1.0, 0.0, false, "mesh");
  }
  mask.updateInternalShapeLists();

  const Eigen::Vector3d sensorPos(0.0, 0.0, 0.0);

#if RELEASE_BUILD == 1
  const size_t numPoints = 10000;
#else
  const size_t numPoints = 10;
#endif

  Cloud cloud;
  CloudModifier mod(cloud);
  mod.setPointCloud2FieldsByString(1, "xyz");
  mod.resize(numPoints);
  CloudIter it_x(cloud, "x");
  CloudIter it_y(cloud, "y");
  CloudIter it_z(cloud, "z");

  // generate a bunch of points close the the bodies so that all filtering parts get activated
  Eigen::Vector3d p;
  for (size_t i = 0; i < numPoints; ++i)
  {
    p = Eigen::Vector3d::Random();
    p.normalize();
    p *= 10.0;
    *it_x = p.x(); *it_y = p.y(); *it_z = p.z();
    ++it_x; ++it_y; ++it_z;
  }

  std::vector<RayCastingShapeMask::MaskValue> vals;

  ros::WallTime start = ros::WallTime::now();
  mask.maskContainmentAndShadows(cloud, vals, sensorPos);
  ros::WallTime end = ros::WallTime::now();

  ASSERT_EQ(numPoints, vals.size());
#if RELEASE_BUILD == 1
  EXPECT_GT(0.1, (end - start).toSec());
#else
  EXPECT_GT(1.0, (end - start).toSec());
#endif
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}