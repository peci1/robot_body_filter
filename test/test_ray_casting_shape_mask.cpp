// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include "gtest/gtest.h"

#include <functional>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cras_cpp_common/cloud.hpp>
#include <geometric_shapes/shapes.h>
#include <moveit/point_containment_filter/shape_mask.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>
#include <robot_body_filter/RayCastingShapeMask.hpp>
#include <robot_body_filter/utils/shapes.hpp>
#include <urdf_model/link.h>

#include "utils.cpp"  // NOLINT

using point_containment_filter::ShapeHandle;
using robot_body_filter::constructShape;
using robot_body_filter::RayCastingShapeMask;

class TestMask : public robot_body_filter::RayCastingShapeMask {
public:
  TestMask(
    rclcpp::Node& node, const ShapeMask::TransformCallback& transform_callback,
    double min_sensor_dist, double max_sensor_dist, bool do_clipping, bool do_contains_test, bool do_shadow_test)
    : RayCastingShapeMask(node.get_logger(), node.get_clock(), transform_callback,
      min_sensor_dist, max_sensor_dist, do_clipping, do_contains_test, do_shadow_test) {
  }

  friend class RayCastingShapeMask_Basic_Test;
  friend class RayCastingShapeMask_Bspheres_Test;
  friend class RayCastingShapeMask_ClassifyPoint_Test;
};

// Code for getting std::function address, from https://stackoverflow.com/q/18039723/1076564

template<typename Function>
struct function_traits : function_traits<decltype(&Function::operator())> {};

template<typename ClassType, typename ReturnType, typename... Args>
struct function_traits<ReturnType(ClassType::*)(Args...) const> {
  using pointer = ReturnType(*)(Args...);
  using function = std::function<ReturnType(Args...)>;
};

template<typename Function>
typename function_traits<Function>::function to_function(Function& lambda) {
  return static_cast<typename function_traits<Function>::function>(lambda);
}

template<typename Lambda>
size_t getAddress(Lambda lambda) {
  auto function = new decltype(to_function(lambda))(to_function(lambda));
  auto func = static_cast<void*>(function);
  return reinterpret_cast<size_t>(func);
}

TEST(RayCastingShapeMask, Basic) {
  rclcpp::Node nh("test_ray_casting_shape_mask");
  auto cb = [](ShapeHandle, Eigen::Isometry3d& t) -> bool {
    t = Eigen::Isometry3d::Identity();
    return true;
  };
  TestMask mask(nh, cb, 1.0, 10.0, true, false, false);

  EXPECT_NE(0, getAddress(mask.transform_callback_));
  EXPECT_TRUE(mask.do_clipping_);
  EXPECT_FALSE(mask.do_contains_test_);
  EXPECT_FALSE(mask.do_shadow_test_);
  EXPECT_DOUBLE_EQ(1.0, mask.min_sensor_dist_);
  EXPECT_DOUBLE_EQ(10.0, mask.max_sensor_dist_);
  EXPECT_TRUE(mask.bodies_.empty());
  EXPECT_TRUE(mask.getBodiesForContainsTest().empty());
  EXPECT_TRUE(mask.getBodiesForShadowTest().empty());
  EXPECT_TRUE(mask.bspheres_.empty());
  EXPECT_TRUE(mask.ignore_in_contains_test_.empty());
  EXPECT_TRUE(mask.ignore_in_shadow_test_.empty());

  shapes::ShapeConstPtr shape(new shapes::Box(1.0, 2.0, 3.0));
  auto handle = mask.addShape(shape, 1.0, 0.0, true, "box");
  EXPECT_EQ(1, mask.bodies_.size());
  EXPECT_EQ(1, mask.getBodiesForContainsTest().size());
  EXPECT_EQ(1, mask.getBodiesForShadowTest().size());
  // bspheres are only updated by calling updateBodyPoses()
  EXPECT_EQ(0, mask.bspheres_.size());
  EXPECT_EQ(0, mask.bspheres_body_indices_.size());
  EXPECT_EQ(0, mask.bspheres_for_contains_test_.size());
  EXPECT_EQ(0, mask.bspheres_for_contains_test_body_indices_.size());
  EXPECT_EQ(handle.contains, handle.shadow);

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
  EXPECT_TRUE(mask.ignore_in_contains_test_.empty());
  EXPECT_TRUE(mask.ignore_in_shadow_test_.empty());

  // test adding without updating internal structures
  handle = mask.addShape(shape, 1.0, 0.0, false, "box");
  EXPECT_EQ(1, mask.bodies_.size());
  EXPECT_EQ(0, mask.getBodiesForContainsTest().size());
  EXPECT_EQ(0, mask.getBodiesForShadowTest().size());
  EXPECT_EQ(handle.contains, handle.shadow);

  mask.updateInternalShapeLists();
  EXPECT_EQ(1, mask.bodies_.size());
  EXPECT_EQ(1, mask.getBodiesForContainsTest().size());
  EXPECT_EQ(1, mask.getBodiesForShadowTest().size());

  handle = mask.addShape(shape, 1.0, 0.0, 2.0, 1.0, 1.0, 0.0, 1.0, 0.0, true, "doubleBox");
  EXPECT_EQ(3, mask.bodies_.size());
  EXPECT_EQ(2, mask.getBodiesForContainsTest().size());
  EXPECT_EQ(2, mask.getBodiesForShadowTest().size());
  EXPECT_NE(handle.contains, handle.shadow);

  mask.removeShape(handle);
  EXPECT_EQ(1, mask.bodies_.size());
  EXPECT_EQ(1, mask.getBodiesForContainsTest().size());
  EXPECT_EQ(1, mask.getBodiesForShadowTest().size());

  auto cb2 = [](ShapeHandle, Eigen::Isometry3d& t) -> bool {
    t = Eigen::Isometry3d::Identity();
    return false;
  };
  const auto prev_address = getAddress(mask.transform_callback_);
  mask.setTransformCallback(cb2);
  EXPECT_NE(prev_address, getAddress(mask.transform_callback_));
}

TEST(RayCastingShapeMask, Bspheres) {
  rclcpp::Node nh("test_ray_casting_shape_mask");
  auto cb = [](ShapeHandle, Eigen::Isometry3d& t) -> bool {
    t = Eigen::Isometry3d::Identity();
    return true;
  };
  TestMask mask(nh, cb, 1.0, 10.0, true, true, true);

  shapes::ShapeConstPtr shape1(new shapes::Box(1.0, 2.0, 3.0));
  const auto multi_handle1 = mask.addShape(shape1, 1.0, 0.0, false, "box");
  const auto handle1 = multi_handle1.contains;
  shapes::ShapeConstPtr shape2(new shapes::Sphere(2.0));
  const auto multi_handle2 = mask.addShape(shape2, 2.0, 0.5, false, "sphere");
  const auto handle2 = multi_handle2.contains;
  const auto multi_handle3 = mask.addShape(shape1, 1.0, 0.0, 2.0, 1.0, 1.0, 0.0, 1.0, 0.0, true, "doubleBox");
  EXPECT_NE(multi_handle3.contains, multi_handle3.shadow);
  const auto handle3_contains = multi_handle3.contains;
  const auto handle3_shadow = multi_handle3.shadow;

  mask.updateBodyPoses();

  auto bspheres = mask.getBoundingSpheres();
  auto bspheres_contains = mask.getBoundingSpheresForContainsTest();
  ASSERT_EQ(4, bspheres.size());
  ASSERT_EQ(3, bspheres_contains.size());
  ASSERT_NE(bspheres.end(), bspheres.find(handle1));
  ASSERT_NE(bspheres.end(), bspheres.find(handle2));
  ASSERT_NE(bspheres.end(), bspheres.find(handle3_contains));
  ASSERT_NE(bspheres.end(), bspheres.find(handle3_shadow));
  ASSERT_NE(bspheres_contains.end(), bspheres_contains.find(handle1));
  ASSERT_NE(bspheres_contains.end(), bspheres_contains.find(handle2));
  ASSERT_NE(bspheres_contains.end(), bspheres_contains.find(handle3_contains));
  ASSERT_EQ(bspheres_contains.end(), bspheres_contains.find(handle3_shadow));

  EXPECT_NEAR(sqrt(0.5 * 0.5 + 1.0 * 1.0 + 1.5 * 1.5), bspheres[handle1].radius, 1e-9);
  EXPECT_DOUBLE_EQ(0.0, bspheres[handle1].center.x());
  EXPECT_DOUBLE_EQ(0.0, bspheres[handle1].center.y());
  EXPECT_DOUBLE_EQ(0.0, bspheres[handle1].center.z());
  EXPECT_NEAR(4.5, bspheres[handle2].radius, 1e-9);
  EXPECT_DOUBLE_EQ(0.0, bspheres[handle2].center.x());
  EXPECT_DOUBLE_EQ(0.0, bspheres[handle2].center.y());
  EXPECT_DOUBLE_EQ(0.0, bspheres[handle2].center.z());
  EXPECT_NEAR(sqrt(0.5 * 0.5 + 1.0 * 1.0 + 1.5 * 1.5), bspheres[handle3_contains].radius, 1e-9);
  EXPECT_DOUBLE_EQ(0.0, bspheres[handle3_contains].center.x());
  EXPECT_DOUBLE_EQ(0.0, bspheres[handle3_contains].center.y());
  EXPECT_DOUBLE_EQ(0.0, bspheres[handle3_contains].center.z());
  EXPECT_NEAR(sqrt(2.0 * 2.0 + 3.0 * 3.0 + 4.0 * 4.0), bspheres[handle3_shadow].radius, 1e-9);
  EXPECT_DOUBLE_EQ(0.0, bspheres[handle3_shadow].center.x());
  EXPECT_DOUBLE_EQ(0.0, bspheres[handle3_shadow].center.y());
  EXPECT_DOUBLE_EQ(0.0, bspheres[handle3_shadow].center.z());

  auto bsphere = mask.getBoundingSphere();
  EXPECT_NEAR(bspheres[handle3_shadow].radius, bsphere.radius, 1e-9);
  EXPECT_DOUBLE_EQ(0.0, bsphere.center.x());
  EXPECT_DOUBLE_EQ(0.0, bsphere.center.y());
  EXPECT_DOUBLE_EQ(0.0, bsphere.center.z());

  auto bsphere_for_contains_test = mask.getBoundingSphereForContainsTestNoLock();
  EXPECT_NEAR(4.5, bsphere_for_contains_test.radius, 1e-9);
  EXPECT_DOUBLE_EQ(bsphere.center.x(), bsphere_for_contains_test.center.x());
  EXPECT_DOUBLE_EQ(bsphere.center.y(), bsphere_for_contains_test.center.y());
  EXPECT_DOUBLE_EQ(bsphere.center.z(), bsphere_for_contains_test.center.z());

  auto cb2 = [](ShapeHandle, Eigen::Isometry3d& t) -> bool {
    t = Eigen::Isometry3d::Identity();
    t.translate(Eigen::Vector3d(1.0, 2.0, 3.0));
    return true;
  };
  mask.setTransformCallback(cb2);
  mask.updateBodyPoses();

  bspheres = mask.getBoundingSpheres();
  ASSERT_EQ(4, bspheres.size());
  ASSERT_NE(bspheres.end(), bspheres.find(handle1));
  ASSERT_NE(bspheres.end(), bspheres.find(handle2));
  ASSERT_NE(bspheres.end(), bspheres.find(handle3_contains));
  ASSERT_NE(bspheres.end(), bspheres.find(handle3_shadow));

  EXPECT_NEAR(sqrt(0.5 * 0.5 + 1.0 * 1.0 + 1.5 * 1.5), bspheres[handle1].radius, 1e-9);
  EXPECT_DOUBLE_EQ(1.0, bspheres[handle1].center.x());
  EXPECT_DOUBLE_EQ(2.0, bspheres[handle1].center.y());
  EXPECT_DOUBLE_EQ(3.0, bspheres[handle1].center.z());
  EXPECT_NEAR(4.5, bspheres[handle2].radius, 1e-9);
  EXPECT_DOUBLE_EQ(1.0, bspheres[handle2].center.x());
  EXPECT_DOUBLE_EQ(2.0, bspheres[handle2].center.y());
  EXPECT_DOUBLE_EQ(3.0, bspheres[handle2].center.z());
  EXPECT_NEAR(sqrt(0.5 * 0.5 + 1.0 * 1.0 + 1.5 * 1.5), bspheres[handle3_contains].radius, 1e-9);
  EXPECT_DOUBLE_EQ(1.0, bspheres[handle3_contains].center.x());
  EXPECT_DOUBLE_EQ(2.0, bspheres[handle3_contains].center.y());
  EXPECT_DOUBLE_EQ(3.0, bspheres[handle3_contains].center.z());
  EXPECT_NEAR(sqrt(2.0 * 2.0 + 3.0 * 3.0 + 4.0 * 4.0), bspheres[handle3_shadow].radius, 1e-9);
  EXPECT_DOUBLE_EQ(1.0, bspheres[handle3_shadow].center.x());
  EXPECT_DOUBLE_EQ(2.0, bspheres[handle3_shadow].center.y());
  EXPECT_DOUBLE_EQ(3.0, bspheres[handle3_shadow].center.z());

  bsphere = mask.getBoundingSphere();
  EXPECT_NEAR(bspheres[handle3_shadow].radius, bsphere.radius, 1e-9);
  EXPECT_DOUBLE_EQ(1.0, bsphere.center.x());
  EXPECT_DOUBLE_EQ(2.0, bsphere.center.y());
  EXPECT_DOUBLE_EQ(3.0, bsphere.center.z());

  bsphere_for_contains_test = mask.getBoundingSphereForContainsTestNoLock();
  EXPECT_NEAR(4.5, bsphere_for_contains_test.radius, 1e-9);
  EXPECT_DOUBLE_EQ(bsphere.center.x(), bsphere_for_contains_test.center.x());
  EXPECT_DOUBLE_EQ(bsphere.center.y(), bsphere_for_contains_test.center.y());
  EXPECT_DOUBLE_EQ(bsphere.center.z(), bsphere_for_contains_test.center.z());

  // make the sphere's position unresolvable
  auto cb3 = [handle2](ShapeHandle h, Eigen::Isometry3d& t) -> bool {
    t = Eigen::Isometry3d::Identity();
    return h != handle2;
  };
  mask.setTransformCallback(cb3);
  mask.updateBodyPoses();

  bspheres = mask.getBoundingSpheres();
  ASSERT_EQ(3, bspheres.size());
  ASSERT_NE(bspheres.end(), bspheres.find(handle1));
  ASSERT_EQ(bspheres.end(), bspheres.find(handle2));
  ASSERT_NE(bspheres.end(), bspheres.find(handle3_contains));
  ASSERT_NE(bspheres.end(), bspheres.find(handle3_shadow));

  EXPECT_NEAR(sqrt(0.5 * 0.5 + 1.0 * 1.0 + 1.5 * 1.5), bspheres[handle1].radius, 1e-9);
  EXPECT_DOUBLE_EQ(0.0, bspheres[handle1].center.x());
  EXPECT_DOUBLE_EQ(0.0, bspheres[handle1].center.y());
  EXPECT_DOUBLE_EQ(0.0, bspheres[handle1].center.z());

  bsphere = mask.getBoundingSphere();
  EXPECT_NEAR(bspheres[handle3_shadow].radius, bsphere.radius, 1e-9);
  EXPECT_DOUBLE_EQ(0.0, bsphere.center.x());
  EXPECT_DOUBLE_EQ(0.0, bsphere.center.y());
  EXPECT_DOUBLE_EQ(0.0, bsphere.center.z());

  bsphere_for_contains_test = mask.getBoundingSphereForContainsTestNoLock();
  EXPECT_NEAR(bspheres[handle1].radius, bsphere_for_contains_test.radius, 1e-9);
  EXPECT_DOUBLE_EQ(bsphere.center.x(), bsphere_for_contains_test.center.x());
  EXPECT_DOUBLE_EQ(bsphere.center.y(), bsphere_for_contains_test.center.y());
  EXPECT_DOUBLE_EQ(bsphere.center.z(), bsphere_for_contains_test.center.z());

  mask.setIgnoreInContainsTest({multi_handle1, multi_handle3});
  mask.updateBodyPoses();

  bsphere = mask.getBoundingSphere();
  EXPECT_NEAR(bspheres[handle3_shadow].radius, bsphere.radius, 1e-9);
  EXPECT_DOUBLE_EQ(0.0, bsphere.center.x());
  EXPECT_DOUBLE_EQ(0.0, bsphere.center.y());
  EXPECT_DOUBLE_EQ(0.0, bsphere.center.z());

  bsphere_for_contains_test = mask.getBoundingSphereForContainsTestNoLock();
  EXPECT_DOUBLE_EQ(0.0, bsphere_for_contains_test.radius);
  EXPECT_DOUBLE_EQ(0.0, bsphere_for_contains_test.center.x());
  EXPECT_DOUBLE_EQ(0.0, bsphere_for_contains_test.center.y());
  EXPECT_DOUBLE_EQ(0.0, bsphere_for_contains_test.center.z());

  // test the case when all transforms are unavailable
  auto cb4 = [](ShapeHandle, Eigen::Isometry3d& t) -> bool {
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

  bsphere_for_contains_test = mask.getBoundingSphereForContainsTestNoLock();
  EXPECT_NEAR(bsphere.radius, bsphere_for_contains_test.radius, 1e-9);
  EXPECT_DOUBLE_EQ(bsphere.center.x(), bsphere_for_contains_test.center.x());
  EXPECT_DOUBLE_EQ(bsphere.center.y(), bsphere_for_contains_test.center.y());
  EXPECT_DOUBLE_EQ(bsphere.center.z(), bsphere_for_contains_test.center.z());
}

TEST(RayCastingShapeMask, UpdateBodyPoses) {
  rclcpp::Node nh("test_ray_casting_shape_mask");
  auto fooCb = [](ShapeHandle, Eigen::Isometry3d&) -> bool {
    return true;
  };
  TestMask mask(nh, fooCb, 1.0, 10.0, true, true, true);

  const shapes::ShapeConstPtr shape1(new shapes::Box(1.0, 2.0, 3.0));
  const auto multi_handle1 = mask.addShape(shape1, 1.0, 0.0, false, "box");
  const auto handle1 = multi_handle1.contains;
  const shapes::ShapeConstPtr shape2(new shapes::Sphere(2.0));
  const auto multi_handle2 = mask.addShape(shape2, 2.0, 0.5, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, true, "doubleSphere");
  const auto handle2_contains = multi_handle2.contains;
  const auto handle2_shadow = multi_handle2.shadow;

  size_t num_called = 0;
  const Eigen::Isometry3d t1 = randomPose();
  const Eigen::Isometry3d t2 = randomPose();
  auto cb = [&](const ShapeHandle h, Eigen::Isometry3d& t) -> bool {
    if (h == handle1) {
      t = t1;
    } else if (h == handle2_contains || h == handle2_shadow) {
      t = t2;
    } else {
      ADD_FAILURE();
    }

    num_called++;
    return true;
  };
  mask.setTransformCallback(cb);

  EXPECT_EQ(0, num_called);
  mask.updateBodyPoses();
  EXPECT_EQ(2, num_called);
  expectTransformsDoubleEq(t1, mask.getBodies()[handle1]->getPose());
  expectTransformsDoubleEq(t2, mask.getBodies()[handle2_contains]->getPose());
  expectTransformsDoubleEq(t2, mask.getBodies()[handle2_shadow]->getPose());

  const Eigen::Isometry3d t3 = randomPose();
  const Eigen::Isometry3d t4 = randomPose();
  auto cb2 = [&](const ShapeHandle h, Eigen::Isometry3d& t) -> bool {
    if (h == handle1) {
      t = t3;
    } else if (h == handle2_contains || h == handle2_shadow) {
      t = t4;
    } else {
      ADD_FAILURE();
    }

    num_called++;
    return true;
  };
  num_called = 0;
  mask.setTransformCallback(cb2);

  EXPECT_EQ(0, num_called);
  mask.updateBodyPoses();
  EXPECT_EQ(2, num_called);
  expectTransformsDoubleEq(t3, mask.getBodies()[handle1]->getPose());
  expectTransformsDoubleEq(t4, mask.getBodies()[handle2_contains]->getPose());
  expectTransformsDoubleEq(t4, mask.getBodies()[handle2_shadow]->getPose());
}

TEST(RayCastingShapeMask, ClassifyPoint) {
  rclcpp::Node nh("test_ray_casting_shape_mask");
  auto fooCb = [](ShapeHandle, Eigen::Isometry3d&) -> bool {
    return true;
  };
  TestMask mask(nh, fooCb, 0.1, 10.0, false, false, false);

  shapes::ShapeConstPtr shape1(new shapes::Box(1.8, 1.8, 1.8));
  const auto multi_handle1 = mask.addShape(shape1, 1.0, 0.02, 1.1, 0.01, 1.0, 0.02, 1.0, 0.02, false, "box");
  const auto handle1 = multi_handle1.contains;
  shapes::ShapeConstPtr shape2(new shapes::Sphere(1.375));
  const auto multi_handle2 = mask.addShape(shape2, 1.0, 0.0, false, "sphere");
  const auto handle2 = multi_handle2.contains;
  shapes::ShapeConstPtr shapeSensor(new shapes::Box(1.0, 1.0, 1.0));
  const auto multi_handleSensor = mask.addShape(shapeSensor, 0.1, 0.0, true, "sensor");
  const auto handle_sensor = multi_handleSensor.contains;

  const Eigen::Vector3d sensor_pos(-1.5, 0.0, 0.0);

  auto cb = [&](const ShapeHandle h, Eigen::Isometry3d& t) -> bool {
    t = Eigen::Isometry3d::Identity();
    if (h == handle_sensor) {
      t.translate(sensor_pos);
    }
    return true;
  };
  mask.setTransformCallback(cb);

  mask.updateBodyPoses();

  RayCastingShapeMask::MaskValue val;

  // This is a test set of points.
  // For an overview, open test_ray_casting_shape_mask.blend in Blender 2.80+.
  const Eigen::Vector3d point_sensor(sensor_pos);
  const Eigen::Vector3d point_sensor2(-1.47, 0, 0);
  const Eigen::Vector3d point_clip_min(-1.42, 0, 0);
  const Eigen::Vector3d point_clip_max(10, 0, 0);
  const Eigen::Vector3d point_in_box(0.85, 0.85, 0.85);
  const Eigen::Vector3d point_in_sphere(1.35, 0, 0);
  const Eigen::Vector3d point_in_both(0, 0, 0);
  const Eigen::Vector3d point_shadow_box(-0.25, -2, 2);
  const Eigen::Vector3d point_shadow_sphere(-0.560762, 0, 1.83871);
  const Eigen::Vector3d point_shadow_both(-sensor_pos);
  const Eigen::Vector3d point_outside(-3, 0, 0);
  const Eigen::Vector3d point_one_nan(-3, 0, std::numeric_limits<double>::quiet_NaN());
  const Eigen::Vector3d point_all_nan(std::numeric_limits<double>::quiet_NaN(),
                                      std::numeric_limits<double>::quiet_NaN(),
                                      std::numeric_limits<double>::quiet_NaN());

  // do_clipping_, do_contains_test_ and do_shadow_test_ are all false, so only OUTSIDE is possible
  mask.classifyPointNoLock(point_sensor, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(point_sensor2, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(point_clip_min, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(point_clip_max, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(point_in_box, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(point_in_sphere, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(point_in_both, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(point_shadow_box, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(point_shadow_sphere, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(point_shadow_both, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(point_outside, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(point_one_nan, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(point_all_nan, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);

  mask.do_clipping_ = true;
  mask.do_contains_test_ = false;
  mask.do_shadow_test_ = false;
  mask.classifyPointNoLock(point_sensor, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::CLIP, val);
  mask.classifyPointNoLock(point_sensor2, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::CLIP, val);
  mask.classifyPointNoLock(point_clip_min, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::CLIP, val);
  mask.classifyPointNoLock(point_clip_max, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::CLIP, val);
  mask.classifyPointNoLock(point_in_box, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(point_in_sphere, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(point_in_both, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(point_shadow_box, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(point_shadow_sphere, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(point_shadow_both, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(point_outside, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(point_one_nan, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(point_all_nan, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);

  mask.do_clipping_ = false;
  mask.do_contains_test_ = true;
  mask.do_shadow_test_ = false;
  mask.classifyPointNoLock(point_sensor, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::INSIDE, val);
  mask.classifyPointNoLock(point_sensor2, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::INSIDE, val);
  mask.classifyPointNoLock(point_clip_min, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(point_clip_max, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(point_in_box, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::INSIDE, val);
  mask.classifyPointNoLock(point_in_sphere, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::INSIDE, val);
  mask.classifyPointNoLock(point_in_both, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::INSIDE, val);
  mask.classifyPointNoLock(point_shadow_box, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(point_shadow_sphere, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(point_shadow_both, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(point_outside, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(point_one_nan, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(point_all_nan, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);

  // shadow filtering with sensor body not excluded - everything is shadowed by the sensor body
  mask.do_clipping_ = false;
  mask.do_contains_test_ = false;
  mask.do_shadow_test_ = true;
  mask.classifyPointNoLock(point_sensor, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);  // no ray, so no shadow
  mask.classifyPointNoLock(point_sensor2, val, sensor_pos);
  // the sensor is not considered to shadow points inside itself
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(point_clip_min, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, val);
  mask.classifyPointNoLock(point_clip_max, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, val);
  mask.classifyPointNoLock(point_in_box, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, val);
  mask.classifyPointNoLock(point_in_sphere, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, val);
  mask.classifyPointNoLock(point_in_both, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, val);
  mask.classifyPointNoLock(point_shadow_box, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, val);
  mask.classifyPointNoLock(point_shadow_sphere, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, val);
  mask.classifyPointNoLock(point_shadow_both, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, val);
  mask.classifyPointNoLock(point_outside, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, val);
  mask.classifyPointNoLock(point_one_nan, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(point_all_nan, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);

  mask.setIgnoreInShadowTest({multi_handleSensor});
  mask.do_clipping_ = false;
  mask.do_contains_test_ = false;
  mask.do_shadow_test_ = true;
  mask.classifyPointNoLock(point_sensor, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(point_sensor2, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(point_clip_min, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(point_clip_max, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, val);
  mask.classifyPointNoLock(point_in_box, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, val);
  mask.classifyPointNoLock(point_in_sphere, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, val);
  mask.classifyPointNoLock(point_in_both, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, val);
  mask.classifyPointNoLock(point_shadow_box, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, val);
  mask.classifyPointNoLock(point_shadow_sphere, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, val);
  mask.classifyPointNoLock(point_shadow_both, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, val);
  mask.classifyPointNoLock(point_outside, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(point_one_nan, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(point_all_nan, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);

  mask.do_clipping_ = true;
  mask.do_contains_test_ = true;
  mask.do_shadow_test_ = false;
  mask.classifyPointNoLock(point_sensor, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::CLIP, val);
  mask.classifyPointNoLock(point_sensor2, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::CLIP, val);
  mask.classifyPointNoLock(point_clip_min, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::CLIP, val);
  mask.classifyPointNoLock(point_clip_max, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::CLIP, val);
  mask.classifyPointNoLock(point_in_box, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::INSIDE, val);
  mask.classifyPointNoLock(point_in_sphere, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::INSIDE, val);
  mask.classifyPointNoLock(point_in_both, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::INSIDE, val);
  mask.classifyPointNoLock(point_shadow_box, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(point_shadow_sphere, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(point_shadow_both, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(point_outside, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(point_one_nan, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(point_all_nan, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);

  mask.do_clipping_ = true;
  mask.do_contains_test_ = false;
  mask.do_shadow_test_ = true;
  mask.classifyPointNoLock(point_sensor, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::CLIP, val);
  mask.classifyPointNoLock(point_sensor2, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::CLIP, val);
  mask.classifyPointNoLock(point_clip_min, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::CLIP, val);
  mask.classifyPointNoLock(point_clip_max, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::CLIP, val);
  mask.classifyPointNoLock(point_in_box, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, val);
  mask.classifyPointNoLock(point_in_sphere, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, val);
  mask.classifyPointNoLock(point_in_both, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, val);
  mask.classifyPointNoLock(point_shadow_box, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, val);
  mask.classifyPointNoLock(point_shadow_sphere, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, val);
  mask.classifyPointNoLock(point_shadow_both, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, val);
  mask.classifyPointNoLock(point_outside, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(point_one_nan, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(point_all_nan, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);

  mask.do_clipping_ = false;
  mask.do_contains_test_ = true;
  mask.do_shadow_test_ = true;
  mask.classifyPointNoLock(point_sensor, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::INSIDE, val);
  mask.classifyPointNoLock(point_sensor2, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::INSIDE, val);
  mask.classifyPointNoLock(point_clip_min, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(point_clip_max, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, val);
  mask.classifyPointNoLock(point_in_box, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::INSIDE, val);
  mask.classifyPointNoLock(point_in_sphere, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::INSIDE, val);
  mask.classifyPointNoLock(point_in_both, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::INSIDE, val);
  mask.classifyPointNoLock(point_shadow_box, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, val);
  mask.classifyPointNoLock(point_shadow_sphere, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, val);
  mask.classifyPointNoLock(point_shadow_both, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, val);
  mask.classifyPointNoLock(point_outside, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(point_one_nan, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(point_all_nan, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);

  mask.do_clipping_ = true;
  mask.do_contains_test_ = true;
  mask.do_shadow_test_ = true;
  mask.classifyPointNoLock(point_sensor, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::CLIP, val);
  mask.classifyPointNoLock(point_sensor2, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::CLIP, val);
  mask.classifyPointNoLock(point_clip_min, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::CLIP, val);
  mask.classifyPointNoLock(point_clip_max, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::CLIP, val);
  mask.classifyPointNoLock(point_in_box, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::INSIDE, val);
  mask.classifyPointNoLock(point_in_sphere, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::INSIDE, val);
  mask.classifyPointNoLock(point_in_both, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::INSIDE, val);
  mask.classifyPointNoLock(point_shadow_box, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, val);
  mask.classifyPointNoLock(point_shadow_sphere, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, val);
  mask.classifyPointNoLock(point_shadow_both, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, val);
  mask.classifyPointNoLock(point_outside, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(point_one_nan, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.classifyPointNoLock(point_all_nan, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);

  random_numbers::RandomNumberGenerator rng;
  Eigen::Vector3d point_inside;
  for (size_t i = 0; i < 100; ++i) {
    if (mask.getBodies().at(handle1)->samplePointInside(rng, 10, point_inside)) {
      mask.classifyPointNoLock(point_inside, val, sensor_pos);
      EXPECT_EQ(RayCastingShapeMask::MaskValue::INSIDE, val);
    }
    if (mask.getBodies().at(handle2)->samplePointInside(rng, 10, point_inside)) {
      mask.classifyPointNoLock(point_inside, val, sensor_pos);
      EXPECT_EQ(RayCastingShapeMask::MaskValue::INSIDE, val);
    }
  }
}

TEST(RayCastingShapeMask, Mask) {
  rclcpp::Node nh("test_ray_casting_shape_mask");
  auto fooCb = [](ShapeHandle, Eigen::Isometry3d&) -> bool {
    return true;
  };
  TestMask mask(nh, fooCb, 0.1, 10.0, true, true, true);

  shapes::ShapeConstPtr shape1(new shapes::Box(1.8, 1.8, 1.8));
  const auto multi_handle1 = mask.addShape(shape1, 1.0, 0.02, 1.1, 0.01, 1.0, 0.02, 1.0, 0.02, false, "box");
  const auto handle1 = multi_handle1.contains;
  EXPECT_NE(handle1, 0);
  shapes::ShapeConstPtr shape2(new shapes::Sphere(1.375));
  const auto multi_handle2 = mask.addShape(shape2, 1.0, 0.0, false, "sphere");
  const auto handle2 = multi_handle2.contains;
  EXPECT_NE(handle2, 0);
  shapes::ShapeConstPtr shape_sensor(new shapes::Box(1.0, 1.0, 1.0));
  const auto multi_handleSensor = mask.addShape(shape_sensor, 0.1, 0.0, true, "sensor");
  const auto handle_sensor = multi_handleSensor.contains;

  const Eigen::Vector3d sensor_pos(-1.5, 0.0, 0.0);

  auto cb = [&](ShapeHandle h, Eigen::Isometry3d& t) -> bool {
    t = Eigen::Isometry3d::Identity();
    if (h == handle_sensor) {
      t.translate(sensor_pos);
    }
    return true;
  };
  mask.setTransformCallback(cb);
  mask.setIgnoreInShadowTest({multi_handleSensor});

  RayCastingShapeMask::MaskValue val;

  // This is a test set of points.
  // For an overview, open test_ray_casting_shape_mask.blend in Blender 2.80+.
  const Eigen::Vector3f point_sensor(sensor_pos.x(), sensor_pos.y(), sensor_pos.z());
  const Eigen::Vector3f point_sensor2(-1.47, 0, 0);
  const Eigen::Vector3f point_clip_min(-1.42, 0, 0);
  const Eigen::Vector3f point_clip_max(10, 0, 0);
  const Eigen::Vector3f point_in_box(0.85, 0.85, 0.85);
  const Eigen::Vector3f point_in_sphere(1.35, 0, 0);
  const Eigen::Vector3f point_in_both(0, 0, 0);
  const Eigen::Vector3f point_shadow_box(-0.25, -2, 2);
  const Eigen::Vector3f point_shadow_sphere(-0.560762, 0, 1.83871);
  const Eigen::Vector3f point_shadow_both(-sensor_pos.x(), 0, 0);
  const Eigen::Vector3f point_outside(-3, 0, 0);
  const Eigen::Vector3f point_one_nan(-3, 0, std::numeric_limits<float>::quiet_NaN());
  const Eigen::Vector3f point_all_nan(std::numeric_limits<float>::quiet_NaN(),
                                      std::numeric_limits<float>::quiet_NaN(),
                                      std::numeric_limits<float>::quiet_NaN());

  mask.maskContainmentAndShadows(point_sensor, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::CLIP, val);
  mask.maskContainmentAndShadows(point_sensor2, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::CLIP, val);
  mask.maskContainmentAndShadows(point_clip_min, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::CLIP, val);
  mask.maskContainmentAndShadows(point_clip_max, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::CLIP, val);
  mask.maskContainmentAndShadows(point_in_box, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::INSIDE, val);
  mask.maskContainmentAndShadows(point_in_sphere, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::INSIDE, val);
  mask.maskContainmentAndShadows(point_in_both, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::INSIDE, val);
  mask.maskContainmentAndShadows(point_shadow_box, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, val);
  mask.maskContainmentAndShadows(point_shadow_sphere, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, val);
  mask.maskContainmentAndShadows(point_shadow_both, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, val);
  mask.maskContainmentAndShadows(point_outside, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.maskContainmentAndShadows(point_one_nan, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);
  mask.maskContainmentAndShadows(point_all_nan, val, sensor_pos);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, val);

  cras::Cloud cloud;
  cras::CloudModifier mod(cloud);
  mod.setPointCloud2FieldsByString(1, "xyz");
  mod.resize(13);
  cras::CloudIter it_x(cloud, "x");
  cras::CloudIter it_y(cloud, "y");
  cras::CloudIter it_z(cloud, "z");

  Eigen::Vector3f p;
  p = point_sensor; *it_x = p.x(); *it_y = p.y(); *it_z = p.z(); ++it_x; ++it_y; ++it_z;
  p = point_sensor2; *it_x = p.x(); *it_y = p.y(); *it_z = p.z(); ++it_x; ++it_y; ++it_z;
  p = point_clip_min; *it_x = p.x(); *it_y = p.y(); *it_z = p.z(); ++it_x; ++it_y; ++it_z;
  p = point_clip_max; *it_x = p.x(); *it_y = p.y(); *it_z = p.z(); ++it_x; ++it_y; ++it_z;
  p = point_in_box; *it_x = p.x(); *it_y = p.y(); *it_z = p.z(); ++it_x; ++it_y; ++it_z;
  p = point_in_sphere; *it_x = p.x(); *it_y = p.y(); *it_z = p.z(); ++it_x; ++it_y; ++it_z;
  p = point_in_both; *it_x = p.x(); *it_y = p.y(); *it_z = p.z(); ++it_x; ++it_y; ++it_z;
  p = point_shadow_box; *it_x = p.x(); *it_y = p.y(); *it_z = p.z(); ++it_x; ++it_y; ++it_z;
  p = point_shadow_sphere; *it_x = p.x(); *it_y = p.y(); *it_z = p.z(); ++it_x; ++it_y; ++it_z;
  p = point_shadow_both; *it_x = p.x(); *it_y = p.y(); *it_z = p.z(); ++it_x; ++it_y; ++it_z;
  p = point_outside; *it_x = p.x(); *it_y = p.y(); *it_z = p.z(); ++it_x; ++it_y; ++it_z;
  p = point_one_nan; *it_x = p.x(); *it_y = p.y(); *it_z = p.z(); ++it_x; ++it_y; ++it_z;
  p = point_all_nan; *it_x = p.x(); *it_y = p.y(); *it_z = p.z(); ++it_x; ++it_y; ++it_z;

  std::vector<RayCastingShapeMask::MaskValue> vals;
  vals.push_back(RayCastingShapeMask::MaskValue::INSIDE);  // be adverse and pass garbage
  mask.maskContainmentAndShadows(cloud, vals, sensor_pos);

  ASSERT_EQ(13, vals.size());
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
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, vals[11]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, vals[12]);
}

TEST(RayCastingShapeMask, MaskPerformancePoints) {
  rclcpp::Node nh("test_ray_casting_shape_mask");
  auto fooCb = [](ShapeHandle, Eigen::Isometry3d&) -> bool {
    return true;
  };
  TestMask mask(nh, fooCb, 0.1, 10.0, true, true, true);

  shapes::ShapeConstPtr shape1(new shapes::Box(2.0, 2.0, 2.0));
  const auto multi_handle1 = mask.addShape(shape1, 1.0, 0.0, false, "box");
  const auto handle1 = multi_handle1.contains;
  shapes::ShapeConstPtr shape2(new shapes::Sphere(1.375));
  const auto multi_handle2 = mask.addShape(shape2, 1.0, 0.0, false, "sphere");
  const auto handle2 = multi_handle2.contains;
  shapes::ShapeConstPtr shape_sensor(new shapes::Box(1.0, 1.0, 1.0));
  const auto multi_handleSensor = mask.addShape(shape_sensor, 0.1, 0.0, true, "sensor");
  const auto handle_sensor = multi_handleSensor.contains;

  const Eigen::Vector3d sensor_pos(-1.5, 0.0, 0.0);

  auto cb = [&](const ShapeHandle h, Eigen::Isometry3d& t) -> bool {
    t = Eigen::Isometry3d::Identity();
    if (h == handle_sensor) {
      t.translate(sensor_pos);
    }
    return true;
  };
  mask.setTransformCallback(cb);
  mask.setIgnoreInShadowTest({multi_handleSensor});

#if RELEASE_BUILD == 1
  constexpr size_t num_points = 10000000;
#else
  constexpr size_t num_points = 100000;
#endif

  cras::Cloud cloud;
  cras::CloudModifier mod(cloud);
  mod.setPointCloud2FieldsByString(1, "xyz");
  mod.resize(num_points);
  cras::CloudIter it_x(cloud, "x");
  cras::CloudIter it_y(cloud, "y");
  cras::CloudIter it_z(cloud, "z");

  // generate a bunch of points close the the bodies so that all filtering parts get activated
  random_numbers::RandomNumberGenerator rng;
  Eigen::Vector3d p;
  auto body1 = mask.getBodies()[handle1];
  auto body2 = mask.getBodies()[handle2];
  for (size_t i = 0; i < num_points / 2; ++i) {
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

  rclcpp::Clock wall_clock(RCL_SYSTEM_TIME);
  rclcpp::Time start = wall_clock.now();
  mask.maskContainmentAndShadows(cloud, vals, sensor_pos);
  rclcpp::Time end = wall_clock.now();

  ASSERT_EQ(num_points, vals.size());
#if RELEASE_BUILD == 1
  EXPECT_GT(2.0, (end - start).seconds());
#else
  EXPECT_GT(1.5, (end - start).seconds());
#endif
}

TEST(RayCastingShapeMask, MaskPerformanceBodies) {
  rclcpp::Node nh("test_ray_casting_shape_mask");
  auto cb = [](ShapeHandle, Eigen::Isometry3d& t) -> bool {
    t = randomPose();
    t.translation().normalize();
    t.translation() *= 10.0;  // so that we don't get too far behind the clipping plane
    return true;
  };
  TestMask mask(nh, cb, 0.1, 10.0, true, true, true);

#if RELEASE_BUILD == 1
  constexpr size_t numBodies = 1000000;
#else
  constexpr size_t numBodies = 20000;
#endif

  for (size_t i = 0; i < numBodies / 2; ++i) {
    shapes::ShapeConstPtr shape1(new shapes::Box(2.0, 2.0, 2.0));
    mask.addShape(shape1, 1.0, 0.0, false, "box");
    shapes::ShapeConstPtr shape2(new shapes::Sphere(1.375));
    mask.addShape(shape2, 1.0, 0.0, false, "sphere");
  }
  mask.updateInternalShapeLists();

  const Eigen::Vector3d sensor_pos(0.0, 0.0, 0.0);

  constexpr size_t num_points = 10;

  cras::Cloud cloud;
  cras::CloudModifier mod(cloud);
  mod.setPointCloud2FieldsByString(1, "xyz");
  mod.resize(num_points);
  cras::CloudIter it_x(cloud, "x");
  cras::CloudIter it_y(cloud, "y");
  cras::CloudIter it_z(cloud, "z");

  // generate a bunch of points close the the bodies so that all filtering parts get activated
  Eigen::Vector3d p;
  for (size_t i = 0; i < num_points; ++i) {
    p = Eigen::Vector3d::Random();
    p.normalize();
    p *= 10.0;
    *it_x = p.x(); *it_y = p.y(); *it_z = p.z();
    ++it_x; ++it_y; ++it_z;
  }

  std::vector<RayCastingShapeMask::MaskValue> vals;

  rclcpp::Clock wall_clock(RCL_SYSTEM_TIME);
  rclcpp::Time start = wall_clock.now();
  mask.maskContainmentAndShadows(cloud, vals, sensor_pos);
  rclcpp::Time end = wall_clock.now();

  ASSERT_EQ(num_points, vals.size());
#if RELEASE_BUILD == 1
  EXPECT_GT(1.0, (end - start).seconds());
#else
  EXPECT_GT(2.0, (end - start).seconds());
#endif
}

TEST(RayCastingShapeMask, MaskPerformanceBodiesMesh) {
  rclcpp::Node nh("test_ray_casting_shape_mask");
  auto cb = [](ShapeHandle, Eigen::Isometry3d& t) -> bool {
    t = randomPose();
    t.translation().normalize();
    t.translation() *= 10.0;  // so that we don't get too far behind the clipping plane
    return true;
  };
  TestMask mask(nh, cb, 0.1, 10.0, true, true, true);

#if RELEASE_BUILD == 1
  constexpr size_t num_bodies = 10000;
#else
  constexpr size_t num_bodies = 5000;
#endif

  auto g = urdf::Mesh();
  g.scale = {1.0, 2.0, 3.0};
  g.filename = std::string("file://") + TEST_DATA_DIR + "/box.dae";
  for (size_t i = 0; i < num_bodies; ++i) {
    const auto shape = constructShape(g);
    mask.addShape(shape, 1.0, 0.0, false, "mesh");
  }
  mask.updateInternalShapeLists();

  const Eigen::Vector3d sensor_pos(0.0, 0.0, 0.0);

#if RELEASE_BUILD == 1
  constexpr size_t num_points = 10000;
#else
  constexpr size_t num_points = 10;
#endif

  cras::Cloud cloud;
  cras::CloudModifier mod(cloud);
  mod.setPointCloud2FieldsByString(1, "xyz");
  mod.resize(num_points);
  cras::CloudIter it_x(cloud, "x");
  cras::CloudIter it_y(cloud, "y");
  cras::CloudIter it_z(cloud, "z");

  // generate a bunch of points close the the bodies so that all filtering parts get activated
  Eigen::Vector3d p;
  for (size_t i = 0; i < num_points; ++i) {
    p = Eigen::Vector3d::Random();
    p.normalize();
    p *= 10.0;
    *it_x = p.x(); *it_y = p.y(); *it_z = p.z();
    ++it_x; ++it_y; ++it_z;
  }

  std::vector<RayCastingShapeMask::MaskValue> vals;

  rclcpp::Clock wall_clock(RCL_SYSTEM_TIME);
  rclcpp::Time start = wall_clock.now();
  mask.maskContainmentAndShadows(cloud, vals, sensor_pos);
  rclcpp::Time end = wall_clock.now();

  ASSERT_EQ(num_points, vals.size());
#if RELEASE_BUILD == 1
  EXPECT_GT(0.1, (end - start).seconds());
#else
  EXPECT_GT(1.0, (end - start).seconds());
#endif
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
