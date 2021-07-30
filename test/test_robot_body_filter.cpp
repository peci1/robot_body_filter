#include "gtest/gtest.h"

#include <robot_body_filter/RobotBodyFilter.h>
#include <xmlrpcpp/XmlRpcValue.h>
#include "utils.cpp"

using namespace robot_body_filter;

// This model corresponds to what is in test_ray_casting_shape_mask.blend.
// However, the sizes of the objects are smaller so that they reach the desired
// size after applying scale 1.1 and padding 0.01 (as set in test_robot_body_filter.yaml).
const std::string ROBOT_URDF =
    "<?xml version=\"1.0\" ?>\n"
    "<robot name=\"NIFTi\">\n"
    "    <link name=\"base_link\">\n"
    "        <visual>\n"
    "            <origin rpy=\"0 0 0\" xyz=\"-0.1220 0 0\"/>\n"
    "            <geometry><box size=\"1.8 1.8 1.8\"/></geometry>\n"
    "        </visual>\n"
    "        <collision>\n"
    "            <origin rpy=\"0 0 0\" xyz=\"-0.1220 0 0\"/>\n"
    "            <geometry><box size=\"1.8 1.8 1.8\"/></geometry>\n"
    "        </collision>\n"
    "        <collision name=\"big_collision_box\">\n"
    "            <origin rpy=\"0 0 0\" xyz=\"-0.1 0 0\"/>\n"
    "            <geometry><box size=\"2.2545 2.2545 2.2545\"/></geometry>\n"
    "        </collision>\n"
    "        <inertial>\n"
    "            <origin rpy=\"0 0 0\" xyz=\"-0.034 0 0.142\"/>\n"
    "            <mass value=\"6.0\"/>\n"
    "            <inertia ixx=\"0.001\" ixy=\"0.0001\" ixz=\"0.0\" iyy=\"0.02\" iyz=\"-0.0001\" izz=\"0.03\"/>\n"
    "        </inertial>\n"
    "    </link>\n"
    "    <link name=\"antenna\">\n"
    "        <visual>\n"
    "            <origin rpy=\"0 0 0\" xyz=\"-0.01864 0 0\"/>\n"
    "            <geometry><sphere radius=\"1.24091\" /></geometry>\n"
    "        </visual>\n"
    "        <collision>\n"
    "            <origin rpy=\"0 0 0\" xyz=\"-0.01864 0 0\"/>\n"
    "            <geometry><sphere radius=\"1.24091\"/></geometry>\n"
    "        </collision>\n"
    "        <inertial>\n"
    "            <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n"
    "            <mass value=\"0.5\"/>\n"
    "            <inertia ixx=\"0.005\" ixy=\"0\" ixz=\"0\" iyy=\"0.001\" iyz=\"-0.0001\" izz=\"0.004\"/>\n"
    "        </inertial>\n"
    "    </link>\n"
    "    <joint name=\"antenna_j\" type=\"fixed\">\n"
    "        <parent link=\"base_link\"/>\n"
    "        <child link=\"antenna\"/>\n"
    "        <origin rpy=\"0 0 0\" xyz=\"0.01864 0 0\"/>\n"
    "    </joint>\n"
    "    <link name=\"laser_base\">\n"
    "        <visual>\n"
    "            <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n"
    "            <geometry><box size=\"0.0005 0.0005 0.0005\"/></geometry>\n"
    "        </visual>\n"
    "        <inertial>\n"
    "            <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n"
    "            <mass value=\"1e-5\"/>\n"
    "            <inertia ixx=\"1e-3\" ixy=\"1e-6\" ixz=\"1e-6\" iyy=\"1e-3\" iyz=\"1e-6\" izz=\"1e-3\"/>\n"
    "        </inertial>\n"
    "    </link>\n"
    "    <joint name=\"laser_base_j\" type=\"fixed\">\n"
    "        <parent link=\"base_link\"/>\n"
    "        <child link=\"laser_base\"/>\n"
    "        <origin rpy=\"0 0 0\" xyz=\"-1.5 0 0\"/>\n"
    "    </joint>\n"
    "    <link name=\"laser\">\n"
    "        <visual>\n"
    "            <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n"
    "            <geometry><box size=\"0.07273 0.07273 0.07273\"/></geometry>\n"
    "        </visual>\n"
    "        <collision>\n"
    "            <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n"
    "            <geometry><box size=\"0.07273 0.07273 0.07273\"/></geometry>\n"
    "        </collision>\n"
    "        <inertial>\n"
    "            <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n"
    "            <mass value=\"1\"/>\n"
    "            <inertia ixx=\"0.003\" ixy=\"0\" ixz=\"0\" iyy=\"0.003\" iyz=\"0\" izz=\"0.002\"/>\n"
    "        </inertial>\n"
    "    </link>\n"
    "    <joint name=\"laser_j\" type=\"revolute\">\n"
    "        <parent link=\"laser_base\"/>\n"
    "        <child link=\"laser\"/>\n"
    "        <axis xyz=\"1 0 0\"/>\n"
    "        <origin rpy=\"0 0 0\" xyz=\"0 0 0\"/>\n"
    "        <limit effort=\"0\" lower=\"-2.3561945\" upper=\"2.3561945\" velocity=\"4\"/>\n"
    "    </joint>\n"
    "</robot>";

class RobotBodyFilterLaserScanTest : public RobotBodyFilterLaserScan
{
  public: RobotBodyFilterLaserScanTest()
  {
    this->failWithoutRobotDescription = true;
  }

  virtual ~RobotBodyFilterLaserScanTest()
  {
    // this prevents spurious SIGABRTs caused probably by some too fast cleanup
    // after tfFramesWatchdog, or I don't know what...
    ros::WallDuration(0.1).sleep();
  };

  friend class RobotBodyFilter_InitFromArray_Test;
  friend class RobotBodyFilter_InitFromDict_Test;
  friend class RobotBodyFilter_LoadParams_Test;
  friend class RobotBodyFilter_LoadParamsAllConfig_Test;
  friend class RobotBodyFilter_ParseRobot_Test;
  friend class RobotBodyFilter_Transforms_Test;
  friend class RobotBodyFilter_ComputeMaskPointByPoint_Test;
  friend class RobotBodyFilter_UpdateLaserScan_Test;
};

class RobotBodyFilterPointCloud2Test : public RobotBodyFilterPointCloud2
{
  public: RobotBodyFilterPointCloud2Test()
  {
    this->failWithoutRobotDescription = true;
  }

  virtual ~RobotBodyFilterPointCloud2Test()
  {
    // this prevents spurious SIGABRTs caused probably by some too fast cleanup
    // after tfFramesWatchdog, or I don't know what...
    ros::WallDuration(0.1).sleep();
  };

  friend class RobotBodyFilter_ComputeMaskAllAtOnce_Test;
  friend class RobotBodyFilter_UpdatePointCloud2_Test;
};

TEST(RobotBodyFilter, InitFromArray)
{
  ros::NodeHandle nh;

  auto filter = std::make_shared<RobotBodyFilterLaserScanTest>();
  auto filterBase = std::dynamic_pointer_cast<filters::FilterBase<sensor_msgs::LaserScan>>(filter);

  XmlRpc::XmlRpcValue value;
  nh.getParam("test_chain_config", value);
  ASSERT_EQ(XmlRpc::XmlRpcValue::TypeArray, value.getType());
  ASSERT_EQ(1, value.size());

  // test_robot_description parameter doesn't exist
  nh.deleteParam("test_robot_description");
  EXPECT_THROW(filterBase->configure(value[0]), std::runtime_error);
  EXPECT_FALSE(filter->configured_);
  ASSERT_NE(nullptr, filter->tfFramesWatchdog);

  // test that invalid robot model doesn't throw any exception, but also generates no filter shapes
  nh.setParam("test_robot_description", "<robot name='test'></robot>");
  filterBase->configure(value[0]);
  EXPECT_EQ("test_robot_description", filter->robotDescriptionParam);
  EXPECT_EQ(0, filter->shapesToLinks.size());
  EXPECT_TRUE(filter->configured_);
  EXPECT_GT(ros::Duration(0.1), ros::Time::now() - filter->timeConfigured);
  ASSERT_NE(nullptr, filter->tfFramesWatchdog);
  EXPECT_TRUE(filter->tfFramesWatchdog->isMonitored("laser"));  // Issue #6, monitor sensor frame
}

TEST(RobotBodyFilter, InitFromDict)
{
  ros::NodeHandle nh;

  auto filter = std::make_shared<RobotBodyFilterLaserScanTest>();
  auto filterBase = std::dynamic_pointer_cast<filters::FilterBase<sensor_msgs::LaserScan>>(filter);

  // test_robot_description parameter doesn't exist
  nh.deleteParam("test_robot_description");
  EXPECT_THROW(filterBase->configure("test_dict_config", nh), std::runtime_error);
  EXPECT_FALSE(filter->configured_);
  ASSERT_NE(nullptr, filter->tfFramesWatchdog);

  // test that invalid robot model doesn't throw any exception, but also generates no filter shapes
  nh.setParam("test_robot_description", "<robot name='test'></robot>");
  filterBase->configure("test_dict_config", nh);
  EXPECT_EQ("test_robot_description", filter->robotDescriptionParam);
  EXPECT_EQ(0, filter->shapesToLinks.size());
  EXPECT_TRUE(filter->configured_);
  EXPECT_GT(ros::Duration(0.1), ros::Time::now() - filter->timeConfigured);
  ASSERT_NE(nullptr, filter->tfFramesWatchdog);
  EXPECT_TRUE(filter->tfFramesWatchdog->isMonitored("laser"));  // Issue #6, monitor sensor frame
}

TEST(RobotBodyFilter, LoadParams)
{
  ros::NodeHandle nh;

  auto filter = std::make_shared<RobotBodyFilterLaserScanTest>();
  auto filterBase = std::dynamic_pointer_cast<filters::FilterBase<sensor_msgs::LaserScan>>(filter);

  // test that invalid robot model doesn't throw any exception, but also generates no filter shapes
  nh.setParam("test_robot_description", "<robot name='test'></robot>");
  filterBase->configure("test_dict_config", nh);

  EXPECT_EQ("odom", filter->fixedFrame);
  EXPECT_EQ("laser", filter->sensorFrame);
  EXPECT_EQ("odom", filter->filteringFrame);
  EXPECT_TRUE(filter->keepCloudsOrganized);
  EXPECT_FLOAT_EQ(0.002, filter->modelPoseUpdateInterval.toSec());
  EXPECT_TRUE(filter->pointByPointScan);
  EXPECT_FLOAT_EQ(0.1, filter->minDistance);
  EXPECT_FLOAT_EQ(10.0, filter->maxDistance);
  EXPECT_EQ(std::set<std::string>({"antenna", "base_link::big_collision_box"}),
      filter->linksIgnoredInBoundingSphere);
  EXPECT_EQ(std::set<std::string>({"laser", "base_link::big_collision_box"}),
      filter->linksIgnoredInShadowTest);
  EXPECT_TRUE(filter->linksIgnoredInBoundingBox.empty());
  EXPECT_TRUE(filter->linksIgnoredInContainsTest.empty());
  EXPECT_TRUE(filter->linksIgnoredEverywhere.empty());
  EXPECT_TRUE(filter->onlyLinks.empty());
  EXPECT_DOUBLE_EQ(1.1, filter->defaultContainsInflation.scale);
  EXPECT_DOUBLE_EQ(0.01, filter->defaultContainsInflation.padding);
  EXPECT_DOUBLE_EQ(1.1, filter->defaultShadowInflation.scale);
  EXPECT_DOUBLE_EQ(0.01, filter->defaultShadowInflation.padding);
  EXPECT_EQ((std::map<std::string, ScaleAndPadding>({
    {"*::big_collision_box", ScaleAndPadding(2.0, 0.01)},
    {"base_link", ScaleAndPadding(1.1, 0.05)},
    {"antenna", ScaleAndPadding(1.2, 0.01)},
  })), filter->perLinkContainsInflation);
  EXPECT_EQ((std::map<std::string, ScaleAndPadding>({
    {"*::big_collision_box", ScaleAndPadding(3.0, 0.01)},
    {"base_link", ScaleAndPadding(1.1, 0.05)},
    {"laser", ScaleAndPadding(1.1, 0.015)},
  })), filter->perLinkShadowInflation);
  EXPECT_EQ("test_robot_description", filter->robotDescriptionParam);
  EXPECT_EQ("robot_model", filter->robotDescriptionUpdatesFieldName);
  EXPECT_DOUBLE_EQ(60.0, filter->tfBufferLength.toSec());
  EXPECT_DOUBLE_EQ(0.2, filter->reachableTransformTimeout.toSec());
  EXPECT_DOUBLE_EQ(0.2, filter->unreachableTransformTimeout.toSec());
  EXPECT_TRUE(filter->computeBoundingSphere);
  EXPECT_FALSE(filter->computeDebugBoundingSphere);
  EXPECT_FALSE(filter->publishBoundingSphereMarker);
  EXPECT_FALSE(filter->publishNoBoundingSpherePointcloud);
  EXPECT_FALSE(filter->computeBoundingBox);
  EXPECT_FALSE(filter->computeDebugBoundingBox);
  EXPECT_FALSE(filter->publishBoundingBoxMarker);
  EXPECT_FALSE(filter->publishNoBoundingBoxPointcloud);
  EXPECT_FALSE(filter->computeOrientedBoundingBox);
  EXPECT_FALSE(filter->computeDebugOrientedBoundingBox);
  EXPECT_FALSE(filter->publishOrientedBoundingBoxMarker);
  EXPECT_FALSE(filter->publishNoOrientedBoundingBoxPointcloud);
  EXPECT_TRUE(filter->computeLocalBoundingBox);
  EXPECT_FALSE(filter->computeDebugLocalBoundingBox);
  EXPECT_FALSE(filter->publishLocalBoundingBoxMarker);
  EXPECT_FALSE(filter->publishNoLocalBoundingBoxPointcloud);
  EXPECT_EQ("base_link", filter->localBoundingBoxFrame);
  EXPECT_FALSE(filter->publishDebugPclInside);
  EXPECT_FALSE(filter->publishDebugPclClip);
  EXPECT_FALSE(filter->publishDebugPclShadow);
  EXPECT_FALSE(filter->publishDebugContainsMarker);
  EXPECT_FALSE(filter->publishDebugShadowMarker);
  EXPECT_FALSE(filter->requireAllFramesReachable);

  EXPECT_EQ("/robot_bounding_sphere", filter->boundingSpherePublisher.getTopic());
  EXPECT_EQ("", filter->boundingBoxPublisher.getTopic());
  EXPECT_EQ("", filter->orientedBoundingBoxPublisher.getTopic());
  EXPECT_EQ("/robot_local_bounding_box", filter->localBoundingBoxPublisher.getTopic());
  EXPECT_EQ("", filter->boundingSphereMarkerPublisher.getTopic());
  EXPECT_EQ("", filter->boundingBoxMarkerPublisher.getTopic());
  EXPECT_EQ("", filter->orientedBoundingBoxMarkerPublisher.getTopic());
  EXPECT_EQ("", filter->localBoundingBoxMarkerPublisher.getTopic());
  EXPECT_EQ("", filter->boundingSphereDebugMarkerPublisher.getTopic());
  EXPECT_EQ("", filter->boundingBoxDebugMarkerPublisher.getTopic());
  EXPECT_EQ("", filter->orientedBoundingBoxDebugMarkerPublisher.getTopic());
  EXPECT_EQ("", filter->localBoundingBoxDebugMarkerPublisher.getTopic());
  EXPECT_EQ("", filter->scanPointCloudNoBoundingSpherePublisher.getTopic());
  EXPECT_EQ("", filter->scanPointCloudNoBoundingBoxPublisher.getTopic());
  EXPECT_EQ("", filter->scanPointCloudNoOrientedBoundingBoxPublisher.getTopic());
  EXPECT_EQ("", filter->scanPointCloudNoLocalBoundingBoxPublisher.getTopic());
  EXPECT_EQ("", filter->debugPointCloudInsidePublisher.getTopic());
  EXPECT_EQ("", filter->debugPointCloudClipPublisher.getTopic());
  EXPECT_EQ("", filter->debugPointCloudShadowPublisher.getTopic());
  EXPECT_EQ("", filter->debugContainsMarkerPublisher.getTopic());
  EXPECT_EQ("", filter->debugShadowMarkerPublisher.getTopic());

  EXPECT_EQ(ros::this_node::getName() + "/reload_model",
      filter->reloadRobotModelServiceServer.getService());
}

TEST(RobotBodyFilter, LoadParamsAllConfig)
{
  ros::NodeHandle nh;

  auto filter = std::make_shared<RobotBodyFilterLaserScanTest>();
  auto filterBase = std::dynamic_pointer_cast<filters::FilterBase<sensor_msgs::LaserScan>>(filter);

  // test that invalid robot model doesn't throw any exception, but also generates no filter shapes
  nh.setParam("test_robot_description", "<robot name='test'></robot>");
  filterBase->configure("all_config", nh);

  EXPECT_EQ("odom", filter->fixedFrame);
  EXPECT_EQ("laser", filter->sensorFrame);
  EXPECT_EQ("base_link", filter->filteringFrame);
  EXPECT_TRUE(filter->keepCloudsOrganized);
  EXPECT_FLOAT_EQ(0.002, filter->modelPoseUpdateInterval.toSec());
  EXPECT_TRUE(filter->pointByPointScan);
  EXPECT_FLOAT_EQ(0.1, filter->minDistance);
  EXPECT_FLOAT_EQ(10.0, filter->maxDistance);
  EXPECT_EQ(std::set<std::string>({"antenna", "base_link::big_collision_box"}),
      filter->linksIgnoredInBoundingSphere);
  EXPECT_EQ(std::set<std::string>({"laser", "base_link::big_collision_box"}),
      filter->linksIgnoredInShadowTest);
  EXPECT_EQ(std::set<std::string>({"base_link"}), filter->linksIgnoredInBoundingBox);
  EXPECT_EQ(std::set<std::string>({"base_link"}), filter->linksIgnoredInContainsTest);
  EXPECT_EQ(std::set<std::string>({"base_link"}), filter->linksIgnoredEverywhere);
  EXPECT_EQ(std::set<std::string>({"laser"}), filter->onlyLinks);
  EXPECT_DOUBLE_EQ(1.1, filter->defaultContainsInflation.scale);
  EXPECT_DOUBLE_EQ(0.01, filter->defaultContainsInflation.padding);
  EXPECT_DOUBLE_EQ(1.1, filter->defaultShadowInflation.scale);
  EXPECT_DOUBLE_EQ(0.01, filter->defaultShadowInflation.padding);
  EXPECT_EQ("test_robot_description", filter->robotDescriptionParam);
  EXPECT_EQ("robot", filter->robotDescriptionUpdatesFieldName);
  EXPECT_DOUBLE_EQ(60.0, filter->tfBufferLength.toSec());
  EXPECT_DOUBLE_EQ(0.2, filter->reachableTransformTimeout.toSec());
  EXPECT_DOUBLE_EQ(0.2, filter->unreachableTransformTimeout.toSec());
  EXPECT_TRUE(filter->computeBoundingSphere);
  EXPECT_TRUE(filter->computeDebugBoundingSphere);
  EXPECT_TRUE(filter->publishBoundingSphereMarker);
  EXPECT_TRUE(filter->publishNoBoundingSpherePointcloud);
  EXPECT_TRUE(filter->computeBoundingBox);
  EXPECT_TRUE(filter->computeDebugBoundingBox);
  EXPECT_TRUE(filter->publishBoundingBoxMarker);
  EXPECT_TRUE(filter->publishNoBoundingBoxPointcloud);
  EXPECT_TRUE(filter->computeOrientedBoundingBox);
  EXPECT_TRUE(filter->computeDebugOrientedBoundingBox);
  EXPECT_TRUE(filter->publishOrientedBoundingBoxMarker);
  EXPECT_TRUE(filter->publishNoOrientedBoundingBoxPointcloud);
  EXPECT_TRUE(filter->computeLocalBoundingBox);
  EXPECT_TRUE(filter->computeDebugLocalBoundingBox);
  EXPECT_TRUE(filter->publishLocalBoundingBoxMarker);
  EXPECT_TRUE(filter->publishNoLocalBoundingBoxPointcloud);
  EXPECT_EQ("base_link", filter->localBoundingBoxFrame);
  EXPECT_TRUE(filter->publishDebugPclInside);
  EXPECT_TRUE(filter->publishDebugPclClip);
  EXPECT_TRUE(filter->publishDebugPclShadow);
  EXPECT_TRUE(filter->publishDebugContainsMarker);
  EXPECT_TRUE(filter->publishDebugShadowMarker);
  EXPECT_TRUE(filter->requireAllFramesReachable);

  EXPECT_EQ("/robot_bounding_sphere", filter->boundingSpherePublisher.getTopic());
  EXPECT_EQ("/robot_bounding_box", filter->boundingBoxPublisher.getTopic());
  EXPECT_EQ("/robot_oriented_bounding_box", filter->orientedBoundingBoxPublisher.getTopic());
  EXPECT_EQ("/robot_local_bounding_box", filter->localBoundingBoxPublisher.getTopic());
  EXPECT_EQ("/robot_bounding_sphere_marker", filter->boundingSphereMarkerPublisher.getTopic());
  EXPECT_EQ("/robot_bounding_box_marker", filter->boundingBoxMarkerPublisher.getTopic());
  EXPECT_EQ("/robot_oriented_bounding_box_marker", filter->orientedBoundingBoxMarkerPublisher.getTopic());
  EXPECT_EQ("/robot_local_bounding_box_marker", filter->localBoundingBoxMarkerPublisher.getTopic());
  EXPECT_EQ("/robot_bounding_sphere_debug", filter->boundingSphereDebugMarkerPublisher.getTopic());
  EXPECT_EQ("/robot_bounding_box_debug", filter->boundingBoxDebugMarkerPublisher.getTopic());
  EXPECT_EQ("/robot_oriented_bounding_box_debug", filter->orientedBoundingBoxDebugMarkerPublisher.getTopic());
  EXPECT_EQ("/robot_local_bounding_box_debug", filter->localBoundingBoxDebugMarkerPublisher.getTopic());
  EXPECT_EQ("/scan_point_cloud_no_bsphere", filter->scanPointCloudNoBoundingSpherePublisher.getTopic());
  EXPECT_EQ("/scan_point_cloud_no_bbox", filter->scanPointCloudNoBoundingBoxPublisher.getTopic());
  EXPECT_EQ("/scan_point_cloud_no_oriented_bbox", filter->scanPointCloudNoOrientedBoundingBoxPublisher.getTopic());
  EXPECT_EQ("/scan_point_cloud_no_local_bbox", filter->scanPointCloudNoLocalBoundingBoxPublisher.getTopic());
  EXPECT_EQ("/scan_point_cloud_inside", filter->debugPointCloudInsidePublisher.getTopic());
  EXPECT_EQ("/scan_point_cloud_clip", filter->debugPointCloudClipPublisher.getTopic());
  EXPECT_EQ("/scan_point_cloud_shadow", filter->debugPointCloudShadowPublisher.getTopic());
  EXPECT_EQ("/robot_model_for_contains_test", filter->debugContainsMarkerPublisher.getTopic());
  EXPECT_EQ("/robot_model_for_shadow_test", filter->debugShadowMarkerPublisher.getTopic());

  EXPECT_EQ(ros::this_node::getName() + "/reload_model",
      filter->reloadRobotModelServiceServer.getService());
}

TEST(RobotBodyFilter, ParseRobot)
{
  ros::NodeHandle nh;

  auto filter = std::make_shared<RobotBodyFilterLaserScanTest>();
  auto filterBase = std::dynamic_pointer_cast<filters::FilterBase<sensor_msgs::LaserScan>>(filter);

  nh.setParam("test_robot_description", ROBOT_URDF);
  filterBase->configure("test_dict_config", nh);

  // base_link, base_link::big_collision_box::contains/shadow, laser::contains/shadow, antenna::contains/shadow
  EXPECT_EQ(7, filter->shapesToLinks.size());
  EXPECT_TRUE(filter->tfFramesWatchdog->isMonitored("laser"));
  EXPECT_TRUE(filter->tfFramesWatchdog->isMonitored("base_link"));
  EXPECT_EQ(7, filter->shapeMask->getBodies().size());
  EXPECT_EQ(4, filter->shapeMask->getBodiesForContainsTest().size());
  EXPECT_EQ(2, filter->shapeMask->getBodiesForShadowTest().size());

  filter->clearRobotMask();
  EXPECT_EQ(0, filter->shapesToLinks.size());
  EXPECT_FALSE(filter->tfFramesWatchdog->isMonitored("laser"));
  EXPECT_FALSE(filter->tfFramesWatchdog->isMonitored("base_link"));
  EXPECT_EQ(0, filter->shapeMask->getBodies().size());
  EXPECT_EQ(0, filter->shapeMask->getBodiesForContainsTest().size());
  EXPECT_EQ(0, filter->shapeMask->getBodiesForShadowTest().size());

  filter->addRobotMaskFromUrdf(ROBOT_URDF);
  EXPECT_EQ(7, filter->shapesToLinks.size());
  EXPECT_TRUE(filter->tfFramesWatchdog->isMonitored("laser"));
  EXPECT_TRUE(filter->tfFramesWatchdog->isMonitored("base_link"));
  EXPECT_EQ(7, filter->shapeMask->getBodies().size());
  EXPECT_EQ(4, filter->shapeMask->getBodiesForContainsTest().size());
  EXPECT_EQ(2, filter->shapeMask->getBodiesForShadowTest().size());

  nh.setParam("test_robot_description", "<robot name='test'></robot>");
  std_srvs::TriggerRequest req;
  std_srvs::TriggerResponse resp;
  EXPECT_TRUE(filter->triggerModelReload(req, resp));

  EXPECT_EQ(0, filter->shapesToLinks.size());
  EXPECT_FALSE(filter->tfFramesWatchdog->isMonitored("laser"));
  EXPECT_FALSE(filter->tfFramesWatchdog->isMonitored("base_link"));
  EXPECT_EQ(0, filter->shapeMask->getBodies().size());
  EXPECT_EQ(0, filter->shapeMask->getBodiesForContainsTest().size());
  EXPECT_EQ(0, filter->shapeMask->getBodiesForShadowTest().size());

  auto cfg = boost::make_shared<dynamic_reconfigure::Config>();
  dynamic_reconfigure::StrParameter param;
  param.name = "robot_model";
  param.value = ROBOT_URDF;
  cfg->strs.push_back(param);

  auto cfgConst = boost::const_pointer_cast<const dynamic_reconfigure::Config>(cfg);
  filter->robotDescriptionUpdated(cfgConst);
  EXPECT_EQ(7, filter->shapesToLinks.size());
  EXPECT_TRUE(filter->tfFramesWatchdog->isMonitored("laser"));
  EXPECT_TRUE(filter->tfFramesWatchdog->isMonitored("base_link"));
  EXPECT_EQ(7, filter->shapeMask->getBodies().size());
  EXPECT_EQ(4, filter->shapeMask->getBodiesForContainsTest().size());
  EXPECT_EQ(2, filter->shapeMask->getBodiesForShadowTest().size());

  // test reconfiguring (this happens when playing back a bag file and a new one starts playing)
  filter->clearRobotMask();
  nh.setParam("test_robot_description", ROBOT_URDF);
  filterBase->configure("test_dict_config", nh);
  EXPECT_EQ(7, filter->shapesToLinks.size());
  EXPECT_TRUE(filter->tfFramesWatchdog->isMonitored("laser"));
  EXPECT_TRUE(filter->tfFramesWatchdog->isMonitored("base_link"));
  EXPECT_EQ(7, filter->shapeMask->getBodies().size());
  EXPECT_EQ(4, filter->shapeMask->getBodiesForContainsTest().size());
  EXPECT_EQ(2, filter->shapeMask->getBodiesForShadowTest().size());
}

TEST(RobotBodyFilter, Transforms)
{
  ros::NodeHandle nh;

  auto filter = std::make_shared<RobotBodyFilterLaserScanTest>();
  auto filterBase = std::dynamic_pointer_cast<filters::FilterBase<sensor_msgs::LaserScan>>(filter);

  nh.setParam("test_robot_description", ROBOT_URDF);
  filterBase->configure("test_dict_config", nh);

  geometry_msgs::TransformStamped tf;
  tf.transform.rotation.w = 1.0;
  for (double d = -5.0; d < 5.0; d += 0.1)
  {
    tf.header.stamp = ros::Time::now() + ros::Duration(d);

    tf.transform.translation.x = 1;
    tf.header.frame_id = "odom";
    tf.child_frame_id = "base_link";
    filter->tfBuffer->setTransform(tf, "test");

    tf.transform.translation.x = -1.5;
    tf.header.frame_id = "base_link";
    tf.child_frame_id = "laser";
    filter->tfBuffer->setTransform(tf, "test");

    tf.transform.translation.x = 0.01864;
    tf.header.frame_id = "base_link";
    tf.child_frame_id = "antenna";
    filter->tfBuffer->setTransform(tf, "test");
  }

  for (double d = 25.0; d < 35.0; d += 0.1)
  {
    tf.header.stamp = ros::Time::now() + ros::Duration(d);

    tf.transform.translation.x = 11;
    tf.header.frame_id = "odom";
    tf.child_frame_id = "base_link";
    filter->tfBuffer->setTransform(tf, "test");

    tf.transform.translation.x = -8.5;
    tf.header.frame_id = "base_link";
    tf.child_frame_id = "laser";
    filter->tfBuffer->setTransform(tf, "test");

    tf.transform.translation.x = 10.01864;
    tf.header.frame_id = "base_link";
    tf.child_frame_id = "antenna";
    filter->tfBuffer->setTransform(tf, "test");
  }

  while (!filter->tfFramesWatchdog->isReachable("antenna"))
    ros::WallDuration(0.01).sleep();
  filter->updateTransformCache(ros::Time::now(), ros::Time::now() + ros::Duration(30));

  std::map<std::string, point_containment_filter::ShapeHandle> shapes;
  for (auto& pair : filter->shapesToLinks)
    shapes[pair.second.cacheKey] = pair.first;

  ASSERT_NE(shapes.end(), shapes.find("base_link-0"));
  ASSERT_NE(shapes.end(), shapes.find("base_link-1"));
  ASSERT_NE(shapes.end(), shapes.find("antenna-0"));
  ASSERT_NE(shapes.end(), shapes.find("laser-0"));

  Eigen::Isometry3d transform;

  filter->cacheLookupBetweenScansRatio = 0.0;
  // the positions do not exactly correspond to the ones from URDF; instead, they're composed of the
  // transforms defined above and the offsets of the collision shapes from their links' origins
  ASSERT_TRUE(filter->getShapeTransform(shapes["antenna-0"], transform));
  EXPECT_NEAR(1.0 -0.01864 + 0.01864, transform.translation().x(), 1e-6);
  ASSERT_TRUE(filter->getShapeTransform(shapes["laser-0"], transform));
  EXPECT_NEAR(1 - 1.5, transform.translation().x(), 1e-6);
  ASSERT_TRUE(filter->getShapeTransform(shapes["base_link-0"], transform));
  EXPECT_NEAR(1.0 - 0.1220, transform.translation().x(), 1e-6);
  ASSERT_TRUE(filter->getShapeTransform(shapes["base_link-1"], transform));
  EXPECT_NEAR(1.0 - 0.1, transform.translation().x(), 1e-6);

  filter->cacheLookupBetweenScansRatio = 1.0;
  ASSERT_TRUE(filter->getShapeTransform(shapes["antenna-0"], transform));
  EXPECT_NEAR(11 - 0.01864 + 10.01864, transform.translation().x(), 1e-6);
  ASSERT_TRUE(filter->getShapeTransform(shapes["laser-0"], transform));
  EXPECT_NEAR(11 - 8.5, transform.translation().x(), 1e-6);
  ASSERT_TRUE(filter->getShapeTransform(shapes["base_link-0"], transform));
  EXPECT_NEAR(11 - 0.1220, transform.translation().x(), 1e-6);
  ASSERT_TRUE(filter->getShapeTransform(shapes["base_link-1"], transform));
  EXPECT_NEAR(11 - 0.1, transform.translation().x(), 1e-6);
}


TEST(RobotBodyFilter, ComputeMaskPointByPoint)
{
  ros::NodeHandle nh;

  auto filter = std::make_shared<RobotBodyFilterLaserScanTest>();
  auto filterBase = std::dynamic_pointer_cast<filters::FilterBase<sensor_msgs::LaserScan>>(filter);

  nh.setParam("test_robot_description", ROBOT_URDF);
  filterBase->configure("compute_mask_config_point_by_point", nh);

  Cloud cloud;
  cloud.header.frame_id = filter->filteringFrame;
  CloudModifier mod(cloud);
  mod.setPointCloud2Fields(7,
                           "x", 1, sensor_msgs::PointField::FLOAT32,
                           "y", 1, sensor_msgs::PointField::FLOAT32,
                           "z", 1, sensor_msgs::PointField::FLOAT32,
                           "vp_x", 1, sensor_msgs::PointField::FLOAT32,
                           "vp_y", 1, sensor_msgs::PointField::FLOAT32,
                           "vp_z", 1, sensor_msgs::PointField::FLOAT32,
                           "stamps", 1, sensor_msgs::PointField::FLOAT32);
  mod.resize(11);

  {
    CloudIter x_it(cloud, "x");
    CloudIter y_it(cloud, "y");
    CloudIter z_it(cloud, "z");
    CloudIter vp_x_it(cloud, "vp_x");
    CloudIter vp_y_it(cloud, "vp_y");
    CloudIter vp_z_it(cloud, "vp_z");
    CloudIter stamps_it(cloud, "stamps");

    // pointSensor
    *x_it = -1.5; *y_it = 0; *z_it = 0; *vp_x_it = -1.5; *vp_y_it = 0; *vp_z_it = 0; *stamps_it = 0;
    ++x_it, ++y_it, ++z_it, ++vp_x_it, ++vp_y_it, ++vp_z_it, ++stamps_it;
    // pointSensor2
    *x_it = -1.47; *y_it = 0; *z_it = 0; *vp_x_it = -1.5; *vp_y_it = 0; *vp_z_it = 0; *stamps_it = 0;
    ++x_it, ++y_it, ++z_it, ++vp_x_it, ++vp_y_it, ++vp_z_it, ++stamps_it;
    // pointClipMin
    *x_it = -1.42; *y_it = 0; *z_it = 0; *vp_x_it = -1.5; *vp_y_it = 0; *vp_z_it = 0; *stamps_it = 0;
    ++x_it, ++y_it, ++z_it, ++vp_x_it, ++vp_y_it, ++vp_z_it, ++stamps_it;
    // pointClipMax
    *x_it = 10; *y_it = 0; *z_it = 0; *vp_x_it = -1.5; *vp_y_it = 0; *vp_z_it = 0; *stamps_it = 0;
    ++x_it, ++y_it, ++z_it, ++vp_x_it, ++vp_y_it, ++vp_z_it, ++stamps_it;
    // pointInBox
    *x_it = 0.85; *y_it = 0.85; *z_it = 0.85; *vp_x_it = -1.5; *vp_y_it = 0; *vp_z_it = 0; *stamps_it = 0;
    ++x_it, ++y_it, ++z_it, ++vp_x_it, ++vp_y_it, ++vp_z_it, ++stamps_it;
    // pointInSphere
    *x_it = 1.35; *y_it = 0; *z_it = 0; *vp_x_it = -1.5; *vp_y_it = 0; *vp_z_it = 0; *stamps_it = 0;
    ++x_it, ++y_it, ++z_it, ++vp_x_it, ++vp_y_it, ++vp_z_it, ++stamps_it;
    // pointInBoth
    *x_it = 0; *y_it = 0; *z_it = 0; *vp_x_it = -1.5; *vp_y_it = 0; *vp_z_it = 0; *stamps_it = 0;
    ++x_it, ++y_it, ++z_it, ++vp_x_it, ++vp_y_it, ++vp_z_it, ++stamps_it;
    // pointShadowBox
    *x_it = -0.25; *y_it = -2; *z_it = 2; *vp_x_it = -1.5; *vp_y_it = 0; *vp_z_it = 0; *stamps_it = 0;
    ++x_it, ++y_it, ++z_it, ++vp_x_it, ++vp_y_it, ++vp_z_it, ++stamps_it;
    // pointShadowSphere
    *x_it = -0.560762; *y_it = 0; *z_it = 1.83871; *vp_x_it = -1.5; *vp_y_it = 0; *vp_z_it = 0; *stamps_it = 0;
    ++x_it, ++y_it, ++z_it, ++vp_x_it, ++vp_y_it, ++vp_z_it, ++stamps_it;
    // pointShadowBoth
    *x_it = 1.5; *y_it = 0; *z_it = 0; *vp_x_it = -1.5; *vp_y_it = 0; *vp_z_it = 0; *stamps_it = 0;
    ++x_it, ++y_it, ++z_it, ++vp_x_it, ++vp_y_it, ++vp_z_it, ++stamps_it;
    // pointOutside
    *x_it = -3; *y_it = 0; *z_it = 0; *vp_x_it = -1.5; *vp_y_it = 0; *vp_z_it = 0; *stamps_it = 0;
    ++x_it, ++y_it, ++z_it, ++vp_x_it, ++vp_y_it, ++vp_z_it, ++stamps_it;
  }

  {
    ros::Time now = ros::Time::now();
    cloud.header.stamp = now;
    geometry_msgs::TransformStamped tf;
    tf.transform.rotation.w = 1.0;
    for (double d = -5.0; d < 5.0; d += 0.1)
    {
      tf.header.stamp = now + ros::Duration(d);

      tf.transform.translation.x = 0.122;
      tf.header.frame_id = "odom";
      tf.child_frame_id = "base_link";
      filter->tfBuffer->setTransform(tf, "test");

      tf.transform.translation.x = -1.5 - 0.122;
      tf.header.frame_id = "base_link";
      tf.child_frame_id = "laser";
      filter->tfBuffer->setTransform(tf, "test");

      tf.transform.translation.x = 0.01864 - 0.122;
      tf.header.frame_id = "base_link";
      tf.child_frame_id = "antenna";
      filter->tfBuffer->setTransform(tf, "test");
    }
  }

  while (!filter->tfFramesWatchdog->isReachable("antenna"))
    ros::WallDuration(0.01).sleep();
  filter->updateTransformCache(cloud.header.stamp, cloud.header.stamp + ros::Duration(1));

  std::map<std::string, point_containment_filter::ShapeHandle> shapes;
  for (auto& pair : filter->shapesToLinks)
    shapes[pair.second.cacheKey] = pair.first;

  ASSERT_NE(shapes.end(), shapes.find("base_link-0"));
  ASSERT_NE(shapes.end(), shapes.find("base_link-1"));
  ASSERT_NE(shapes.end(), shapes.find("antenna-0"));
  ASSERT_NE(shapes.end(), shapes.find("laser-0"));

  // we reconstruct the scene from test_ray_casting_shape_mask.blend
  Eigen::Isometry3d transform;
  filter->cacheLookupBetweenScansRatio = 0.0;
  ASSERT_TRUE(filter->getShapeTransform(shapes["antenna-0"], transform));
  EXPECT_NEAR(0, transform.translation().x(), 1e-6);
  ASSERT_TRUE(filter->getShapeTransform(shapes["laser-0"], transform));
  EXPECT_NEAR(-1.5, transform.translation().x(), 1e-6);
  ASSERT_TRUE(filter->getShapeTransform(shapes["base_link-0"], transform));
  EXPECT_NEAR(0, transform.translation().x(), 1e-6);
  ASSERT_TRUE(filter->getShapeTransform(shapes["base_link-1"], transform));
  EXPECT_NEAR(0.022, transform.translation().x(), 1e-6);

  std::vector<RayCastingShapeMask::MaskValue> mask;
  filter->computeMask(cloud, mask);

  ASSERT_EQ(num_points(cloud), mask.size());
  EXPECT_EQ(RayCastingShapeMask::MaskValue::CLIP, mask[0]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::CLIP, mask[1]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::CLIP, mask[2]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::CLIP, mask[3]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::INSIDE, mask[4]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::INSIDE, mask[5]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::INSIDE, mask[6]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, mask[7]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, mask[8]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, mask[9]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, mask[10]);

  // move some points by 5 meters, and some by 10 meters in x and change the relative timestamps so
  // that the 5-meter motion is 10 seconds in future and the 10-meter motion is 20 seconds
  // this will test the filter->cacheLookupBetweenScansRatio usage
  {
    CloudIter x_it(cloud, "x");
    CloudIter y_it(cloud, "y");
    CloudIter z_it(cloud, "z");
    CloudIter vp_x_it(cloud, "vp_x");
    CloudIter vp_y_it(cloud, "vp_y");
    CloudIter vp_z_it(cloud, "vp_z");
    CloudIter stamps_it(cloud, "stamps");

    // pointSensor
    *x_it = -1.5; *y_it = 0; *z_it = 0; *vp_x_it = -1.5; *vp_y_it = 0; *vp_z_it = 0; *stamps_it = 0;
    ++x_it, ++y_it, ++z_it, ++vp_x_it, ++vp_y_it, ++vp_z_it, ++stamps_it;
    // pointSensor2
    *x_it = -1.47; *y_it = 0; *z_it = 0; *vp_x_it = -1.5; *vp_y_it = 0; *vp_z_it = 0; *stamps_it = 0;
    ++x_it, ++y_it, ++z_it, ++vp_x_it, ++vp_y_it, ++vp_z_it, ++stamps_it;
    // pointClipMin
    *x_it = -1.42; *y_it = 0; *z_it = 0; *vp_x_it = -1.5; *vp_y_it = 0; *vp_z_it = 0; *stamps_it = 0;
    ++x_it, ++y_it, ++z_it, ++vp_x_it, ++vp_y_it, ++vp_z_it, ++stamps_it;
    // pointClipMax
    *x_it = 10; *y_it = 0; *z_it = 0; *vp_x_it = -1.5; *vp_y_it = 0; *vp_z_it = 0; *stamps_it = 0;
    ++x_it, ++y_it, ++z_it, ++vp_x_it, ++vp_y_it, ++vp_z_it, ++stamps_it;
    // pointInBox
    *x_it = 5.85; *y_it = 0.85; *z_it = 0.85; *vp_x_it = 3.5; *vp_y_it = 0; *vp_z_it = 0; *stamps_it = 10;
    ++x_it, ++y_it, ++z_it, ++vp_x_it, ++vp_y_it, ++vp_z_it, ++stamps_it;
    // pointInSphere
    *x_it = 6.35; *y_it = 0; *z_it = 0; *vp_x_it = 3.5; *vp_y_it = 0; *vp_z_it = 0; *stamps_it = 10;
    ++x_it, ++y_it, ++z_it, ++vp_x_it, ++vp_y_it, ++vp_z_it, ++stamps_it;
    // pointInBoth
    *x_it = 5; *y_it = 0; *z_it = 0; *vp_x_it = 3.5; *vp_y_it = 0; *vp_z_it = 0; *stamps_it = 10;
    ++x_it, ++y_it, ++z_it, ++vp_x_it, ++vp_y_it, ++vp_z_it, ++stamps_it;
    // pointShadowBox
    *x_it = 5-0.25; *y_it = -2; *z_it = 2; *vp_x_it = 3.5; *vp_y_it = 0; *vp_z_it = 0; *stamps_it = 10;
    ++x_it, ++y_it, ++z_it, ++vp_x_it, ++vp_y_it, ++vp_z_it, ++stamps_it;
    // pointShadowSphere
    *x_it = 10-0.560762; *y_it = 0; *z_it = 1.83871; *vp_x_it = 8.5; *vp_y_it = 0; *vp_z_it = 0; *stamps_it = 20;
    ++x_it, ++y_it, ++z_it, ++vp_x_it, ++vp_y_it, ++vp_z_it, ++stamps_it;
    // pointShadowBoth
    *x_it = 11.5; *y_it = 0; *z_it = 0; *vp_x_it = 8.5; *vp_y_it = 0; *vp_z_it = 0; *stamps_it = 20;
    ++x_it, ++y_it, ++z_it, ++vp_x_it, ++vp_y_it, ++vp_z_it, ++stamps_it;
    // pointOutside
    *x_it = 7; *y_it = 0; *z_it = 0; *vp_x_it = 8.5; *vp_y_it = 0; *vp_z_it = 0; *stamps_it = 20;
    ++x_it, ++y_it, ++z_it, ++vp_x_it, ++vp_y_it, ++vp_z_it, ++stamps_it;
  }

  {
    geometry_msgs::TransformStamped tf;
    ros::Time now = ros::Time::now();
    cloud.header.stamp = now;
    tf.transform.rotation.w = 1;
    for (double d = -5.0; d < 5.0; d += 0.1)
    {
      tf.header.stamp = now + ros::Duration(d);

      tf.transform.translation.x = 0.122;
      tf.header.frame_id = "odom";
      tf.child_frame_id = "base_link";
      ASSERT_TRUE(filter->tfBuffer->setTransform(tf, "test"));

      tf.transform.translation.x = -1.5 - 0.122;
      tf.header.frame_id = "base_link";
      tf.child_frame_id = "laser";
      ASSERT_TRUE(filter->tfBuffer->setTransform(tf, "test"));

      tf.transform.translation.x = 0.01864 - 0.122;
      tf.header.frame_id = "base_link";
      tf.child_frame_id = "antenna";
      ASSERT_TRUE(filter->tfBuffer->setTransform(tf, "test"));

      tf.header.stamp += ros::Duration(20);

      tf.transform.translation.x = 10.122;
      tf.header.frame_id = "odom";
      tf.child_frame_id = "base_link";
      ASSERT_TRUE(filter->tfBuffer->setTransform(tf, "test"));

      tf.transform.translation.x = -1.5 - 0.122;
      tf.header.frame_id = "base_link";
      tf.child_frame_id = "laser";
      ASSERT_TRUE(filter->tfBuffer->setTransform(tf, "test"));

      tf.transform.translation.x = 0.01864 - 0.122;
      tf.header.frame_id = "base_link";
      tf.child_frame_id = "antenna";
      ASSERT_TRUE(filter->tfBuffer->setTransform(tf, "test"));
    }
  }
  filter->updateTransformCache(cloud.header.stamp, cloud.header.stamp + ros::Duration(20));

  SphereStampedConstPtr boundingSphere;
  geometry_msgs::PolygonStampedConstPtr boundingBox;
  OrientedBoundingBoxStampedConstPtr orientedBoundingBox;
  geometry_msgs::PolygonStampedConstPtr localBoundingBox;
  visualization_msgs::MarkerConstPtr boundingSphereMarker;
  visualization_msgs::MarkerConstPtr boundingBoxMarker;
  visualization_msgs::MarkerConstPtr orientedBoundingBoxMarker;
  visualization_msgs::MarkerConstPtr localBoundingBoxMarker;
  visualization_msgs::MarkerArrayConstPtr boundingSphereDebugMarker;
  visualization_msgs::MarkerArrayConstPtr boundingBoxDebugMarker;
  visualization_msgs::MarkerArrayConstPtr orientedBoundingBoxDebugMarker;
  visualization_msgs::MarkerArrayConstPtr localBoundingBoxDebugMarker;
  sensor_msgs::PointCloud2ConstPtr pclNoBoundingSphere;
  sensor_msgs::PointCloud2ConstPtr pclNoBoundingBox;
  sensor_msgs::PointCloud2ConstPtr pclNoOrientedBoundingBox;
  sensor_msgs::PointCloud2ConstPtr pclNoLocalBoundingBox;
  sensor_msgs::PointCloud2ConstPtr pclInside;
  sensor_msgs::PointCloud2ConstPtr pclClip;
  sensor_msgs::PointCloud2ConstPtr pclShadow;
  visualization_msgs::MarkerArrayConstPtr robotModelContainsTest;
  visualization_msgs::MarkerArrayConstPtr robotModelShadowTest;

  ros::Subscriber boundingSphereSubscriber = nh.subscribe<SphereStamped>("/robot_bounding_sphere", 10, [&](const SphereStampedConstPtr& msg){boundingSphere=msg;});
  ros::Subscriber boundingBoxSubscriber = nh.subscribe<geometry_msgs::PolygonStamped>("/robot_bounding_box", 10, [&](const geometry_msgs::PolygonStampedConstPtr& msg){boundingBox=msg;});
  ros::Subscriber orientedBoundingBoxSubscriber = nh.subscribe<OrientedBoundingBoxStamped>("/robot_oriented_bounding_box", 10, [&](const OrientedBoundingBoxStampedConstPtr& msg){orientedBoundingBox=msg;});
  ros::Subscriber localBoundingBoxSubscriber = nh.subscribe<geometry_msgs::PolygonStamped>("/robot_local_bounding_box", 10, [&](const geometry_msgs::PolygonStampedConstPtr& msg){localBoundingBox=msg;});
  ros::Subscriber boundingSphereMarkerSubscriber = nh.subscribe<visualization_msgs::Marker>("/robot_bounding_sphere_marker", 10, [&](const visualization_msgs::MarkerConstPtr& msg){boundingSphereMarker=msg;});
  ros::Subscriber boundingBoxMarkerSubscriber = nh.subscribe<visualization_msgs::Marker>("/robot_bounding_box_marker", 10, [&](const visualization_msgs::MarkerConstPtr& msg){boundingBoxMarker=msg;});
  ros::Subscriber orientedBoundingBoxMarkerSubscriber = nh.subscribe<visualization_msgs::Marker>("/robot_oriented_bounding_box_marker", 10, [&](const visualization_msgs::MarkerConstPtr& msg){orientedBoundingBoxMarker=msg;});
  ros::Subscriber localBoundingBoxMarkerSubscriber = nh.subscribe<visualization_msgs::Marker>("/robot_local_bounding_box_marker", 10, [&](const visualization_msgs::MarkerConstPtr& msg){localBoundingBoxMarker=msg;});
  ros::Subscriber boundingSphereDebugMarkerSubscriber = nh.subscribe<visualization_msgs::MarkerArray>("/robot_bounding_sphere_debug", 10, [&](const visualization_msgs::MarkerArrayConstPtr& msg){boundingSphereDebugMarker=msg;});
  ros::Subscriber boundingBoxDebugMarkerSubscriber = nh.subscribe<visualization_msgs::MarkerArray>("/robot_bounding_box_debug", 10, [&](const visualization_msgs::MarkerArrayConstPtr& msg){boundingBoxDebugMarker=msg;});
  ros::Subscriber orientedBoundingBoxDebugMarkerSubscriber = nh.subscribe<visualization_msgs::MarkerArray>("/robot_oriented_bounding_box_debug", 10, [&](const visualization_msgs::MarkerArrayConstPtr& msg){orientedBoundingBoxDebugMarker=msg;});
  ros::Subscriber localBoundingBoxDebugMarkerSubscriber = nh.subscribe<visualization_msgs::MarkerArray>("/robot_local_bounding_box_debug", 10, [&](const visualization_msgs::MarkerArrayConstPtr& msg){localBoundingBoxDebugMarker=msg;});
  ros::Subscriber scanPointCloudNoBoundingSphereSubscriber = nh.subscribe<sensor_msgs::PointCloud2>("/scan_point_cloud_no_bsphere", 10, [&](const sensor_msgs::PointCloud2ConstPtr& msg){pclNoBoundingSphere=msg;});
  ros::Subscriber scanPointCloudNoBoundingBoxSubscriber = nh.subscribe<sensor_msgs::PointCloud2>("/scan_point_cloud_no_bbox", 10, [&](const sensor_msgs::PointCloud2ConstPtr& msg){pclNoBoundingBox=msg;});
  ros::Subscriber scanPointCloudNoOrientedBoundingBoxSubscriber = nh.subscribe<sensor_msgs::PointCloud2>("/scan_point_cloud_no_oriented_bbox", 10, [&](const sensor_msgs::PointCloud2ConstPtr& msg){pclNoOrientedBoundingBox=msg;});
  ros::Subscriber scanPointCloudNoLocalBoundingBoxSubscriber = nh.subscribe<sensor_msgs::PointCloud2>("/scan_point_cloud_no_local_bbox", 10, [&](const sensor_msgs::PointCloud2ConstPtr& msg){pclNoLocalBoundingBox=msg;});
  ros::Subscriber debugPointCloudInsideSubscriber = nh.subscribe<sensor_msgs::PointCloud2>("/scan_point_cloud_inside", 10, [&](const sensor_msgs::PointCloud2ConstPtr& msg){pclInside=msg;});
  ros::Subscriber debugPointCloudClipSubscriber = nh.subscribe<sensor_msgs::PointCloud2>("/scan_point_cloud_clip", 10, [&](const sensor_msgs::PointCloud2ConstPtr& msg){pclClip=msg;});
  ros::Subscriber debugPointCloudShadowSubscriber = nh.subscribe<sensor_msgs::PointCloud2>("/scan_point_cloud_shadow", 10, [&](const sensor_msgs::PointCloud2ConstPtr& msg){pclShadow=msg;});
  ros::Subscriber debugContainsMarkerSubscriber = nh.subscribe<visualization_msgs::MarkerArray>("/robot_model_for_contains_test", 10, [&](const visualization_msgs::MarkerArrayConstPtr& msg){robotModelContainsTest=msg;});
  ros::Subscriber debugShadowMarkerSubscriber = nh.subscribe<visualization_msgs::MarkerArray>("/robot_model_for_shadow_test", 10, [&](const visualization_msgs::MarkerArrayConstPtr& msg){robotModelShadowTest=msg;});

  filter->computeMask(cloud, mask);

  ASSERT_EQ(num_points(cloud), mask.size());
  EXPECT_EQ(RayCastingShapeMask::MaskValue::CLIP, mask[0]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::CLIP, mask[1]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::CLIP, mask[2]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::CLIP, mask[3]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::INSIDE, mask[4]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::INSIDE, mask[5]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::INSIDE, mask[6]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, mask[7]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, mask[8]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, mask[9]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, mask[10]);

  WAIT_FOR_MESSAGE(boundingSphere)
  WAIT_FOR_MESSAGE(boundingBox)
  WAIT_FOR_MESSAGE(orientedBoundingBox)
  WAIT_FOR_MESSAGE(localBoundingBox)
  WAIT_FOR_MESSAGE(boundingSphereMarker)
  WAIT_FOR_MESSAGE(boundingBoxMarker)
  WAIT_FOR_MESSAGE(orientedBoundingBoxMarker)
  WAIT_FOR_MESSAGE(localBoundingBoxMarker)
  WAIT_FOR_MESSAGE(boundingSphereDebugMarker)
  WAIT_FOR_MESSAGE(boundingBoxDebugMarker)
  WAIT_FOR_MESSAGE(orientedBoundingBoxDebugMarker)
  WAIT_FOR_MESSAGE(localBoundingBoxDebugMarker)
  WAIT_FOR_MESSAGE(pclNoBoundingSphere)
  WAIT_FOR_MESSAGE(pclNoBoundingBox)
  WAIT_FOR_MESSAGE(pclNoOrientedBoundingBox)
  WAIT_FOR_MESSAGE(pclNoLocalBoundingBox)
  WAIT_FOR_MESSAGE(pclInside)
  WAIT_FOR_MESSAGE(pclClip)
  WAIT_FOR_MESSAGE(pclShadow)
  WAIT_FOR_MESSAGE(robotModelContainsTest)
  WAIT_FOR_MESSAGE(robotModelShadowTest)

  EXPECT_EQ("odom", boundingSphere->header.frame_id);
  EXPECT_EQ(cloud.header.stamp, boundingSphere->header.stamp);
  EXPECT_NEAR(sqrt(3) * 0.92, boundingSphere->sphere.radius, 1e-6);
  EXPECT_NEAR(0.0, boundingSphere->sphere.center.x, 1e-6);
  EXPECT_DOUBLE_EQ(0, boundingSphere->sphere.center.y);
  EXPECT_DOUBLE_EQ(0, boundingSphere->sphere.center.z);
  EXPECT_EQ("odom", boundingSphereMarker->header.frame_id);
  EXPECT_EQ(cloud.header.stamp, boundingSphereMarker->header.stamp);
  EXPECT_NEAR(2*sqrt(3) * 0.92, boundingSphereMarker->scale.x, 1e-6);
  EXPECT_NEAR(2*sqrt(3) * 0.92, boundingSphereMarker->scale.y, 1e-6);
  EXPECT_NEAR(2*sqrt(3) * 0.92, boundingSphereMarker->scale.z, 1e-6);
  EXPECT_NEAR(0, boundingSphereMarker->pose.position.x, 1e-6);
  EXPECT_NEAR(0, boundingSphereMarker->pose.position.y, 1e-6);
  EXPECT_NEAR(0, boundingSphereMarker->pose.position.z, 1e-6);
  EXPECT_NEAR(0, boundingSphereMarker->pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, boundingSphereMarker->pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, boundingSphereMarker->pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, boundingSphereMarker->pose.orientation.w, 1e-6);
  EXPECT_LT(0, boundingSphereMarker->color.a);
  // make sure the marker has some color
  EXPECT_LT(0, boundingSphereMarker->color.r + boundingSphereMarker->color.g +
               boundingSphereMarker->color.b);
  EXPECT_EQ(visualization_msgs::Marker::SPHERE, boundingSphereMarker->type);
  EXPECT_EQ(visualization_msgs::Marker::ADD, boundingSphereMarker->action);
  EXPECT_EQ("bounding_sphere", boundingSphereMarker->ns);
  EXPECT_EQ(1, boundingSphereMarker->frame_locked);

  EXPECT_EQ("odom", boundingBox->header.frame_id);
  EXPECT_EQ(cloud.header.stamp, boundingBox->header.stamp);
  ASSERT_EQ(2, boundingBox->polygon.points.size());
  EXPECT_NEAR(-1.55, boundingBox->polygon.points[0].x, 1e-5);
  EXPECT_NEAR(-1.375, boundingBox->polygon.points[0].y, 1e-6);
  EXPECT_NEAR(-1.375, boundingBox->polygon.points[0].z, 1e-6);
  EXPECT_NEAR(1.375, boundingBox->polygon.points[1].x, 1e-5);
  EXPECT_NEAR(1.375, boundingBox->polygon.points[1].y, 1e-6);
  EXPECT_NEAR(1.375, boundingBox->polygon.points[1].z, 1e-6);
  EXPECT_EQ("odom", boundingBoxMarker->header.frame_id);
  EXPECT_EQ(cloud.header.stamp, boundingBoxMarker->header.stamp);
  EXPECT_NEAR(2.925, boundingBoxMarker->scale.x, 1e-5);
  EXPECT_NEAR(2.75, boundingBoxMarker->scale.y, 1e-5);
  EXPECT_NEAR(2.75, boundingBoxMarker->scale.z, 1e-5);
  EXPECT_NEAR(-0.0875, boundingBoxMarker->pose.position.x, 1e-6);
  EXPECT_NEAR(0, boundingBoxMarker->pose.position.y, 1e-6);
  EXPECT_NEAR(0, boundingBoxMarker->pose.position.z, 1e-6);
  EXPECT_NEAR(0, boundingBoxMarker->pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, boundingBoxMarker->pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, boundingBoxMarker->pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, boundingBoxMarker->pose.orientation.w, 1e-6);
  EXPECT_LT(0, boundingBoxMarker->color.a);
  // make sure the marker has some color
  EXPECT_LT(0, boundingBoxMarker->color.r + boundingBoxMarker->color.g +
      boundingBoxMarker->color.b);
  EXPECT_EQ(visualization_msgs::Marker::CUBE, boundingBoxMarker->type);
  EXPECT_EQ(visualization_msgs::Marker::ADD, boundingBoxMarker->action);
  EXPECT_EQ("bounding_box", boundingBoxMarker->ns);
  EXPECT_EQ(1, boundingBoxMarker->frame_locked);

  EXPECT_EQ("odom", orientedBoundingBox->header.frame_id);
  EXPECT_EQ(cloud.header.stamp, orientedBoundingBox->header.stamp);
  EXPECT_NEAR(2.925, orientedBoundingBox->obb.extents.x, 1e-5);
  EXPECT_NEAR(2.75, orientedBoundingBox->obb.extents.y, 1e-5);
  EXPECT_NEAR(2.75, orientedBoundingBox->obb.extents.z, 1e-5);
  EXPECT_NEAR(-0.0875, orientedBoundingBox->obb.pose.translation.x, 1e-5);
  EXPECT_NEAR(0, orientedBoundingBox->obb.pose.translation.y, 1e-6);
  EXPECT_NEAR(0, orientedBoundingBox->obb.pose.translation.z, 1e-6);
  EXPECT_NEAR(0, orientedBoundingBox->obb.pose.rotation.x, 1e-6);
  EXPECT_NEAR(0, orientedBoundingBox->obb.pose.rotation.y, 1e-6);
  EXPECT_NEAR(0, orientedBoundingBox->obb.pose.rotation.z, 1e-6);
  EXPECT_NEAR(1, orientedBoundingBox->obb.pose.rotation.w, 1e-6);
  EXPECT_EQ("odom", orientedBoundingBoxMarker->header.frame_id);
  EXPECT_EQ(cloud.header.stamp, orientedBoundingBoxMarker->header.stamp);
  EXPECT_NEAR(2.925, orientedBoundingBoxMarker->scale.x, 1e-5);
  EXPECT_NEAR(2.75, orientedBoundingBoxMarker->scale.y, 1e-5);
  EXPECT_NEAR(2.75, orientedBoundingBoxMarker->scale.z, 1e-5);
  EXPECT_NEAR(-0.0875, orientedBoundingBoxMarker->pose.position.x, 1e-6);
  EXPECT_NEAR(0, orientedBoundingBoxMarker->pose.position.y, 1e-6);
  EXPECT_NEAR(0, orientedBoundingBoxMarker->pose.position.z, 1e-6);
  EXPECT_NEAR(0, orientedBoundingBoxMarker->pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, orientedBoundingBoxMarker->pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, orientedBoundingBoxMarker->pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, orientedBoundingBoxMarker->pose.orientation.w, 1e-6);
  EXPECT_LT(0, orientedBoundingBoxMarker->color.a);
  // make sure the marker has some color
  EXPECT_LT(0, orientedBoundingBoxMarker->color.r + orientedBoundingBoxMarker->color.g +
      orientedBoundingBoxMarker->color.b);
  EXPECT_EQ(visualization_msgs::Marker::CUBE, orientedBoundingBoxMarker->type);
  EXPECT_EQ(visualization_msgs::Marker::ADD, orientedBoundingBoxMarker->action);
  EXPECT_EQ("oriented_bounding_box", orientedBoundingBoxMarker->ns);
  EXPECT_EQ(1, orientedBoundingBoxMarker->frame_locked);

  EXPECT_EQ("base_link", localBoundingBox->header.frame_id);
  EXPECT_EQ(cloud.header.stamp, localBoundingBox->header.stamp);
  ASSERT_EQ(2, localBoundingBox->polygon.points.size());
  EXPECT_NEAR(-1.55 - 0.122, localBoundingBox->polygon.points[0].x, 1e-5);
  EXPECT_NEAR(-1.375, localBoundingBox->polygon.points[0].y, 1e-6);
  EXPECT_NEAR(-1.375, localBoundingBox->polygon.points[0].z, 1e-6);
  EXPECT_NEAR(1.375 - 0.122, localBoundingBox->polygon.points[1].x, 1e-5);
  EXPECT_NEAR(1.375, localBoundingBox->polygon.points[1].y, 1e-6);
  EXPECT_NEAR(1.375, localBoundingBox->polygon.points[1].z, 1e-6);
  EXPECT_EQ("base_link", localBoundingBoxMarker->header.frame_id);
  EXPECT_EQ(cloud.header.stamp, localBoundingBoxMarker->header.stamp);
  EXPECT_NEAR(2.925, localBoundingBoxMarker->scale.x, 1e-5);
  EXPECT_NEAR(2.75, localBoundingBoxMarker->scale.y, 1e-5);
  EXPECT_NEAR(2.75, localBoundingBoxMarker->scale.z, 1e-5);
  EXPECT_NEAR(-0.0875 - 0.122, localBoundingBoxMarker->pose.position.x, 1e-6);
  EXPECT_NEAR(0, localBoundingBoxMarker->pose.position.y, 1e-6);
  EXPECT_NEAR(0, localBoundingBoxMarker->pose.position.z, 1e-6);
  EXPECT_NEAR(0, localBoundingBoxMarker->pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, localBoundingBoxMarker->pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, localBoundingBoxMarker->pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, localBoundingBoxMarker->pose.orientation.w, 1e-6);
  EXPECT_LT(0, localBoundingBoxMarker->color.a);
  // make sure the marker has some color
  EXPECT_LT(0, localBoundingBoxMarker->color.r + localBoundingBoxMarker->color.g +
      localBoundingBoxMarker->color.b);
  EXPECT_EQ(visualization_msgs::Marker::CUBE, localBoundingBoxMarker->type);
  EXPECT_EQ(visualization_msgs::Marker::ADD, localBoundingBoxMarker->action);
  EXPECT_EQ("local_bounding_box", localBoundingBoxMarker->ns);
  EXPECT_EQ(1, localBoundingBoxMarker->frame_locked);

  ASSERT_EQ(8, num_points(*pclNoBoundingSphere));
  CloudConstIter x_it(*pclNoBoundingSphere, "x");
//  EXPECT_NEAR(-1.5, *x_it, 1e-6); ++x_it; // pointSensor
//  EXPECT_NEAR(-1.47, *x_it, 1e-6); ++x_it; // pointSensor2
//  EXPECT_NEAR(-1.42, *x_it, 1e-6); ++x_it; // pointClipMin
  EXPECT_NEAR(10, *x_it, 1e-6); ++x_it; // pointClipMax
  EXPECT_NEAR(5.85, *x_it, 1e-6); ++x_it; // pointInBox
  EXPECT_NEAR(6.35, *x_it, 1e-6); ++x_it; // pointInSphere
  EXPECT_NEAR(5, *x_it, 1e-6); ++x_it; // pointInBoth
  EXPECT_NEAR(5-0.25, *x_it, 1e-6); ++x_it; // pointShadowBox
  EXPECT_NEAR(10-0.560762, *x_it, 1e-6); ++x_it; // pointShadowSphere
  EXPECT_NEAR(11.5, *x_it, 1e-6); ++x_it; // pointShadowBoth
  EXPECT_NEAR(7, *x_it, 1e-6); ++x_it; // pointOutside

  ASSERT_EQ(8, num_points(*pclNoBoundingBox));
  x_it = CloudConstIter(*pclNoBoundingBox, "x");
//  EXPECT_NEAR(-1.5, *x_it, 1e-6); ++x_it; // pointSensor
//  EXPECT_NEAR(-1.47, *x_it, 1e-6); ++x_it; // pointSensor2
//  EXPECT_NEAR(-1.42, *x_it, 1e-6); ++x_it; // pointClipMin
  EXPECT_NEAR(10, *x_it, 1e-6); ++x_it; // pointClipMax
  EXPECT_NEAR(5.85, *x_it, 1e-6); ++x_it; // pointInBox
  EXPECT_NEAR(6.35, *x_it, 1e-6); ++x_it; // pointInSphere
  EXPECT_NEAR(5, *x_it, 1e-6); ++x_it; // pointInBoth
  EXPECT_NEAR(5-0.25, *x_it, 1e-6); ++x_it; // pointShadowBox
  EXPECT_NEAR(10-0.560762, *x_it, 1e-6); ++x_it; // pointShadowSphere
  EXPECT_NEAR(11.5, *x_it, 1e-6); ++x_it; // pointShadowBoth
  EXPECT_NEAR(7, *x_it, 1e-6); ++x_it; // pointOutside

  ASSERT_EQ(8, num_points(*pclNoOrientedBoundingBox));
  x_it = CloudConstIter(*pclNoOrientedBoundingBox, "x");
//  EXPECT_NEAR(-1.5, *x_it, 1e-6); ++x_it; // pointSensor
//  EXPECT_NEAR(-1.47, *x_it, 1e-6); ++x_it; // pointSensor2
//  EXPECT_NEAR(-1.42, *x_it, 1e-6); ++x_it; // pointClipMin
  EXPECT_NEAR(10, *x_it, 1e-6); ++x_it; // pointClipMax
  EXPECT_NEAR(5.85, *x_it, 1e-6); ++x_it; // pointInBox
  EXPECT_NEAR(6.35, *x_it, 1e-6); ++x_it; // pointInSphere
  EXPECT_NEAR(5, *x_it, 1e-6); ++x_it; // pointInBoth
  EXPECT_NEAR(5-0.25, *x_it, 1e-6); ++x_it; // pointShadowBox
  EXPECT_NEAR(10-0.560762, *x_it, 1e-6); ++x_it; // pointShadowSphere
  EXPECT_NEAR(11.5, *x_it, 1e-6); ++x_it; // pointShadowBoth
  EXPECT_NEAR(7, *x_it, 1e-6); ++x_it; // pointOutside

  ASSERT_EQ(8, num_points(*pclNoLocalBoundingBox));
  x_it = CloudConstIter(*pclNoLocalBoundingBox, "x");
//  EXPECT_NEAR(-1.5, *x_it, 1e-6); ++x_it; // pointSensor
//  EXPECT_NEAR(-1.47, *x_it, 1e-6); ++x_it; // pointSensor2
//  EXPECT_NEAR(-1.42, *x_it, 1e-6); ++x_it; // pointClipMin
  EXPECT_NEAR(10, *x_it, 1e-6); ++x_it; // pointClipMax
  EXPECT_NEAR(5.85, *x_it, 1e-6); ++x_it; // pointInBox
  EXPECT_NEAR(6.35, *x_it, 1e-6); ++x_it; // pointInSphere
  EXPECT_NEAR(5, *x_it, 1e-6); ++x_it; // pointInBoth
  EXPECT_NEAR(5-0.25, *x_it, 1e-6); ++x_it; // pointShadowBox
  EXPECT_NEAR(10-0.560762, *x_it, 1e-6); ++x_it; // pointShadowSphere
  EXPECT_NEAR(11.5, *x_it, 1e-6); ++x_it; // pointShadowBoth
  EXPECT_NEAR(7, *x_it, 1e-6); ++x_it; // pointOutside

  ASSERT_EQ(3, num_points(*pclInside));
  x_it = CloudConstIter(*pclInside, "x");
//  EXPECT_NEAR(-1.5, *x_it, 1e-6); ++x_it; // pointSensor
//  EXPECT_NEAR(-1.47, *x_it, 1e-6); ++x_it; // pointSensor2
//  EXPECT_NEAR(-1.42, *x_it, 1e-6); ++x_it; // pointClipMin
//  EXPECT_NEAR(10, *x_it, 1e-6); ++x_it; // pointClipMax
  EXPECT_NEAR(5.85, *x_it, 1e-6); ++x_it; // pointInBox
  EXPECT_NEAR(6.35, *x_it, 1e-6); ++x_it; // pointInSphere
  EXPECT_NEAR(5, *x_it, 1e-6); ++x_it; // pointInBoth
//  EXPECT_NEAR(5-0.25, *x_it, 1e-6); ++x_it; // pointShadowBox
//  EXPECT_NEAR(10-0.560762, *x_it, 1e-6); ++x_it; // pointShadowSphere
//  EXPECT_NEAR(11.5, *x_it, 1e-6); ++x_it; // pointShadowBoth
//  EXPECT_NEAR(7, *x_it, 1e-6); ++x_it; // pointOutside

  ASSERT_EQ(4, num_points(*pclClip));
  x_it = CloudConstIter(*pclClip, "x");
  EXPECT_NEAR(-1.5, *x_it, 1e-6); ++x_it; // pointSensor
  EXPECT_NEAR(-1.47, *x_it, 1e-6); ++x_it; // pointSensor2
  EXPECT_NEAR(-1.42, *x_it, 1e-6); ++x_it; // pointClipMin
  EXPECT_NEAR(10, *x_it, 1e-6); ++x_it; // pointClipMax
//  EXPECT_NEAR(5.85, *x_it, 1e-6); ++x_it; // pointInBox
//  EXPECT_NEAR(6.35, *x_it, 1e-6); ++x_it; // pointInSphere
//  EXPECT_NEAR(5, *x_it, 1e-6); ++x_it; // pointInBoth
//  EXPECT_NEAR(5-0.25, *x_it, 1e-6); ++x_it; // pointShadowBox
//  EXPECT_NEAR(10-0.560762, *x_it, 1e-6); ++x_it; // pointShadowSphere
//  EXPECT_NEAR(11.5, *x_it, 1e-6); ++x_it; // pointShadowBoth
//  EXPECT_NEAR(7, *x_it, 1e-6); ++x_it; // pointOutside

  ASSERT_EQ(3, num_points(*pclShadow));
  x_it = CloudConstIter(*pclShadow, "x");
//  EXPECT_NEAR(-1.5, *x_it, 1e-6); ++x_it; // pointSensor
//  EXPECT_NEAR(-1.47, *x_it, 1e-6); ++x_it; // pointSensor2
//  EXPECT_NEAR(-1.42, *x_it, 1e-6); ++x_it; // pointClipMin
//  EXPECT_NEAR(10, *x_it, 1e-6); ++x_it; // pointClipMax
//  EXPECT_NEAR(5.85, *x_it, 1e-6); ++x_it; // pointInBox
//  EXPECT_NEAR(6.35, *x_it, 1e-6); ++x_it; // pointInSphere
//  EXPECT_NEAR(5, *x_it, 1e-6); ++x_it; // pointInBoth
  EXPECT_NEAR(5-0.25, *x_it, 1e-6); ++x_it; // pointShadowBox
  EXPECT_NEAR(10-0.560762, *x_it, 1e-6); ++x_it; // pointShadowSphere
  EXPECT_NEAR(11.5, *x_it, 1e-6); ++x_it; // pointShadowBoth
//  EXPECT_NEAR(7, *x_it, 1e-6); ++x_it; // pointOutside

  ASSERT_EQ(2, boundingSphereDebugMarker->markers.size());
  EXPECT_EQ("odom", boundingSphereDebugMarker->markers[0].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, boundingSphereDebugMarker->markers[0].header.stamp);
  EXPECT_NEAR(2*sqrt(3) * 0.92, boundingSphereDebugMarker->markers[0].scale.x, 1e-6);
  EXPECT_NEAR(2*sqrt(3) * 0.92, boundingSphereDebugMarker->markers[0].scale.y, 1e-6);
  EXPECT_NEAR(2*sqrt(3) * 0.92, boundingSphereDebugMarker->markers[0].scale.z, 1e-6);
  EXPECT_NEAR(0, boundingSphereDebugMarker->markers[0].pose.position.x, 1e-6);
  EXPECT_NEAR(0, boundingSphereDebugMarker->markers[0].pose.position.y, 1e-6);
  EXPECT_NEAR(0, boundingSphereDebugMarker->markers[0].pose.position.z, 1e-6);
  EXPECT_NEAR(0, boundingSphereDebugMarker->markers[0].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, boundingSphereDebugMarker->markers[0].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, boundingSphereDebugMarker->markers[0].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, boundingSphereDebugMarker->markers[0].pose.orientation.w, 1e-6);
  EXPECT_LT(0, boundingSphereDebugMarker->markers[0].color.a);
  // make sure the marker has some color
  EXPECT_LT(0, boundingSphereDebugMarker->markers[0].color.r + boundingSphereDebugMarker->markers[0].color.g +
      boundingSphereDebugMarker->markers[0].color.b);
  EXPECT_EQ(visualization_msgs::Marker::SPHERE, boundingSphereDebugMarker->markers[0].type);
  EXPECT_EQ(visualization_msgs::Marker::ADD, boundingSphereDebugMarker->markers[0].action);
  EXPECT_FALSE(boundingSphereDebugMarker->markers[0].ns.empty());
  EXPECT_EQ(1, boundingSphereDebugMarker->markers[0].frame_locked);
  EXPECT_EQ("odom", boundingSphereDebugMarker->markers[1].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, boundingSphereDebugMarker->markers[1].header.stamp);
  EXPECT_NEAR(0.1*sqrt(3), boundingSphereDebugMarker->markers[1].scale.x, 1e-5);
  EXPECT_NEAR(0.1*sqrt(3), boundingSphereDebugMarker->markers[1].scale.y, 1e-5);
  EXPECT_NEAR(0.1*sqrt(3), boundingSphereDebugMarker->markers[1].scale.z, 1e-5);
  EXPECT_NEAR(-1.5, boundingSphereDebugMarker->markers[1].pose.position.x, 1e-5);
  EXPECT_NEAR(0, boundingSphereDebugMarker->markers[1].pose.position.y, 1e-6);
  EXPECT_NEAR(0, boundingSphereDebugMarker->markers[1].pose.position.z, 1e-6);
  EXPECT_NEAR(0, boundingSphereDebugMarker->markers[1].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, boundingSphereDebugMarker->markers[1].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, boundingSphereDebugMarker->markers[1].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, boundingSphereDebugMarker->markers[1].pose.orientation.w, 1e-6);
  EXPECT_LT(0, boundingSphereDebugMarker->markers[1].color.a);
  // make sure the marker has some color
  EXPECT_LT(0, boundingSphereDebugMarker->markers[1].color.r + boundingSphereDebugMarker->markers[1].color.g +
      boundingSphereDebugMarker->markers[1].color.b);
  EXPECT_EQ(visualization_msgs::Marker::SPHERE, boundingSphereDebugMarker->markers[1].type);
  EXPECT_EQ(visualization_msgs::Marker::ADD, boundingSphereDebugMarker->markers[1].action);
  EXPECT_FALSE(boundingSphereDebugMarker->markers[1].ns.empty());
  EXPECT_EQ(1, boundingSphereDebugMarker->markers[1].frame_locked);
  
  EXPECT_EQ(4, boundingBoxDebugMarker->markers.size());
  EXPECT_EQ("odom", boundingBoxDebugMarker->markers[0].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, boundingBoxDebugMarker->markers[0].header.stamp);
  EXPECT_NEAR(2.75, boundingBoxDebugMarker->markers[0].scale.x, 1e-5);
  EXPECT_NEAR(2.75, boundingBoxDebugMarker->markers[0].scale.y, 1e-5);
  EXPECT_NEAR(2.75, boundingBoxDebugMarker->markers[0].scale.z, 1e-5);
  EXPECT_NEAR(0, boundingBoxDebugMarker->markers[0].pose.position.x, 1e-6);
  EXPECT_NEAR(0, boundingBoxDebugMarker->markers[0].pose.position.y, 1e-6);
  EXPECT_NEAR(0, boundingBoxDebugMarker->markers[0].pose.position.z, 1e-6);
  EXPECT_NEAR(0, boundingBoxDebugMarker->markers[0].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, boundingBoxDebugMarker->markers[0].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, boundingBoxDebugMarker->markers[0].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, boundingBoxDebugMarker->markers[0].pose.orientation.w, 1e-6);
  EXPECT_LT(0, boundingBoxDebugMarker->markers[0].color.a);
  // make sure the marker has some color
  EXPECT_LT(0, boundingBoxDebugMarker->markers[0].color.r + boundingBoxDebugMarker->markers[0].color.g +
      boundingBoxDebugMarker->markers[0].color.b);
  EXPECT_EQ(visualization_msgs::Marker::CUBE, boundingBoxDebugMarker->markers[0].type);
  EXPECT_EQ(visualization_msgs::Marker::ADD, boundingBoxDebugMarker->markers[0].action);
  EXPECT_FALSE(boundingBoxDebugMarker->markers[0].ns.empty());
  EXPECT_EQ(1, boundingBoxDebugMarker->markers[0].frame_locked);
  EXPECT_EQ("odom", boundingBoxDebugMarker->markers[1].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, boundingBoxDebugMarker->markers[1].header.stamp);
  EXPECT_NEAR(1.84, boundingBoxDebugMarker->markers[1].scale.x, 1e-5);
  EXPECT_NEAR(1.84, boundingBoxDebugMarker->markers[1].scale.y, 1e-5);
  EXPECT_NEAR(1.84, boundingBoxDebugMarker->markers[1].scale.z, 1e-5);
  EXPECT_NEAR(0, boundingBoxDebugMarker->markers[1].pose.position.x, 1e-6);
  EXPECT_NEAR(0, boundingBoxDebugMarker->markers[1].pose.position.y, 1e-6);
  EXPECT_NEAR(0, boundingBoxDebugMarker->markers[1].pose.position.z, 1e-6);
  EXPECT_NEAR(0, boundingBoxDebugMarker->markers[1].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, boundingBoxDebugMarker->markers[1].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, boundingBoxDebugMarker->markers[1].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, boundingBoxDebugMarker->markers[1].pose.orientation.w, 1e-6);
  EXPECT_LT(0, boundingBoxDebugMarker->markers[1].color.a);
  // make sure the marker has some color
  EXPECT_LT(0, boundingBoxDebugMarker->markers[1].color.r + boundingBoxDebugMarker->markers[1].color.g +
      boundingBoxDebugMarker->markers[1].color.b);
  EXPECT_EQ(visualization_msgs::Marker::CUBE, boundingBoxDebugMarker->markers[1].type);
  EXPECT_EQ(visualization_msgs::Marker::ADD, boundingBoxDebugMarker->markers[1].action);
  EXPECT_FALSE(boundingBoxDebugMarker->markers[1].ns.empty());
  EXPECT_EQ(1, boundingBoxDebugMarker->markers[1].frame_locked);
  EXPECT_EQ("odom", boundingBoxDebugMarker->markers[2].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, boundingBoxDebugMarker->markers[2].header.stamp);
  EXPECT_NEAR(2.5, boundingBoxDebugMarker->markers[2].scale.x, 1e-4);
  EXPECT_NEAR(2.5, boundingBoxDebugMarker->markers[2].scale.y, 1e-4);
  EXPECT_NEAR(2.5, boundingBoxDebugMarker->markers[2].scale.z, 1e-4);
  EXPECT_NEAR(0.122 - 0.1, boundingBoxDebugMarker->markers[2].pose.position.x, 1e-6);
  EXPECT_NEAR(0, boundingBoxDebugMarker->markers[2].pose.position.y, 1e-6);
  EXPECT_NEAR(0, boundingBoxDebugMarker->markers[2].pose.position.z, 1e-6);
  EXPECT_NEAR(0, boundingBoxDebugMarker->markers[2].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, boundingBoxDebugMarker->markers[2].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, boundingBoxDebugMarker->markers[2].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, boundingBoxDebugMarker->markers[2].pose.orientation.w, 1e-6);
  EXPECT_LT(0, boundingBoxDebugMarker->markers[2].color.a);
  // make sure the marker has some color
  EXPECT_LT(0, boundingBoxDebugMarker->markers[2].color.r + boundingBoxDebugMarker->markers[2].color.g +
      boundingBoxDebugMarker->markers[2].color.b);
  EXPECT_EQ(visualization_msgs::Marker::CUBE, boundingBoxDebugMarker->markers[2].type);
  EXPECT_EQ(visualization_msgs::Marker::ADD, boundingBoxDebugMarker->markers[2].action);
  EXPECT_FALSE(boundingBoxDebugMarker->markers[2].ns.empty());
  EXPECT_EQ(1, boundingBoxDebugMarker->markers[2].frame_locked);
  EXPECT_EQ("odom", boundingBoxDebugMarker->markers[3].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, boundingBoxDebugMarker->markers[3].header.stamp);
  EXPECT_NEAR(0.1, boundingBoxDebugMarker->markers[3].scale.x, 1e-5);
  EXPECT_NEAR(0.1, boundingBoxDebugMarker->markers[3].scale.y, 1e-5);
  EXPECT_NEAR(0.1, boundingBoxDebugMarker->markers[3].scale.z, 1e-5);
  EXPECT_NEAR(-1.5, boundingBoxDebugMarker->markers[3].pose.position.x, 1e-6);
  EXPECT_NEAR(0, boundingBoxDebugMarker->markers[3].pose.position.y, 1e-6);
  EXPECT_NEAR(0, boundingBoxDebugMarker->markers[3].pose.position.z, 1e-6);
  EXPECT_NEAR(0, boundingBoxDebugMarker->markers[3].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, boundingBoxDebugMarker->markers[3].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, boundingBoxDebugMarker->markers[3].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, boundingBoxDebugMarker->markers[3].pose.orientation.w, 1e-6);
  EXPECT_LT(0, boundingBoxDebugMarker->markers[3].color.a);
  // make sure the marker has some color
  EXPECT_LT(0, boundingBoxDebugMarker->markers[3].color.r + boundingBoxDebugMarker->markers[3].color.g +
      boundingBoxDebugMarker->markers[3].color.b);
  EXPECT_EQ(visualization_msgs::Marker::CUBE, boundingBoxDebugMarker->markers[3].type);
  EXPECT_EQ(visualization_msgs::Marker::ADD, boundingBoxDebugMarker->markers[3].action);
  EXPECT_FALSE(boundingBoxDebugMarker->markers[3].ns.empty());
  EXPECT_EQ(1, boundingBoxDebugMarker->markers[3].frame_locked);

  EXPECT_EQ(4, orientedBoundingBoxDebugMarker->markers.size());
  EXPECT_EQ("odom", orientedBoundingBoxDebugMarker->markers[0].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, orientedBoundingBoxDebugMarker->markers[0].header.stamp);
  EXPECT_NEAR(2.75, orientedBoundingBoxDebugMarker->markers[0].scale.x, 1e-5);
  EXPECT_NEAR(2.75, orientedBoundingBoxDebugMarker->markers[0].scale.y, 1e-5);
  EXPECT_NEAR(2.75, orientedBoundingBoxDebugMarker->markers[0].scale.z, 1e-5);
  EXPECT_NEAR(0, orientedBoundingBoxDebugMarker->markers[0].pose.position.x, 1e-6);
  EXPECT_NEAR(0, orientedBoundingBoxDebugMarker->markers[0].pose.position.y, 1e-6);
  EXPECT_NEAR(0, orientedBoundingBoxDebugMarker->markers[0].pose.position.z, 1e-6);
  EXPECT_NEAR(0, orientedBoundingBoxDebugMarker->markers[0].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, orientedBoundingBoxDebugMarker->markers[0].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, orientedBoundingBoxDebugMarker->markers[0].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, orientedBoundingBoxDebugMarker->markers[0].pose.orientation.w, 1e-6);
  EXPECT_LT(0, orientedBoundingBoxDebugMarker->markers[0].color.a);
  // make sure the marker has some color
  EXPECT_LT(0, orientedBoundingBoxDebugMarker->markers[0].color.r + orientedBoundingBoxDebugMarker->markers[0].color.g +
      orientedBoundingBoxDebugMarker->markers[0].color.b);
  EXPECT_EQ(visualization_msgs::Marker::CUBE, orientedBoundingBoxDebugMarker->markers[0].type);
  EXPECT_EQ(visualization_msgs::Marker::ADD, orientedBoundingBoxDebugMarker->markers[0].action);
  EXPECT_FALSE(orientedBoundingBoxDebugMarker->markers[0].ns.empty());
  EXPECT_EQ(1, orientedBoundingBoxDebugMarker->markers[0].frame_locked);
  EXPECT_EQ("odom", orientedBoundingBoxDebugMarker->markers[1].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, orientedBoundingBoxDebugMarker->markers[1].header.stamp);
  EXPECT_NEAR(1.84, orientedBoundingBoxDebugMarker->markers[1].scale.x, 1e-5);
  EXPECT_NEAR(1.84, orientedBoundingBoxDebugMarker->markers[1].scale.y, 1e-5);
  EXPECT_NEAR(1.84, orientedBoundingBoxDebugMarker->markers[1].scale.z, 1e-5);
  EXPECT_NEAR(0, orientedBoundingBoxDebugMarker->markers[1].pose.position.x, 1e-6);
  EXPECT_NEAR(0, orientedBoundingBoxDebugMarker->markers[1].pose.position.y, 1e-6);
  EXPECT_NEAR(0, orientedBoundingBoxDebugMarker->markers[1].pose.position.z, 1e-6);
  EXPECT_NEAR(0, orientedBoundingBoxDebugMarker->markers[1].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, orientedBoundingBoxDebugMarker->markers[1].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, orientedBoundingBoxDebugMarker->markers[1].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, orientedBoundingBoxDebugMarker->markers[1].pose.orientation.w, 1e-6);
  EXPECT_LT(0, orientedBoundingBoxDebugMarker->markers[1].color.a);
  // make sure the marker has some color
  EXPECT_LT(0, orientedBoundingBoxDebugMarker->markers[1].color.r + orientedBoundingBoxDebugMarker->markers[1].color.g +
      orientedBoundingBoxDebugMarker->markers[1].color.b);
  EXPECT_EQ(visualization_msgs::Marker::CUBE, orientedBoundingBoxDebugMarker->markers[1].type);
  EXPECT_EQ(visualization_msgs::Marker::ADD, orientedBoundingBoxDebugMarker->markers[1].action);
  EXPECT_FALSE(orientedBoundingBoxDebugMarker->markers[1].ns.empty());
  EXPECT_EQ(1, orientedBoundingBoxDebugMarker->markers[1].frame_locked);
  EXPECT_EQ("odom", orientedBoundingBoxDebugMarker->markers[2].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, orientedBoundingBoxDebugMarker->markers[2].header.stamp);
  EXPECT_NEAR(2.5, orientedBoundingBoxDebugMarker->markers[2].scale.x, 1e-4);
  EXPECT_NEAR(2.5, orientedBoundingBoxDebugMarker->markers[2].scale.y, 1e-4);
  EXPECT_NEAR(2.5, orientedBoundingBoxDebugMarker->markers[2].scale.z, 1e-4);
  EXPECT_NEAR(0.122 - 0.1, orientedBoundingBoxDebugMarker->markers[2].pose.position.x, 1e-6);
  EXPECT_NEAR(0, orientedBoundingBoxDebugMarker->markers[2].pose.position.y, 1e-6);
  EXPECT_NEAR(0, orientedBoundingBoxDebugMarker->markers[2].pose.position.z, 1e-6);
  EXPECT_NEAR(0, orientedBoundingBoxDebugMarker->markers[2].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, orientedBoundingBoxDebugMarker->markers[2].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, orientedBoundingBoxDebugMarker->markers[2].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, orientedBoundingBoxDebugMarker->markers[2].pose.orientation.w, 1e-6);
  EXPECT_LT(0, orientedBoundingBoxDebugMarker->markers[2].color.a);
  // make sure the marker has some color
  EXPECT_LT(0, orientedBoundingBoxDebugMarker->markers[2].color.r + orientedBoundingBoxDebugMarker->markers[2].color.g +
      orientedBoundingBoxDebugMarker->markers[2].color.b);
  EXPECT_EQ(visualization_msgs::Marker::CUBE, orientedBoundingBoxDebugMarker->markers[2].type);
  EXPECT_EQ(visualization_msgs::Marker::ADD, orientedBoundingBoxDebugMarker->markers[2].action);
  EXPECT_FALSE(orientedBoundingBoxDebugMarker->markers[2].ns.empty());
  EXPECT_EQ(1, orientedBoundingBoxDebugMarker->markers[2].frame_locked);
  EXPECT_EQ("odom", orientedBoundingBoxDebugMarker->markers[3].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, orientedBoundingBoxDebugMarker->markers[3].header.stamp);
  EXPECT_NEAR(0.1, orientedBoundingBoxDebugMarker->markers[3].scale.x, 1e-5);
  EXPECT_NEAR(0.1, orientedBoundingBoxDebugMarker->markers[3].scale.y, 1e-5);
  EXPECT_NEAR(0.1, orientedBoundingBoxDebugMarker->markers[3].scale.z, 1e-5);
  EXPECT_NEAR(-1.5, orientedBoundingBoxDebugMarker->markers[3].pose.position.x, 1e-6);
  EXPECT_NEAR(0, orientedBoundingBoxDebugMarker->markers[3].pose.position.y, 1e-6);
  EXPECT_NEAR(0, orientedBoundingBoxDebugMarker->markers[3].pose.position.z, 1e-6);
  EXPECT_NEAR(0, orientedBoundingBoxDebugMarker->markers[3].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, orientedBoundingBoxDebugMarker->markers[3].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, orientedBoundingBoxDebugMarker->markers[3].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, orientedBoundingBoxDebugMarker->markers[3].pose.orientation.w, 1e-6);
  EXPECT_LT(0, orientedBoundingBoxDebugMarker->markers[3].color.a);
  // make sure the marker has some color
  EXPECT_LT(0, orientedBoundingBoxDebugMarker->markers[3].color.r + orientedBoundingBoxDebugMarker->markers[3].color.g +
      orientedBoundingBoxDebugMarker->markers[3].color.b);
  EXPECT_EQ(visualization_msgs::Marker::CUBE, orientedBoundingBoxDebugMarker->markers[3].type);
  EXPECT_EQ(visualization_msgs::Marker::ADD, orientedBoundingBoxDebugMarker->markers[3].action);
  EXPECT_FALSE(orientedBoundingBoxDebugMarker->markers[3].ns.empty());
  EXPECT_EQ(1, orientedBoundingBoxDebugMarker->markers[3].frame_locked);

  EXPECT_EQ(4, localBoundingBoxDebugMarker->markers.size());
  EXPECT_EQ("base_link", localBoundingBoxDebugMarker->markers[0].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, localBoundingBoxDebugMarker->markers[0].header.stamp);
  EXPECT_NEAR(2.75, localBoundingBoxDebugMarker->markers[0].scale.x, 1e-5);
  EXPECT_NEAR(2.75, localBoundingBoxDebugMarker->markers[0].scale.y, 1e-5);
  EXPECT_NEAR(2.75, localBoundingBoxDebugMarker->markers[0].scale.z, 1e-5);
  EXPECT_NEAR(-0.122, localBoundingBoxDebugMarker->markers[0].pose.position.x, 1e-6);
  EXPECT_NEAR(0, localBoundingBoxDebugMarker->markers[0].pose.position.y, 1e-6);
  EXPECT_NEAR(0, localBoundingBoxDebugMarker->markers[0].pose.position.z, 1e-6);
  EXPECT_NEAR(0, localBoundingBoxDebugMarker->markers[0].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, localBoundingBoxDebugMarker->markers[0].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, localBoundingBoxDebugMarker->markers[0].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, localBoundingBoxDebugMarker->markers[0].pose.orientation.w, 1e-6);
  EXPECT_LT(0, localBoundingBoxDebugMarker->markers[0].color.a);
  // make sure the marker has some color
  EXPECT_LT(0, localBoundingBoxDebugMarker->markers[0].color.r + localBoundingBoxDebugMarker->markers[0].color.g +
      localBoundingBoxDebugMarker->markers[0].color.b);
  EXPECT_EQ(visualization_msgs::Marker::CUBE, localBoundingBoxDebugMarker->markers[0].type);
  EXPECT_EQ(visualization_msgs::Marker::ADD, localBoundingBoxDebugMarker->markers[0].action);
  EXPECT_FALSE(localBoundingBoxDebugMarker->markers[0].ns.empty());
  EXPECT_EQ(1, localBoundingBoxDebugMarker->markers[0].frame_locked);
  EXPECT_EQ("base_link", localBoundingBoxDebugMarker->markers[1].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, localBoundingBoxDebugMarker->markers[1].header.stamp);
  EXPECT_NEAR(1.84, localBoundingBoxDebugMarker->markers[1].scale.x, 1e-5);
  EXPECT_NEAR(1.84, localBoundingBoxDebugMarker->markers[1].scale.y, 1e-5);
  EXPECT_NEAR(1.84, localBoundingBoxDebugMarker->markers[1].scale.z, 1e-5);
  EXPECT_NEAR(-0.122, localBoundingBoxDebugMarker->markers[1].pose.position.x, 1e-6);
  EXPECT_NEAR(0, localBoundingBoxDebugMarker->markers[1].pose.position.y, 1e-6);
  EXPECT_NEAR(0, localBoundingBoxDebugMarker->markers[1].pose.position.z, 1e-6);
  EXPECT_NEAR(0, localBoundingBoxDebugMarker->markers[1].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, localBoundingBoxDebugMarker->markers[1].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, localBoundingBoxDebugMarker->markers[1].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, localBoundingBoxDebugMarker->markers[1].pose.orientation.w, 1e-6);
  EXPECT_LT(0, localBoundingBoxDebugMarker->markers[1].color.a);
  // make sure the marker has some color
  EXPECT_LT(0, localBoundingBoxDebugMarker->markers[1].color.r + localBoundingBoxDebugMarker->markers[1].color.g +
      localBoundingBoxDebugMarker->markers[1].color.b);
  EXPECT_EQ(visualization_msgs::Marker::CUBE, localBoundingBoxDebugMarker->markers[1].type);
  EXPECT_EQ(visualization_msgs::Marker::ADD, localBoundingBoxDebugMarker->markers[1].action);
  EXPECT_FALSE(localBoundingBoxDebugMarker->markers[1].ns.empty());
  EXPECT_EQ(1, localBoundingBoxDebugMarker->markers[1].frame_locked);
  EXPECT_EQ("base_link", localBoundingBoxDebugMarker->markers[2].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, localBoundingBoxDebugMarker->markers[2].header.stamp);
  EXPECT_NEAR(2.5, localBoundingBoxDebugMarker->markers[2].scale.x, 1e-4);
  EXPECT_NEAR(2.5, localBoundingBoxDebugMarker->markers[2].scale.y, 1e-4);
  EXPECT_NEAR(2.5, localBoundingBoxDebugMarker->markers[2].scale.z, 1e-4);
  EXPECT_NEAR(-0.1, localBoundingBoxDebugMarker->markers[2].pose.position.x, 1e-6);
  EXPECT_NEAR(0, localBoundingBoxDebugMarker->markers[2].pose.position.y, 1e-6);
  EXPECT_NEAR(0, localBoundingBoxDebugMarker->markers[2].pose.position.z, 1e-6);
  EXPECT_NEAR(0, localBoundingBoxDebugMarker->markers[2].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, localBoundingBoxDebugMarker->markers[2].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, localBoundingBoxDebugMarker->markers[2].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, localBoundingBoxDebugMarker->markers[2].pose.orientation.w, 1e-6);
  EXPECT_LT(0, localBoundingBoxDebugMarker->markers[2].color.a);
  // make sure the marker has some color
  EXPECT_LT(0, localBoundingBoxDebugMarker->markers[2].color.r + localBoundingBoxDebugMarker->markers[2].color.g +
      localBoundingBoxDebugMarker->markers[2].color.b);
  EXPECT_EQ(visualization_msgs::Marker::CUBE, localBoundingBoxDebugMarker->markers[2].type);
  EXPECT_EQ(visualization_msgs::Marker::ADD, localBoundingBoxDebugMarker->markers[2].action);
  EXPECT_FALSE(localBoundingBoxDebugMarker->markers[2].ns.empty());
  EXPECT_EQ(1, localBoundingBoxDebugMarker->markers[2].frame_locked);
  EXPECT_EQ("base_link", localBoundingBoxDebugMarker->markers[3].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, localBoundingBoxDebugMarker->markers[3].header.stamp);
  EXPECT_NEAR(0.1, localBoundingBoxDebugMarker->markers[3].scale.x, 1e-5);
  EXPECT_NEAR(0.1, localBoundingBoxDebugMarker->markers[3].scale.y, 1e-5);
  EXPECT_NEAR(0.1, localBoundingBoxDebugMarker->markers[3].scale.z, 1e-5);
  EXPECT_NEAR(-1.5-0.122, localBoundingBoxDebugMarker->markers[3].pose.position.x, 1e-6);
  EXPECT_NEAR(0, localBoundingBoxDebugMarker->markers[3].pose.position.y, 1e-6);
  EXPECT_NEAR(0, localBoundingBoxDebugMarker->markers[3].pose.position.z, 1e-6);
  EXPECT_NEAR(0, localBoundingBoxDebugMarker->markers[3].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, localBoundingBoxDebugMarker->markers[3].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, localBoundingBoxDebugMarker->markers[3].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, localBoundingBoxDebugMarker->markers[3].pose.orientation.w, 1e-6);
  EXPECT_LT(0, localBoundingBoxDebugMarker->markers[3].color.a);
  // make sure the marker has some color
  EXPECT_LT(0, localBoundingBoxDebugMarker->markers[3].color.r + localBoundingBoxDebugMarker->markers[3].color.g +
      localBoundingBoxDebugMarker->markers[3].color.b);
  EXPECT_EQ(visualization_msgs::Marker::CUBE, localBoundingBoxDebugMarker->markers[3].type);
  EXPECT_EQ(visualization_msgs::Marker::ADD, localBoundingBoxDebugMarker->markers[3].action);
  EXPECT_FALSE(localBoundingBoxDebugMarker->markers[3].ns.empty());
  EXPECT_EQ(1, localBoundingBoxDebugMarker->markers[3].frame_locked);

  EXPECT_EQ(4, robotModelContainsTest->markers.size());
  EXPECT_EQ("odom", robotModelContainsTest->markers[0].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, robotModelContainsTest->markers[0].header.stamp);
  EXPECT_NEAR(2.75, robotModelContainsTest->markers[0].scale.x, 1e-5);
  EXPECT_NEAR(2.75, robotModelContainsTest->markers[0].scale.y, 1e-5);
  EXPECT_NEAR(2.75, robotModelContainsTest->markers[0].scale.z, 1e-5);
  EXPECT_NEAR(0, robotModelContainsTest->markers[0].pose.position.x, 1e-6);
  EXPECT_NEAR(0, robotModelContainsTest->markers[0].pose.position.y, 1e-6);
  EXPECT_NEAR(0, robotModelContainsTest->markers[0].pose.position.z, 1e-6);
  EXPECT_NEAR(0, robotModelContainsTest->markers[0].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, robotModelContainsTest->markers[0].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, robotModelContainsTest->markers[0].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, robotModelContainsTest->markers[0].pose.orientation.w, 1e-6);
  EXPECT_LT(0, robotModelContainsTest->markers[0].color.a);
  // make sure the marker has some color
  EXPECT_LT(0, robotModelContainsTest->markers[0].color.r + robotModelContainsTest->markers[0].color.g +
      robotModelContainsTest->markers[0].color.b);
  EXPECT_EQ(visualization_msgs::Marker::SPHERE, robotModelContainsTest->markers[0].type);
  EXPECT_EQ(visualization_msgs::Marker::ADD, robotModelContainsTest->markers[0].action);
  EXPECT_EQ("antenna-0", robotModelContainsTest->markers[0].ns);
  EXPECT_EQ(1, robotModelContainsTest->markers[0].frame_locked);
  EXPECT_EQ("odom", robotModelContainsTest->markers[1].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, robotModelContainsTest->markers[1].header.stamp);
  EXPECT_NEAR(1.84, robotModelContainsTest->markers[1].scale.x, 1e-5);
  EXPECT_NEAR(1.84, robotModelContainsTest->markers[1].scale.y, 1e-5);
  EXPECT_NEAR(1.84, robotModelContainsTest->markers[1].scale.z, 1e-5);
  EXPECT_NEAR(0, robotModelContainsTest->markers[1].pose.position.x, 1e-6);
  EXPECT_NEAR(0, robotModelContainsTest->markers[1].pose.position.y, 1e-6);
  EXPECT_NEAR(0, robotModelContainsTest->markers[1].pose.position.z, 1e-6);
  EXPECT_NEAR(0, robotModelContainsTest->markers[1].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, robotModelContainsTest->markers[1].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, robotModelContainsTest->markers[1].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, robotModelContainsTest->markers[1].pose.orientation.w, 1e-6);
  EXPECT_LT(0, robotModelContainsTest->markers[1].color.a);
  // make sure the marker has some color
  EXPECT_LT(0, robotModelContainsTest->markers[1].color.r + robotModelContainsTest->markers[1].color.g +
      robotModelContainsTest->markers[1].color.b);
  EXPECT_EQ(visualization_msgs::Marker::CUBE, robotModelContainsTest->markers[1].type);
  EXPECT_EQ(visualization_msgs::Marker::ADD, robotModelContainsTest->markers[1].action);
  EXPECT_EQ("base_link-0", robotModelContainsTest->markers[1].ns);
  EXPECT_EQ(1, robotModelContainsTest->markers[1].frame_locked);
  EXPECT_EQ("odom", robotModelContainsTest->markers[2].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, robotModelContainsTest->markers[2].header.stamp);
  EXPECT_NEAR(2.5, robotModelContainsTest->markers[2].scale.x, 1e-4);
  EXPECT_NEAR(2.5, robotModelContainsTest->markers[2].scale.y, 1e-4);
  EXPECT_NEAR(2.5, robotModelContainsTest->markers[2].scale.z, 1e-4);
  EXPECT_NEAR(0.122-0.1, robotModelContainsTest->markers[2].pose.position.x, 1e-6);
  EXPECT_NEAR(0, robotModelContainsTest->markers[2].pose.position.y, 1e-6);
  EXPECT_NEAR(0, robotModelContainsTest->markers[2].pose.position.z, 1e-6);
  EXPECT_NEAR(0, robotModelContainsTest->markers[2].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, robotModelContainsTest->markers[2].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, robotModelContainsTest->markers[2].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, robotModelContainsTest->markers[2].pose.orientation.w, 1e-6);
  EXPECT_LT(0, robotModelContainsTest->markers[2].color.a);
  // make sure the marker has some color
  EXPECT_LT(0, robotModelContainsTest->markers[2].color.r + robotModelContainsTest->markers[2].color.g +
      robotModelContainsTest->markers[2].color.b);
  EXPECT_EQ(visualization_msgs::Marker::CUBE, robotModelContainsTest->markers[2].type);
  EXPECT_EQ(visualization_msgs::Marker::ADD, robotModelContainsTest->markers[2].action);
  EXPECT_EQ("base_link-1", robotModelContainsTest->markers[2].ns);
  EXPECT_EQ(1, robotModelContainsTest->markers[2].frame_locked);
  EXPECT_EQ("odom", robotModelContainsTest->markers[3].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, robotModelContainsTest->markers[3].header.stamp);
  EXPECT_NEAR(0.1, robotModelContainsTest->markers[3].scale.x, 1e-5);
  EXPECT_NEAR(0.1, robotModelContainsTest->markers[3].scale.y, 1e-5);
  EXPECT_NEAR(0.1, robotModelContainsTest->markers[3].scale.z, 1e-5);
  EXPECT_NEAR(-1.5, robotModelContainsTest->markers[3].pose.position.x, 1e-6);
  EXPECT_NEAR(0, robotModelContainsTest->markers[3].pose.position.y, 1e-6);
  EXPECT_NEAR(0, robotModelContainsTest->markers[3].pose.position.z, 1e-6);
  EXPECT_NEAR(0, robotModelContainsTest->markers[3].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, robotModelContainsTest->markers[3].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, robotModelContainsTest->markers[3].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, robotModelContainsTest->markers[3].pose.orientation.w, 1e-6);
  EXPECT_LT(0, robotModelContainsTest->markers[3].color.a);
  // make sure the marker has some color
  EXPECT_LT(0, robotModelContainsTest->markers[3].color.r + robotModelContainsTest->markers[3].color.g +
      robotModelContainsTest->markers[3].color.b);
  EXPECT_EQ(visualization_msgs::Marker::CUBE, robotModelContainsTest->markers[3].type);
  EXPECT_EQ(visualization_msgs::Marker::ADD, robotModelContainsTest->markers[3].action);
  EXPECT_EQ("laser-0", robotModelContainsTest->markers[3].ns);
  EXPECT_EQ(1, robotModelContainsTest->markers[3].frame_locked);

  EXPECT_EQ(2, robotModelShadowTest->markers.size());
  EXPECT_EQ("odom", robotModelShadowTest->markers[0].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, robotModelShadowTest->markers[0].header.stamp);
  EXPECT_NEAR(2.75, robotModelShadowTest->markers[0].scale.x, 1e-5);
  EXPECT_NEAR(2.75, robotModelShadowTest->markers[0].scale.y, 1e-5);
  EXPECT_NEAR(2.75, robotModelShadowTest->markers[0].scale.z, 1e-5);
  EXPECT_NEAR(0, robotModelShadowTest->markers[0].pose.position.x, 1e-6);
  EXPECT_NEAR(0, robotModelShadowTest->markers[0].pose.position.y, 1e-6);
  EXPECT_NEAR(0, robotModelShadowTest->markers[0].pose.position.z, 1e-6);
  EXPECT_NEAR(0, robotModelShadowTest->markers[0].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, robotModelShadowTest->markers[0].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, robotModelShadowTest->markers[0].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, robotModelShadowTest->markers[0].pose.orientation.w, 1e-6);
  EXPECT_LT(0, robotModelShadowTest->markers[0].color.a);
  // make sure the marker has some color
  EXPECT_LT(0, robotModelShadowTest->markers[0].color.r + robotModelShadowTest->markers[0].color.g +
      robotModelShadowTest->markers[0].color.b);
  EXPECT_EQ(visualization_msgs::Marker::SPHERE, robotModelShadowTest->markers[0].type);
  EXPECT_EQ(visualization_msgs::Marker::ADD, robotModelShadowTest->markers[0].action);
  EXPECT_EQ("antenna-0", robotModelShadowTest->markers[0].ns);
  EXPECT_EQ(1, robotModelShadowTest->markers[0].frame_locked);
  EXPECT_EQ("odom", robotModelShadowTest->markers[1].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, robotModelShadowTest->markers[1].header.stamp);
  EXPECT_NEAR(2, robotModelShadowTest->markers[1].scale.x, 1e-5);
  EXPECT_NEAR(2, robotModelShadowTest->markers[1].scale.y, 1e-5);
  EXPECT_NEAR(2, robotModelShadowTest->markers[1].scale.z, 1e-5);
  EXPECT_NEAR(0, robotModelShadowTest->markers[1].pose.position.x, 1e-6);
  EXPECT_NEAR(0, robotModelShadowTest->markers[1].pose.position.y, 1e-6);
  EXPECT_NEAR(0, robotModelShadowTest->markers[1].pose.position.z, 1e-6);
  EXPECT_NEAR(0, robotModelShadowTest->markers[1].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, robotModelShadowTest->markers[1].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, robotModelShadowTest->markers[1].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, robotModelShadowTest->markers[1].pose.orientation.w, 1e-6);
  EXPECT_LT(0, robotModelShadowTest->markers[1].color.a);
  // make sure the marker has some color
  EXPECT_LT(0, robotModelShadowTest->markers[1].color.r + robotModelShadowTest->markers[1].color.g +
      robotModelShadowTest->markers[1].color.b);
  EXPECT_EQ(visualization_msgs::Marker::CUBE, robotModelShadowTest->markers[1].type);
  EXPECT_EQ(visualization_msgs::Marker::ADD, robotModelShadowTest->markers[1].action);
  EXPECT_EQ("base_link-0", robotModelShadowTest->markers[1].ns);
  EXPECT_EQ(1, robotModelShadowTest->markers[1].frame_locked);
}

TEST(RobotBodyFilter, ComputeMaskAllAtOnce)
{
  ros::NodeHandle nh;

  auto filter = std::make_shared<RobotBodyFilterPointCloud2Test>();
  auto filterBase = std::dynamic_pointer_cast<filters::FilterBase<sensor_msgs::PointCloud2>>(filter);

  nh.setParam("test_robot_description", ROBOT_URDF);
  filterBase->configure("compute_mask_config_all_at_once", nh);

  Cloud cloud;
  cloud.header.frame_id = filter->filteringFrame;
  CloudModifier mod(cloud);
  mod.setPointCloud2Fields(3,
                           "x", 1, sensor_msgs::PointField::FLOAT32,
                           "y", 1, sensor_msgs::PointField::FLOAT32,
                           "z", 1, sensor_msgs::PointField::FLOAT32);
  mod.resize(12);
  cloud.width = 4;
  cloud.height = 3;
  cloud.row_step = cloud.width * cloud.point_step;

  {
    CloudIter x_it(cloud, "x");
    CloudIter y_it(cloud, "y");
    CloudIter z_it(cloud, "z");

    *x_it = 1.5 + -1.5; *y_it = 0; *z_it = 0; ++x_it, ++y_it, ++z_it; // pointSensor
    *x_it = 1.5 + -1.47; *y_it = 0; *z_it = 0; ++x_it, ++y_it, ++z_it; // pointSensor2
    *x_it = 1.5 + -1.42; *y_it = 0; *z_it = 0; ++x_it, ++y_it, ++z_it; // pointClipMin
    *x_it = 1.5 + 10; *y_it = 0; *z_it = 0; ++x_it, ++y_it, ++z_it; // pointClipMax
    *x_it = 1.5 + 0.85; *y_it = 0.85; *z_it = 0.85; ++x_it, ++y_it, ++z_it; // pointInBox
    *x_it = 1.5 + 1.35; *y_it = 0; *z_it = 0; ++x_it, ++y_it, ++z_it; // pointInSphere
    *x_it = 1.5 + 0; *y_it = 0; *z_it = 0; ++x_it, ++y_it, ++z_it; // pointInBoth
    *x_it = 1.5 + -0.25; *y_it = -2; *z_it = 2; ++x_it, ++y_it, ++z_it; // pointShadowBox
    *x_it = 1.5 + -0.560762; *y_it = 0; *z_it = 1.83871; ++x_it, ++y_it, ++z_it; // pointShadowSphere
    *x_it = 1.5 + 1.5; *y_it = 0; *z_it = 0; ++x_it, ++y_it, ++z_it; // pointShadowBoth
    *x_it = 1.5 + -3; *y_it = 0; *z_it = 0; ++x_it, ++y_it, ++z_it; // pointOutside
    *x_it = 1.5 + -4; *y_it = 0; *z_it = 0; ++x_it, ++y_it, ++z_it; // pointOutside2
  }

  {
    ros::Time now = ros::Time::now();
    cloud.header.stamp = now;
    geometry_msgs::TransformStamped tf;
    tf.transform.rotation.w = 1.0;
    for (double d = -5.0; d < 5.0; d += 0.1)
    {
      tf.header.stamp = now + ros::Duration(d);

      tf.transform.translation.x = 0.122;
      tf.header.frame_id = "odom";
      tf.child_frame_id = "base_link";
      filter->tfBuffer->setTransform(tf, "test");

      tf.transform.translation.x = -1.5 - 0.122;
      tf.header.frame_id = "base_link";
      tf.child_frame_id = "laser";
      filter->tfBuffer->setTransform(tf, "test");

      tf.transform.translation.x = 0.01864 - 0.122;
      tf.header.frame_id = "base_link";
      tf.child_frame_id = "antenna";
      filter->tfBuffer->setTransform(tf, "test");
    }
  }

  while (!filter->tfFramesWatchdog->isReachable("antenna"))
    ros::WallDuration(0.01).sleep();
  filter->updateTransformCache(cloud.header.stamp, cloud.header.stamp + ros::Duration(1));

  SphereStampedConstPtr boundingSphere;
  geometry_msgs::PolygonStampedConstPtr boundingBox;
  OrientedBoundingBoxStampedConstPtr orientedBoundingBox;
  geometry_msgs::PolygonStampedConstPtr localBoundingBox;
  visualization_msgs::MarkerConstPtr boundingSphereMarker;
  visualization_msgs::MarkerConstPtr boundingBoxMarker;
  visualization_msgs::MarkerConstPtr orientedBoundingBoxMarker;
  visualization_msgs::MarkerConstPtr localBoundingBoxMarker;
  visualization_msgs::MarkerArrayConstPtr boundingSphereDebugMarker;
  visualization_msgs::MarkerArrayConstPtr boundingBoxDebugMarker;
  visualization_msgs::MarkerArrayConstPtr orientedBoundingBoxDebugMarker;
  visualization_msgs::MarkerArrayConstPtr localBoundingBoxDebugMarker;
  sensor_msgs::PointCloud2ConstPtr pclNoBoundingSphere;
  sensor_msgs::PointCloud2ConstPtr pclNoBoundingBox;
  sensor_msgs::PointCloud2ConstPtr pclNoOrientedBoundingBox;
  sensor_msgs::PointCloud2ConstPtr pclNoLocalBoundingBox;
  sensor_msgs::PointCloud2ConstPtr pclInside;
  sensor_msgs::PointCloud2ConstPtr pclClip;
  sensor_msgs::PointCloud2ConstPtr pclShadow;
  visualization_msgs::MarkerArrayConstPtr robotModelContainsTest;
  visualization_msgs::MarkerArrayConstPtr robotModelShadowTest;

  ros::Subscriber boundingSphereSubscriber = nh.subscribe<SphereStamped>("/robot_bounding_sphere", 10, [&](const SphereStampedConstPtr& msg){boundingSphere=msg;});
  ros::Subscriber boundingBoxSubscriber = nh.subscribe<geometry_msgs::PolygonStamped>("/robot_bounding_box", 10, [&](const geometry_msgs::PolygonStampedConstPtr& msg){boundingBox=msg;});
  ros::Subscriber orientedBoundingBoxSubscriber = nh.subscribe<OrientedBoundingBoxStamped>("/robot_oriented_bounding_box", 10, [&](const OrientedBoundingBoxStampedConstPtr& msg){orientedBoundingBox=msg;});
  ros::Subscriber localBoundingBoxSubscriber = nh.subscribe<geometry_msgs::PolygonStamped>("/robot_local_bounding_box", 10, [&](const geometry_msgs::PolygonStampedConstPtr& msg){localBoundingBox=msg;});
  ros::Subscriber boundingSphereMarkerSubscriber = nh.subscribe<visualization_msgs::Marker>("/robot_bounding_sphere_marker", 10, [&](const visualization_msgs::MarkerConstPtr& msg){boundingSphereMarker=msg;});
  ros::Subscriber boundingBoxMarkerSubscriber = nh.subscribe<visualization_msgs::Marker>("/robot_bounding_box_marker", 10, [&](const visualization_msgs::MarkerConstPtr& msg){boundingBoxMarker=msg;});
  ros::Subscriber orientedBoundingBoxMarkerSubscriber = nh.subscribe<visualization_msgs::Marker>("/robot_oriented_bounding_box_marker", 10, [&](const visualization_msgs::MarkerConstPtr& msg){orientedBoundingBoxMarker=msg;});
  ros::Subscriber localBoundingBoxMarkerSubscriber = nh.subscribe<visualization_msgs::Marker>("/robot_local_bounding_box_marker", 10, [&](const visualization_msgs::MarkerConstPtr& msg){localBoundingBoxMarker=msg;});
  ros::Subscriber boundingSphereDebugMarkerSubscriber = nh.subscribe<visualization_msgs::MarkerArray>("/robot_bounding_sphere_debug", 10, [&](const visualization_msgs::MarkerArrayConstPtr& msg){boundingSphereDebugMarker=msg;});
  ros::Subscriber boundingBoxDebugMarkerSubscriber = nh.subscribe<visualization_msgs::MarkerArray>("/robot_bounding_box_debug", 10, [&](const visualization_msgs::MarkerArrayConstPtr& msg){boundingBoxDebugMarker=msg;});
  ros::Subscriber orientedBoundingBoxDebugMarkerSubscriber = nh.subscribe<visualization_msgs::MarkerArray>("/robot_oriented_bounding_box_debug", 10, [&](const visualization_msgs::MarkerArrayConstPtr& msg){orientedBoundingBoxDebugMarker=msg;});
  ros::Subscriber localBoundingBoxDebugMarkerSubscriber = nh.subscribe<visualization_msgs::MarkerArray>("/robot_local_bounding_box_debug", 10, [&](const visualization_msgs::MarkerArrayConstPtr& msg){localBoundingBoxDebugMarker=msg;});
  ros::Subscriber scanPointCloudNoBoundingSphereSubscriber = nh.subscribe<sensor_msgs::PointCloud2>("/scan_point_cloud_no_bsphere", 10, [&](const sensor_msgs::PointCloud2ConstPtr& msg){pclNoBoundingSphere=msg;});
  ros::Subscriber scanPointCloudNoBoundingBoxSubscriber = nh.subscribe<sensor_msgs::PointCloud2>("/scan_point_cloud_no_bbox", 10, [&](const sensor_msgs::PointCloud2ConstPtr& msg){pclNoBoundingBox=msg;});
  ros::Subscriber scanPointCloudNoOrientedBoundingBoxSubscriber = nh.subscribe<sensor_msgs::PointCloud2>("/scan_point_cloud_no_oriented_bbox", 10, [&](const sensor_msgs::PointCloud2ConstPtr& msg){pclNoOrientedBoundingBox=msg;});
  ros::Subscriber scanPointCloudNoLocalBoundingBoxSubscriber = nh.subscribe<sensor_msgs::PointCloud2>("/scan_point_cloud_no_local_bbox", 10, [&](const sensor_msgs::PointCloud2ConstPtr& msg){pclNoLocalBoundingBox=msg;});
  ros::Subscriber debugPointCloudInsideSubscriber = nh.subscribe<sensor_msgs::PointCloud2>("/scan_point_cloud_inside", 10, [&](const sensor_msgs::PointCloud2ConstPtr& msg){pclInside=msg;});
  ros::Subscriber debugPointCloudClipSubscriber = nh.subscribe<sensor_msgs::PointCloud2>("/scan_point_cloud_clip", 10, [&](const sensor_msgs::PointCloud2ConstPtr& msg){pclClip=msg;});
  ros::Subscriber debugPointCloudShadowSubscriber = nh.subscribe<sensor_msgs::PointCloud2>("/scan_point_cloud_shadow", 10, [&](const sensor_msgs::PointCloud2ConstPtr& msg){pclShadow=msg;});
  ros::Subscriber debugContainsMarkerSubscriber = nh.subscribe<visualization_msgs::MarkerArray>("/robot_model_for_contains_test", 10, [&](const visualization_msgs::MarkerArrayConstPtr& msg){robotModelContainsTest=msg;});
  ros::Subscriber debugShadowMarkerSubscriber = nh.subscribe<visualization_msgs::MarkerArray>("/robot_model_for_shadow_test", 10, [&](const visualization_msgs::MarkerArrayConstPtr& msg){robotModelShadowTest=msg;});

  std::vector<RayCastingShapeMask::MaskValue> mask;
  filter->computeMask(cloud, mask, "laser");

  ASSERT_EQ(num_points(cloud), mask.size());
  EXPECT_EQ(RayCastingShapeMask::MaskValue::CLIP, mask[0]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::CLIP, mask[1]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::CLIP, mask[2]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::CLIP, mask[3]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::INSIDE, mask[4]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::INSIDE, mask[5]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::INSIDE, mask[6]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, mask[7]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, mask[8]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::SHADOW, mask[9]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, mask[10]);
  EXPECT_EQ(RayCastingShapeMask::MaskValue::OUTSIDE, mask[11]);

  WAIT_FOR_MESSAGE(boundingSphere)
  WAIT_FOR_MESSAGE(boundingBox)
  WAIT_FOR_MESSAGE(orientedBoundingBox)
  WAIT_FOR_MESSAGE(localBoundingBox)
  WAIT_FOR_MESSAGE(boundingSphereMarker)
  WAIT_FOR_MESSAGE(boundingBoxMarker)
  WAIT_FOR_MESSAGE(orientedBoundingBoxMarker)
  WAIT_FOR_MESSAGE(localBoundingBoxMarker)
  WAIT_FOR_MESSAGE(boundingSphereDebugMarker)
  WAIT_FOR_MESSAGE(boundingBoxDebugMarker)
  WAIT_FOR_MESSAGE(orientedBoundingBoxDebugMarker)
  WAIT_FOR_MESSAGE(localBoundingBoxDebugMarker)
  WAIT_FOR_MESSAGE(pclNoBoundingSphere)
  WAIT_FOR_MESSAGE(pclNoBoundingBox)
  WAIT_FOR_MESSAGE(pclNoOrientedBoundingBox)
  WAIT_FOR_MESSAGE(pclNoLocalBoundingBox)
  WAIT_FOR_MESSAGE(pclInside)
  WAIT_FOR_MESSAGE(pclClip)
  WAIT_FOR_MESSAGE(pclShadow)
  WAIT_FOR_MESSAGE(robotModelContainsTest)
  WAIT_FOR_MESSAGE(robotModelShadowTest)

  EXPECT_EQ("laser", boundingSphere->header.frame_id);
  EXPECT_EQ(cloud.header.stamp, boundingSphere->header.stamp);
  EXPECT_NEAR(sqrt(3) * 0.92, boundingSphere->sphere.radius, 1e-6);
  EXPECT_NEAR(1.5, boundingSphere->sphere.center.x, 1e-6);
  EXPECT_DOUBLE_EQ(0, boundingSphere->sphere.center.y);
  EXPECT_DOUBLE_EQ(0, boundingSphere->sphere.center.z);
  EXPECT_EQ("laser", boundingSphereMarker->header.frame_id);
  EXPECT_EQ(cloud.header.stamp, boundingSphereMarker->header.stamp);
  EXPECT_NEAR(2*sqrt(3) * 0.92, boundingSphereMarker->scale.x, 1e-6);
  EXPECT_NEAR(2*sqrt(3) * 0.92, boundingSphereMarker->scale.y, 1e-6);
  EXPECT_NEAR(2*sqrt(3) * 0.92, boundingSphereMarker->scale.z, 1e-6);
  EXPECT_NEAR(1.5, boundingSphereMarker->pose.position.x, 1e-6);
  EXPECT_NEAR(0, boundingSphereMarker->pose.position.y, 1e-6);
  EXPECT_NEAR(0, boundingSphereMarker->pose.position.z, 1e-6);
  EXPECT_NEAR(0, boundingSphereMarker->pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, boundingSphereMarker->pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, boundingSphereMarker->pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, boundingSphereMarker->pose.orientation.w, 1e-6);
  EXPECT_LT(0, boundingSphereMarker->color.a);
  // make sure the marker has some color
  EXPECT_LT(0, boundingSphereMarker->color.r + boundingSphereMarker->color.g +
      boundingSphereMarker->color.b);
  EXPECT_EQ(visualization_msgs::Marker::SPHERE, boundingSphereMarker->type);
  EXPECT_EQ(visualization_msgs::Marker::ADD, boundingSphereMarker->action);
  EXPECT_EQ("bounding_sphere", boundingSphereMarker->ns);
  EXPECT_EQ(1, boundingSphereMarker->frame_locked);

  EXPECT_EQ("laser", boundingBox->header.frame_id);
  EXPECT_EQ(cloud.header.stamp, boundingBox->header.stamp);
  ASSERT_EQ(2, boundingBox->polygon.points.size());
  EXPECT_NEAR(1.5-1.55, boundingBox->polygon.points[0].x, 1e-5);
  EXPECT_NEAR(-1.375, boundingBox->polygon.points[0].y, 1e-6);
  EXPECT_NEAR(-1.375, boundingBox->polygon.points[0].z, 1e-6);
  EXPECT_NEAR(1.5 + 1.375, boundingBox->polygon.points[1].x, 1e-5);
  EXPECT_NEAR(1.375, boundingBox->polygon.points[1].y, 1e-6);
  EXPECT_NEAR(1.375, boundingBox->polygon.points[1].z, 1e-6);
  EXPECT_EQ("laser", boundingBoxMarker->header.frame_id);
  EXPECT_EQ(cloud.header.stamp, boundingBoxMarker->header.stamp);
  EXPECT_NEAR(2.925, boundingBoxMarker->scale.x, 1e-5);
  EXPECT_NEAR(2.75, boundingBoxMarker->scale.y, 1e-5);
  EXPECT_NEAR(2.75, boundingBoxMarker->scale.z, 1e-5);
  EXPECT_NEAR(1.5-0.0875, boundingBoxMarker->pose.position.x, 1e-6);
  EXPECT_NEAR(0, boundingBoxMarker->pose.position.y, 1e-6);
  EXPECT_NEAR(0, boundingBoxMarker->pose.position.z, 1e-6);
  EXPECT_NEAR(0, boundingBoxMarker->pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, boundingBoxMarker->pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, boundingBoxMarker->pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, boundingBoxMarker->pose.orientation.w, 1e-6);
  EXPECT_LT(0, boundingBoxMarker->color.a);
  // make sure the marker has some color
  EXPECT_LT(0, boundingBoxMarker->color.r + boundingBoxMarker->color.g +
      boundingBoxMarker->color.b);
  EXPECT_EQ(visualization_msgs::Marker::CUBE, boundingBoxMarker->type);
  EXPECT_EQ(visualization_msgs::Marker::ADD, boundingBoxMarker->action);
  EXPECT_EQ("bounding_box", boundingBoxMarker->ns);
  EXPECT_EQ(1, boundingBoxMarker->frame_locked);

  EXPECT_EQ("laser", orientedBoundingBox->header.frame_id);
  EXPECT_EQ(cloud.header.stamp, orientedBoundingBox->header.stamp);
  EXPECT_NEAR(2.925, orientedBoundingBox->obb.extents.x, 1e-5);
  EXPECT_NEAR(2.75, orientedBoundingBox->obb.extents.y, 1e-5);
  EXPECT_NEAR(2.75, orientedBoundingBox->obb.extents.z, 1e-5);
  EXPECT_NEAR(1.5-0.0875, orientedBoundingBox->obb.pose.translation.x, 1e-5);
  EXPECT_NEAR(0, orientedBoundingBox->obb.pose.translation.y, 1e-6);
  EXPECT_NEAR(0, orientedBoundingBox->obb.pose.translation.z, 1e-6);
  EXPECT_NEAR(0, orientedBoundingBox->obb.pose.rotation.x, 1e-6);
  EXPECT_NEAR(0, orientedBoundingBox->obb.pose.rotation.y, 1e-6);
  EXPECT_NEAR(0, orientedBoundingBox->obb.pose.rotation.z, 1e-6);
  EXPECT_NEAR(1, orientedBoundingBox->obb.pose.rotation.w, 1e-6);
  EXPECT_EQ("laser", orientedBoundingBoxMarker->header.frame_id);
  EXPECT_EQ(cloud.header.stamp, orientedBoundingBoxMarker->header.stamp);
  EXPECT_NEAR(2.925, orientedBoundingBoxMarker->scale.x, 1e-5);
  EXPECT_NEAR(2.75, orientedBoundingBoxMarker->scale.y, 1e-5);
  EXPECT_NEAR(2.75, orientedBoundingBoxMarker->scale.z, 1e-5);
  EXPECT_NEAR(1.5-0.0875, orientedBoundingBoxMarker->pose.position.x, 1e-6);
  EXPECT_NEAR(0, orientedBoundingBoxMarker->pose.position.y, 1e-6);
  EXPECT_NEAR(0, orientedBoundingBoxMarker->pose.position.z, 1e-6);
  EXPECT_NEAR(0, orientedBoundingBoxMarker->pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, orientedBoundingBoxMarker->pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, orientedBoundingBoxMarker->pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, orientedBoundingBoxMarker->pose.orientation.w, 1e-6);
  EXPECT_LT(0, orientedBoundingBoxMarker->color.a);
  // make sure the marker has some color
  EXPECT_LT(0, orientedBoundingBoxMarker->color.r + orientedBoundingBoxMarker->color.g +
      orientedBoundingBoxMarker->color.b);
  EXPECT_EQ(visualization_msgs::Marker::CUBE, orientedBoundingBoxMarker->type);
  EXPECT_EQ(visualization_msgs::Marker::ADD, orientedBoundingBoxMarker->action);
  EXPECT_EQ("oriented_bounding_box", orientedBoundingBoxMarker->ns);
  EXPECT_EQ(1, orientedBoundingBoxMarker->frame_locked);

  EXPECT_EQ("base_link", localBoundingBox->header.frame_id);
  EXPECT_EQ(cloud.header.stamp, localBoundingBox->header.stamp);
  ASSERT_EQ(2, localBoundingBox->polygon.points.size());
  EXPECT_NEAR(-1.55 - 0.122, localBoundingBox->polygon.points[0].x, 1e-5);
  EXPECT_NEAR(-1.375, localBoundingBox->polygon.points[0].y, 1e-6);
  EXPECT_NEAR(-1.375, localBoundingBox->polygon.points[0].z, 1e-6);
  EXPECT_NEAR(1.375 - 0.122, localBoundingBox->polygon.points[1].x, 1e-5);
  EXPECT_NEAR(1.375, localBoundingBox->polygon.points[1].y, 1e-6);
  EXPECT_NEAR(1.375, localBoundingBox->polygon.points[1].z, 1e-6);
  EXPECT_EQ("base_link", localBoundingBoxMarker->header.frame_id);
  EXPECT_EQ(cloud.header.stamp, localBoundingBoxMarker->header.stamp);
  EXPECT_NEAR(2.925, localBoundingBoxMarker->scale.x, 1e-5);
  EXPECT_NEAR(2.75, localBoundingBoxMarker->scale.y, 1e-5);
  EXPECT_NEAR(2.75, localBoundingBoxMarker->scale.z, 1e-5);
  EXPECT_NEAR(-0.0875 - 0.122, localBoundingBoxMarker->pose.position.x, 1e-6);
  EXPECT_NEAR(0, localBoundingBoxMarker->pose.position.y, 1e-6);
  EXPECT_NEAR(0, localBoundingBoxMarker->pose.position.z, 1e-6);
  EXPECT_NEAR(0, localBoundingBoxMarker->pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, localBoundingBoxMarker->pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, localBoundingBoxMarker->pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, localBoundingBoxMarker->pose.orientation.w, 1e-6);
  EXPECT_LT(0, localBoundingBoxMarker->color.a);
  // make sure the marker has some color
  EXPECT_LT(0, localBoundingBoxMarker->color.r + localBoundingBoxMarker->color.g +
      localBoundingBoxMarker->color.b);
  EXPECT_EQ(visualization_msgs::Marker::CUBE, localBoundingBoxMarker->type);
  EXPECT_EQ(visualization_msgs::Marker::ADD, localBoundingBoxMarker->action);
  EXPECT_EQ("local_bounding_box", localBoundingBoxMarker->ns);
  EXPECT_EQ(1, localBoundingBoxMarker->frame_locked);

  ASSERT_EQ(12, num_points(*pclNoBoundingSphere));
  EXPECT_EQ(cloud.header.frame_id, pclNoBoundingSphere->header.frame_id);
  EXPECT_EQ(cloud.header.stamp, pclNoBoundingSphere->header.stamp);
  EXPECT_EQ(cloud.height, pclNoBoundingSphere->height);
  EXPECT_EQ(cloud.width, pclNoBoundingSphere->width);
  EXPECT_EQ(cloud.fields, pclNoBoundingSphere->fields);
  EXPECT_EQ(cloud.point_step, pclNoBoundingSphere->point_step);
  EXPECT_EQ(cloud.row_step, pclNoBoundingSphere->row_step);
  CloudConstIter x_it(*pclNoBoundingSphere, "x");
  EXPECT_NAN(*x_it); ++x_it; // pointSensor
  EXPECT_NAN(*x_it); ++x_it; // pointSensor2
  EXPECT_NAN(*x_it); ++x_it; // pointClipMin
  EXPECT_NEAR(1.5 +  10.0, *x_it, 1e-6); ++x_it; // pointClipMax
  EXPECT_NAN(*x_it); ++x_it; // pointInBox
  EXPECT_NAN(*x_it); ++x_it; // pointInSphere
  EXPECT_NAN(*x_it); ++x_it; // pointInBoth
  EXPECT_NEAR(1.5 + -0.25, *x_it, 1e-6); ++x_it; // pointShadowBox
  EXPECT_NEAR(1.5 + -0.560762, *x_it, 1e-6); ++x_it; // pointShadowSphere
  EXPECT_NAN(*x_it); ++x_it; // pointShadowBoth
  EXPECT_NEAR(1.5 + -3.0, *x_it, 1e-6); ++x_it; // pointOutside
  EXPECT_NEAR(1.5 + -4.0, *x_it, 1e-6); ++x_it; // pointOutside2

  ASSERT_EQ(12, num_points(*pclNoBoundingBox));
  EXPECT_EQ(cloud.header.frame_id, pclNoBoundingBox->header.frame_id);
  EXPECT_EQ(cloud.header.stamp, pclNoBoundingBox->header.stamp);
  EXPECT_EQ(cloud.height, pclNoBoundingBox->height);
  EXPECT_EQ(cloud.width, pclNoBoundingBox->width);
  EXPECT_EQ(cloud.fields, pclNoBoundingBox->fields);
  EXPECT_EQ(cloud.point_step, pclNoBoundingBox->point_step);
  EXPECT_EQ(cloud.row_step, pclNoBoundingBox->row_step);
  x_it = CloudConstIter(*pclNoBoundingBox, "x");
  EXPECT_NAN(*x_it); ++x_it; // pointSensor
  EXPECT_NAN(*x_it); ++x_it; // pointSensor2
  EXPECT_NAN(*x_it); ++x_it; // pointClipMin
  EXPECT_NEAR(1.5 +  10., *x_it, 1e-6); ++x_it; // pointClipMax
  EXPECT_NAN(*x_it); ++x_it; // pointInBox
  EXPECT_NAN(*x_it); ++x_it; // pointInSphere
  EXPECT_NAN(*x_it); ++x_it; // pointInBoth
  EXPECT_NEAR(1.5 + -0.25, *x_it, 1e-6); ++x_it; // pointShadowBox
  EXPECT_NEAR(1.5 + -0.560762, *x_it, 1e-6); ++x_it; // pointShadowSphere
  EXPECT_NEAR(1.5 +  1.5, *x_it, 1e-6); ++x_it; // pointShadowBoth
  EXPECT_NEAR(1.5 + -3.0, *x_it, 1e-6); ++x_it; // pointOutside

  ASSERT_EQ(12, num_points(*pclNoOrientedBoundingBox));
  EXPECT_EQ(cloud.header.frame_id, pclNoOrientedBoundingBox->header.frame_id);
  EXPECT_EQ(cloud.header.stamp, pclNoOrientedBoundingBox->header.stamp);
  EXPECT_EQ(cloud.height, pclNoOrientedBoundingBox->height);
  EXPECT_EQ(cloud.width, pclNoOrientedBoundingBox->width);
  EXPECT_EQ(cloud.fields, pclNoOrientedBoundingBox->fields);
  EXPECT_EQ(cloud.point_step, pclNoOrientedBoundingBox->point_step);
  EXPECT_EQ(cloud.row_step, pclNoOrientedBoundingBox->row_step);
  x_it = CloudConstIter(*pclNoOrientedBoundingBox, "x");
  EXPECT_NAN(*x_it); ++x_it; // pointSensor
  EXPECT_NAN(*x_it); ++x_it; // pointSensor2
  EXPECT_NAN(*x_it); ++x_it; // pointClipMin
  EXPECT_NEAR(1.5 +  10., *x_it, 1e-6); ++x_it; // pointClipMax
  EXPECT_NAN(*x_it); ++x_it; // pointInBox
  EXPECT_NAN(*x_it); ++x_it; // pointInSphere
  EXPECT_NAN(*x_it); ++x_it; // pointInBoth
  EXPECT_NEAR(1.5 + -0.25, *x_it, 1e-6); ++x_it; // pointShadowBox
  EXPECT_NEAR(1.5 + -0.560762, *x_it, 1e-6); ++x_it; // pointShadowSphere
  EXPECT_NEAR(1.5 +  1.5, *x_it, 1e-6); ++x_it; // pointShadowBoth
  EXPECT_NEAR(1.5 + -3.0, *x_it, 1e-6); ++x_it; // pointOutside

  ASSERT_EQ(12, num_points(*pclNoLocalBoundingBox));
  EXPECT_EQ(cloud.header.frame_id, pclNoLocalBoundingBox->header.frame_id);
  EXPECT_EQ(cloud.header.stamp, pclNoLocalBoundingBox->header.stamp);
  EXPECT_EQ(cloud.height, pclNoLocalBoundingBox->height);
  EXPECT_EQ(cloud.width, pclNoLocalBoundingBox->width);
  EXPECT_EQ(cloud.fields, pclNoLocalBoundingBox->fields);
  EXPECT_EQ(cloud.point_step, pclNoLocalBoundingBox->point_step);
  EXPECT_EQ(cloud.row_step, pclNoLocalBoundingBox->row_step);
  x_it = CloudConstIter(*pclNoLocalBoundingBox, "x");
  EXPECT_NAN(*x_it); ++x_it; // pointSensor
  EXPECT_NAN(*x_it); ++x_it; // pointSensor2
  EXPECT_NAN(*x_it); ++x_it; // pointClipMin
  EXPECT_NEAR(1.5 +  10., *x_it, 1e-6); ++x_it; // pointClipMax
  EXPECT_NAN(*x_it); ++x_it; // pointInBox
  EXPECT_NAN(*x_it); ++x_it; // pointInSphere
  EXPECT_NAN(*x_it); ++x_it; // pointInBoth
  EXPECT_NEAR(1.5 + -0.25, *x_it, 1e-6); ++x_it; // pointShadowBox
  EXPECT_NEAR(1.5 + -0.560762, *x_it, 1e-6); ++x_it; // pointShadowSphere
  EXPECT_NEAR(1.5 +  1.5, *x_it, 1e-6); ++x_it; // pointShadowBoth
  EXPECT_NEAR(1.5 + -3.0, *x_it, 1e-6); ++x_it; // pointOutside

  ASSERT_EQ(12, num_points(*pclInside));
  EXPECT_EQ(cloud.header.frame_id, pclInside->header.frame_id);
  EXPECT_EQ(cloud.header.stamp, pclInside->header.stamp);
  EXPECT_EQ(cloud.height, pclInside->height);
  EXPECT_EQ(cloud.width, pclInside->width);
  EXPECT_EQ(cloud.fields, pclInside->fields);
  EXPECT_EQ(cloud.point_step, pclInside->point_step);
  EXPECT_EQ(cloud.row_step, pclInside->row_step);
  x_it = CloudConstIter(*pclInside, "x");
  EXPECT_NAN(*x_it); ++x_it; // pointSensor
  EXPECT_NAN(*x_it); ++x_it; // pointSensor2
  EXPECT_NAN(*x_it); ++x_it; // pointClipMin
  EXPECT_NAN(*x_it); ++x_it; // pointClipMax
  EXPECT_NEAR(1.5 +  0.85, *x_it, 1e-6); ++x_it; // pointInBox
  EXPECT_NEAR(1.5 +  1.35, *x_it, 1e-6); ++x_it; // pointInSphere
  EXPECT_NEAR(1.5 +  0.0, *x_it, 1e-6); ++x_it; // pointInBoth
  EXPECT_NAN(*x_it); ++x_it; // pointShadowBox
  EXPECT_NAN(*x_it); ++x_it; // pointShadowSphere
  EXPECT_NAN(*x_it); ++x_it; // pointShadowBoth
  EXPECT_NAN(*x_it); ++x_it; // pointOutside

  ASSERT_EQ(12, num_points(*pclClip));
  EXPECT_EQ(cloud.header.frame_id, pclClip->header.frame_id);
  EXPECT_EQ(cloud.header.stamp, pclClip->header.stamp);
  EXPECT_EQ(cloud.height, pclClip->height);
  EXPECT_EQ(cloud.width, pclClip->width);
  EXPECT_EQ(cloud.fields, pclClip->fields);
  EXPECT_EQ(cloud.point_step, pclClip->point_step);
  EXPECT_EQ(cloud.row_step, pclClip->row_step);
  x_it = CloudConstIter(*pclClip, "x");
  EXPECT_NEAR(1.5 + -1.5, *x_it, 1e-6); ++x_it; // pointSensor
  EXPECT_NEAR(1.5 + -1.47, *x_it, 1e-6); ++x_it; // pointSensor2
  EXPECT_NEAR(1.5 + -1.42, *x_it, 1e-6); ++x_it; // pointClipMin
  EXPECT_NEAR(1.5 +  10., *x_it, 1e-6); ++x_it; // pointClipMax
  EXPECT_NAN(*x_it); ++x_it; // pointInBox
  EXPECT_NAN(*x_it); ++x_it; // pointInSphere
  EXPECT_NAN(*x_it); ++x_it; // pointInBoth
  EXPECT_NAN(*x_it); ++x_it; // pointShadowBox
  EXPECT_NAN(*x_it); ++x_it; // pointShadowSphere
  EXPECT_NAN(*x_it); ++x_it; // pointShadowBoth
  EXPECT_NAN(*x_it); ++x_it; // pointOutside

  ASSERT_EQ(12, num_points(*pclShadow));
  EXPECT_EQ(cloud.header.frame_id, pclShadow->header.frame_id);
  EXPECT_EQ(cloud.header.stamp, pclShadow->header.stamp);
  EXPECT_EQ(cloud.height, pclShadow->height);
  EXPECT_EQ(cloud.width, pclShadow->width);
  EXPECT_EQ(cloud.fields, pclShadow->fields);
  EXPECT_EQ(cloud.point_step, pclShadow->point_step);
  EXPECT_EQ(cloud.row_step, pclShadow->row_step);
  x_it = CloudConstIter(*pclShadow, "x");
  EXPECT_NAN(*x_it); ++x_it; // pointSensor
  EXPECT_NAN(*x_it); ++x_it; // pointSensor2
  EXPECT_NAN(*x_it); ++x_it; // pointClipMin
  EXPECT_NAN(*x_it); ++x_it; // pointClipMax
  EXPECT_NAN(*x_it); ++x_it; // pointInBox
  EXPECT_NAN(*x_it); ++x_it; // pointInSphere
  EXPECT_NAN(*x_it); ++x_it; // pointInBoth
  EXPECT_NEAR(1.5 + -0.25, *x_it, 1e-6); ++x_it; // pointShadowBox
  EXPECT_NEAR(1.5 + -0.560762, *x_it, 1e-6); ++x_it; // pointShadowSphere
  EXPECT_NEAR(1.5 +  1.5, *x_it, 1e-6); ++x_it; // pointShadowBoth
  EXPECT_NAN(*x_it); ++x_it; // pointOutside

  ASSERT_EQ(2, boundingSphereDebugMarker->markers.size());
  EXPECT_EQ("laser", boundingSphereDebugMarker->markers[0].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, boundingSphereDebugMarker->markers[0].header.stamp);
  EXPECT_NEAR(2*sqrt(3) * 0.92, boundingSphereDebugMarker->markers[0].scale.x, 1e-6);
  EXPECT_NEAR(2*sqrt(3) * 0.92, boundingSphereDebugMarker->markers[0].scale.y, 1e-6);
  EXPECT_NEAR(2*sqrt(3) * 0.92, boundingSphereDebugMarker->markers[0].scale.z, 1e-6);
  EXPECT_NEAR(1.5, boundingSphereDebugMarker->markers[0].pose.position.x, 1e-6);
  EXPECT_NEAR(0, boundingSphereDebugMarker->markers[0].pose.position.y, 1e-6);
  EXPECT_NEAR(0, boundingSphereDebugMarker->markers[0].pose.position.z, 1e-6);
  EXPECT_NEAR(0, boundingSphereDebugMarker->markers[0].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, boundingSphereDebugMarker->markers[0].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, boundingSphereDebugMarker->markers[0].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, boundingSphereDebugMarker->markers[0].pose.orientation.w, 1e-6);
  EXPECT_LT(0, boundingSphereDebugMarker->markers[0].color.a);
  // make sure the marker has some color
  EXPECT_LT(0, boundingSphereDebugMarker->markers[0].color.r + boundingSphereDebugMarker->markers[0].color.g +
      boundingSphereDebugMarker->markers[0].color.b);
  EXPECT_EQ(visualization_msgs::Marker::SPHERE, boundingSphereDebugMarker->markers[0].type);
  EXPECT_EQ(visualization_msgs::Marker::ADD, boundingSphereDebugMarker->markers[0].action);
  EXPECT_FALSE(boundingSphereDebugMarker->markers[0].ns.empty());
  EXPECT_EQ(1, boundingSphereDebugMarker->markers[0].frame_locked);
  EXPECT_EQ("laser", boundingSphereDebugMarker->markers[1].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, boundingSphereDebugMarker->markers[1].header.stamp);
  EXPECT_NEAR(0.1*sqrt(3), boundingSphereDebugMarker->markers[1].scale.x, 1e-5);
  EXPECT_NEAR(0.1*sqrt(3), boundingSphereDebugMarker->markers[1].scale.y, 1e-5);
  EXPECT_NEAR(0.1*sqrt(3), boundingSphereDebugMarker->markers[1].scale.z, 1e-5);
  EXPECT_NEAR(1.5-1.5, boundingSphereDebugMarker->markers[1].pose.position.x, 1e-5);
  EXPECT_NEAR(0, boundingSphereDebugMarker->markers[1].pose.position.y, 1e-6);
  EXPECT_NEAR(0, boundingSphereDebugMarker->markers[1].pose.position.z, 1e-6);
  EXPECT_NEAR(0, boundingSphereDebugMarker->markers[1].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, boundingSphereDebugMarker->markers[1].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, boundingSphereDebugMarker->markers[1].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, boundingSphereDebugMarker->markers[1].pose.orientation.w, 1e-6);
  EXPECT_LT(0, boundingSphereDebugMarker->markers[1].color.a);
  // make sure the marker has some color
  EXPECT_LT(0, boundingSphereDebugMarker->markers[1].color.r + boundingSphereDebugMarker->markers[1].color.g +
      boundingSphereDebugMarker->markers[1].color.b);
  EXPECT_EQ(visualization_msgs::Marker::SPHERE, boundingSphereDebugMarker->markers[1].type);
  EXPECT_EQ(visualization_msgs::Marker::ADD, boundingSphereDebugMarker->markers[1].action);
  EXPECT_FALSE(boundingSphereDebugMarker->markers[1].ns.empty());
  EXPECT_EQ(1, boundingSphereDebugMarker->markers[1].frame_locked);

  EXPECT_EQ(4, boundingBoxDebugMarker->markers.size());
  EXPECT_EQ("laser", boundingBoxDebugMarker->markers[0].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, boundingBoxDebugMarker->markers[0].header.stamp);
  EXPECT_NEAR(2.75, boundingBoxDebugMarker->markers[0].scale.x, 1e-5);
  EXPECT_NEAR(2.75, boundingBoxDebugMarker->markers[0].scale.y, 1e-5);
  EXPECT_NEAR(2.75, boundingBoxDebugMarker->markers[0].scale.z, 1e-5);
  EXPECT_NEAR(1.5, boundingBoxDebugMarker->markers[0].pose.position.x, 1e-6);
  EXPECT_NEAR(0, boundingBoxDebugMarker->markers[0].pose.position.y, 1e-6);
  EXPECT_NEAR(0, boundingBoxDebugMarker->markers[0].pose.position.z, 1e-6);
  EXPECT_NEAR(0, boundingBoxDebugMarker->markers[0].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, boundingBoxDebugMarker->markers[0].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, boundingBoxDebugMarker->markers[0].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, boundingBoxDebugMarker->markers[0].pose.orientation.w, 1e-6);
  EXPECT_LT(0, boundingBoxDebugMarker->markers[0].color.a);
  // make sure the marker has some color
  EXPECT_LT(0, boundingBoxDebugMarker->markers[0].color.r + boundingBoxDebugMarker->markers[0].color.g +
      boundingBoxDebugMarker->markers[0].color.b);
  EXPECT_EQ(visualization_msgs::Marker::CUBE, boundingBoxDebugMarker->markers[0].type);
  EXPECT_EQ(visualization_msgs::Marker::ADD, boundingBoxDebugMarker->markers[0].action);
  EXPECT_FALSE(boundingBoxDebugMarker->markers[0].ns.empty());
  EXPECT_EQ(1, boundingBoxDebugMarker->markers[0].frame_locked);
  EXPECT_EQ("laser", boundingBoxDebugMarker->markers[1].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, boundingBoxDebugMarker->markers[1].header.stamp);
  EXPECT_NEAR(1.84, boundingBoxDebugMarker->markers[1].scale.x, 1e-5);
  EXPECT_NEAR(1.84, boundingBoxDebugMarker->markers[1].scale.y, 1e-5);
  EXPECT_NEAR(1.84, boundingBoxDebugMarker->markers[1].scale.z, 1e-5);
  EXPECT_NEAR(1.5, boundingBoxDebugMarker->markers[1].pose.position.x, 1e-6);
  EXPECT_NEAR(0, boundingBoxDebugMarker->markers[1].pose.position.y, 1e-6);
  EXPECT_NEAR(0, boundingBoxDebugMarker->markers[1].pose.position.z, 1e-6);
  EXPECT_NEAR(0, boundingBoxDebugMarker->markers[1].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, boundingBoxDebugMarker->markers[1].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, boundingBoxDebugMarker->markers[1].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, boundingBoxDebugMarker->markers[1].pose.orientation.w, 1e-6);
  EXPECT_LT(0, boundingBoxDebugMarker->markers[1].color.a);
  // make sure the marker has some color
  EXPECT_LT(0, boundingBoxDebugMarker->markers[1].color.r + boundingBoxDebugMarker->markers[1].color.g +
      boundingBoxDebugMarker->markers[1].color.b);
  EXPECT_EQ(visualization_msgs::Marker::CUBE, boundingBoxDebugMarker->markers[1].type);
  EXPECT_EQ(visualization_msgs::Marker::ADD, boundingBoxDebugMarker->markers[1].action);
  EXPECT_FALSE(boundingBoxDebugMarker->markers[1].ns.empty());
  EXPECT_EQ(1, boundingBoxDebugMarker->markers[1].frame_locked);
  EXPECT_EQ("laser", boundingBoxDebugMarker->markers[2].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, boundingBoxDebugMarker->markers[2].header.stamp);
  EXPECT_NEAR(2.5, boundingBoxDebugMarker->markers[2].scale.x, 1e-4);
  EXPECT_NEAR(2.5, boundingBoxDebugMarker->markers[2].scale.y, 1e-4);
  EXPECT_NEAR(2.5, boundingBoxDebugMarker->markers[2].scale.z, 1e-4);
  EXPECT_NEAR(1.5 + 0.122 - 0.1, boundingBoxDebugMarker->markers[2].pose.position.x, 1e-6);
  EXPECT_NEAR(0, boundingBoxDebugMarker->markers[2].pose.position.y, 1e-6);
  EXPECT_NEAR(0, boundingBoxDebugMarker->markers[2].pose.position.z, 1e-6);
  EXPECT_NEAR(0, boundingBoxDebugMarker->markers[2].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, boundingBoxDebugMarker->markers[2].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, boundingBoxDebugMarker->markers[2].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, boundingBoxDebugMarker->markers[2].pose.orientation.w, 1e-6);
  EXPECT_LT(0, boundingBoxDebugMarker->markers[2].color.a);
  // make sure the marker has some color
  EXPECT_LT(0, boundingBoxDebugMarker->markers[2].color.r + boundingBoxDebugMarker->markers[2].color.g +
      boundingBoxDebugMarker->markers[2].color.b);
  EXPECT_EQ(visualization_msgs::Marker::CUBE, boundingBoxDebugMarker->markers[2].type);
  EXPECT_EQ(visualization_msgs::Marker::ADD, boundingBoxDebugMarker->markers[2].action);
  EXPECT_FALSE(boundingBoxDebugMarker->markers[2].ns.empty());
  EXPECT_EQ(1, boundingBoxDebugMarker->markers[2].frame_locked);
  EXPECT_EQ("laser", boundingBoxDebugMarker->markers[3].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, boundingBoxDebugMarker->markers[3].header.stamp);
  EXPECT_NEAR(0.1, boundingBoxDebugMarker->markers[3].scale.x, 1e-5);
  EXPECT_NEAR(0.1, boundingBoxDebugMarker->markers[3].scale.y, 1e-5);
  EXPECT_NEAR(0.1, boundingBoxDebugMarker->markers[3].scale.z, 1e-5);
  EXPECT_NEAR(1.5-1.5, boundingBoxDebugMarker->markers[3].pose.position.x, 1e-6);
  EXPECT_NEAR(0, boundingBoxDebugMarker->markers[3].pose.position.y, 1e-6);
  EXPECT_NEAR(0, boundingBoxDebugMarker->markers[3].pose.position.z, 1e-6);
  EXPECT_NEAR(0, boundingBoxDebugMarker->markers[3].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, boundingBoxDebugMarker->markers[3].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, boundingBoxDebugMarker->markers[3].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, boundingBoxDebugMarker->markers[3].pose.orientation.w, 1e-6);
  EXPECT_LT(0, boundingBoxDebugMarker->markers[3].color.a);
  // make sure the marker has some color
  EXPECT_LT(0, boundingBoxDebugMarker->markers[3].color.r + boundingBoxDebugMarker->markers[3].color.g +
      boundingBoxDebugMarker->markers[3].color.b);
  EXPECT_EQ(visualization_msgs::Marker::CUBE, boundingBoxDebugMarker->markers[3].type);
  EXPECT_EQ(visualization_msgs::Marker::ADD, boundingBoxDebugMarker->markers[3].action);
  EXPECT_FALSE(boundingBoxDebugMarker->markers[3].ns.empty());
  EXPECT_EQ(1, boundingBoxDebugMarker->markers[3].frame_locked);

  EXPECT_EQ(4, orientedBoundingBoxDebugMarker->markers.size());
  EXPECT_EQ("laser", orientedBoundingBoxDebugMarker->markers[0].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, orientedBoundingBoxDebugMarker->markers[0].header.stamp);
  EXPECT_NEAR(2.75, orientedBoundingBoxDebugMarker->markers[0].scale.x, 1e-5);
  EXPECT_NEAR(2.75, orientedBoundingBoxDebugMarker->markers[0].scale.y, 1e-5);
  EXPECT_NEAR(2.75, orientedBoundingBoxDebugMarker->markers[0].scale.z, 1e-5);
  EXPECT_NEAR(1.5, orientedBoundingBoxDebugMarker->markers[0].pose.position.x, 1e-6);
  EXPECT_NEAR(0, orientedBoundingBoxDebugMarker->markers[0].pose.position.y, 1e-6);
  EXPECT_NEAR(0, orientedBoundingBoxDebugMarker->markers[0].pose.position.z, 1e-6);
  EXPECT_NEAR(0, orientedBoundingBoxDebugMarker->markers[0].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, orientedBoundingBoxDebugMarker->markers[0].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, orientedBoundingBoxDebugMarker->markers[0].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, orientedBoundingBoxDebugMarker->markers[0].pose.orientation.w, 1e-6);
  EXPECT_LT(0, orientedBoundingBoxDebugMarker->markers[0].color.a);
  // make sure the marker has some color
  EXPECT_LT(0, orientedBoundingBoxDebugMarker->markers[0].color.r + orientedBoundingBoxDebugMarker->markers[0].color.g +
      orientedBoundingBoxDebugMarker->markers[0].color.b);
  EXPECT_EQ(visualization_msgs::Marker::CUBE, orientedBoundingBoxDebugMarker->markers[0].type);
  EXPECT_EQ(visualization_msgs::Marker::ADD, orientedBoundingBoxDebugMarker->markers[0].action);
  EXPECT_FALSE(orientedBoundingBoxDebugMarker->markers[0].ns.empty());
  EXPECT_EQ(1, orientedBoundingBoxDebugMarker->markers[0].frame_locked);
  EXPECT_EQ("laser", orientedBoundingBoxDebugMarker->markers[1].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, orientedBoundingBoxDebugMarker->markers[1].header.stamp);
  EXPECT_NEAR(1.84, orientedBoundingBoxDebugMarker->markers[1].scale.x, 1e-5);
  EXPECT_NEAR(1.84, orientedBoundingBoxDebugMarker->markers[1].scale.y, 1e-5);
  EXPECT_NEAR(1.84, orientedBoundingBoxDebugMarker->markers[1].scale.z, 1e-5);
  EXPECT_NEAR(1.5, orientedBoundingBoxDebugMarker->markers[1].pose.position.x, 1e-6);
  EXPECT_NEAR(0, orientedBoundingBoxDebugMarker->markers[1].pose.position.y, 1e-6);
  EXPECT_NEAR(0, orientedBoundingBoxDebugMarker->markers[1].pose.position.z, 1e-6);
  EXPECT_NEAR(0, orientedBoundingBoxDebugMarker->markers[1].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, orientedBoundingBoxDebugMarker->markers[1].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, orientedBoundingBoxDebugMarker->markers[1].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, orientedBoundingBoxDebugMarker->markers[1].pose.orientation.w, 1e-6);
  EXPECT_LT(0, orientedBoundingBoxDebugMarker->markers[1].color.a);
  // make sure the marker has some color
  EXPECT_LT(0, orientedBoundingBoxDebugMarker->markers[1].color.r + orientedBoundingBoxDebugMarker->markers[1].color.g +
      orientedBoundingBoxDebugMarker->markers[1].color.b);
  EXPECT_EQ(visualization_msgs::Marker::CUBE, orientedBoundingBoxDebugMarker->markers[1].type);
  EXPECT_EQ(visualization_msgs::Marker::ADD, orientedBoundingBoxDebugMarker->markers[1].action);
  EXPECT_FALSE(orientedBoundingBoxDebugMarker->markers[1].ns.empty());
  EXPECT_EQ(1, orientedBoundingBoxDebugMarker->markers[1].frame_locked);
  EXPECT_EQ("laser", orientedBoundingBoxDebugMarker->markers[2].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, orientedBoundingBoxDebugMarker->markers[2].header.stamp);
  EXPECT_NEAR(2.5, orientedBoundingBoxDebugMarker->markers[2].scale.x, 1e-4);
  EXPECT_NEAR(2.5, orientedBoundingBoxDebugMarker->markers[2].scale.y, 1e-4);
  EXPECT_NEAR(2.5, orientedBoundingBoxDebugMarker->markers[2].scale.z, 1e-4);
  EXPECT_NEAR(1.5 + 0.122 - 0.1, orientedBoundingBoxDebugMarker->markers[2].pose.position.x, 1e-6);
  EXPECT_NEAR(0, orientedBoundingBoxDebugMarker->markers[2].pose.position.y, 1e-6);
  EXPECT_NEAR(0, orientedBoundingBoxDebugMarker->markers[2].pose.position.z, 1e-6);
  EXPECT_NEAR(0, orientedBoundingBoxDebugMarker->markers[2].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, orientedBoundingBoxDebugMarker->markers[2].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, orientedBoundingBoxDebugMarker->markers[2].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, orientedBoundingBoxDebugMarker->markers[2].pose.orientation.w, 1e-6);
  EXPECT_LT(0, orientedBoundingBoxDebugMarker->markers[2].color.a);
  // make sure the marker has some color
  EXPECT_LT(0, orientedBoundingBoxDebugMarker->markers[2].color.r + orientedBoundingBoxDebugMarker->markers[2].color.g +
      orientedBoundingBoxDebugMarker->markers[2].color.b);
  EXPECT_EQ(visualization_msgs::Marker::CUBE, orientedBoundingBoxDebugMarker->markers[2].type);
  EXPECT_EQ(visualization_msgs::Marker::ADD, orientedBoundingBoxDebugMarker->markers[2].action);
  EXPECT_FALSE(orientedBoundingBoxDebugMarker->markers[2].ns.empty());
  EXPECT_EQ(1, orientedBoundingBoxDebugMarker->markers[2].frame_locked);
  EXPECT_EQ("laser", orientedBoundingBoxDebugMarker->markers[3].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, orientedBoundingBoxDebugMarker->markers[3].header.stamp);
  EXPECT_NEAR(0.1, orientedBoundingBoxDebugMarker->markers[3].scale.x, 1e-5);
  EXPECT_NEAR(0.1, orientedBoundingBoxDebugMarker->markers[3].scale.y, 1e-5);
  EXPECT_NEAR(0.1, orientedBoundingBoxDebugMarker->markers[3].scale.z, 1e-5);
  EXPECT_NEAR(1.5-1.5, orientedBoundingBoxDebugMarker->markers[3].pose.position.x, 1e-6);
  EXPECT_NEAR(0, orientedBoundingBoxDebugMarker->markers[3].pose.position.y, 1e-6);
  EXPECT_NEAR(0, orientedBoundingBoxDebugMarker->markers[3].pose.position.z, 1e-6);
  EXPECT_NEAR(0, orientedBoundingBoxDebugMarker->markers[3].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, orientedBoundingBoxDebugMarker->markers[3].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, orientedBoundingBoxDebugMarker->markers[3].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, orientedBoundingBoxDebugMarker->markers[3].pose.orientation.w, 1e-6);
  EXPECT_LT(0, orientedBoundingBoxDebugMarker->markers[3].color.a);
  // make sure the marker has some color
  EXPECT_LT(0, orientedBoundingBoxDebugMarker->markers[3].color.r + orientedBoundingBoxDebugMarker->markers[3].color.g +
      orientedBoundingBoxDebugMarker->markers[3].color.b);
  EXPECT_EQ(visualization_msgs::Marker::CUBE, orientedBoundingBoxDebugMarker->markers[3].type);
  EXPECT_EQ(visualization_msgs::Marker::ADD, orientedBoundingBoxDebugMarker->markers[3].action);
  EXPECT_FALSE(orientedBoundingBoxDebugMarker->markers[3].ns.empty());
  EXPECT_EQ(1, orientedBoundingBoxDebugMarker->markers[3].frame_locked);

  EXPECT_EQ(4, localBoundingBoxDebugMarker->markers.size());
  EXPECT_EQ("base_link", localBoundingBoxDebugMarker->markers[0].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, localBoundingBoxDebugMarker->markers[0].header.stamp);
  EXPECT_NEAR(2.75, localBoundingBoxDebugMarker->markers[0].scale.x, 1e-5);
  EXPECT_NEAR(2.75, localBoundingBoxDebugMarker->markers[0].scale.y, 1e-5);
  EXPECT_NEAR(2.75, localBoundingBoxDebugMarker->markers[0].scale.z, 1e-5);
  EXPECT_NEAR(-0.122, localBoundingBoxDebugMarker->markers[0].pose.position.x, 1e-6);
  EXPECT_NEAR(0, localBoundingBoxDebugMarker->markers[0].pose.position.y, 1e-6);
  EXPECT_NEAR(0, localBoundingBoxDebugMarker->markers[0].pose.position.z, 1e-6);
  EXPECT_NEAR(0, localBoundingBoxDebugMarker->markers[0].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, localBoundingBoxDebugMarker->markers[0].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, localBoundingBoxDebugMarker->markers[0].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, localBoundingBoxDebugMarker->markers[0].pose.orientation.w, 1e-6);
  EXPECT_LT(0, localBoundingBoxDebugMarker->markers[0].color.a);
  // make sure the marker has some color
  EXPECT_LT(0, localBoundingBoxDebugMarker->markers[0].color.r + localBoundingBoxDebugMarker->markers[0].color.g +
      localBoundingBoxDebugMarker->markers[0].color.b);
  EXPECT_EQ(visualization_msgs::Marker::CUBE, localBoundingBoxDebugMarker->markers[0].type);
  EXPECT_EQ(visualization_msgs::Marker::ADD, localBoundingBoxDebugMarker->markers[0].action);
  EXPECT_FALSE(localBoundingBoxDebugMarker->markers[0].ns.empty());
  EXPECT_EQ(1, localBoundingBoxDebugMarker->markers[0].frame_locked);
  EXPECT_EQ("base_link", localBoundingBoxDebugMarker->markers[1].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, localBoundingBoxDebugMarker->markers[1].header.stamp);
  EXPECT_NEAR(1.84, localBoundingBoxDebugMarker->markers[1].scale.x, 1e-5);
  EXPECT_NEAR(1.84, localBoundingBoxDebugMarker->markers[1].scale.y, 1e-5);
  EXPECT_NEAR(1.84, localBoundingBoxDebugMarker->markers[1].scale.z, 1e-5);
  EXPECT_NEAR(-0.122, localBoundingBoxDebugMarker->markers[1].pose.position.x, 1e-6);
  EXPECT_NEAR(0, localBoundingBoxDebugMarker->markers[1].pose.position.y, 1e-6);
  EXPECT_NEAR(0, localBoundingBoxDebugMarker->markers[1].pose.position.z, 1e-6);
  EXPECT_NEAR(0, localBoundingBoxDebugMarker->markers[1].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, localBoundingBoxDebugMarker->markers[1].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, localBoundingBoxDebugMarker->markers[1].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, localBoundingBoxDebugMarker->markers[1].pose.orientation.w, 1e-6);
  EXPECT_LT(0, localBoundingBoxDebugMarker->markers[1].color.a);
  // make sure the marker has some color
  EXPECT_LT(0, localBoundingBoxDebugMarker->markers[1].color.r + localBoundingBoxDebugMarker->markers[1].color.g +
      localBoundingBoxDebugMarker->markers[1].color.b);
  EXPECT_EQ(visualization_msgs::Marker::CUBE, localBoundingBoxDebugMarker->markers[1].type);
  EXPECT_EQ(visualization_msgs::Marker::ADD, localBoundingBoxDebugMarker->markers[1].action);
  EXPECT_FALSE(localBoundingBoxDebugMarker->markers[1].ns.empty());
  EXPECT_EQ(1, localBoundingBoxDebugMarker->markers[1].frame_locked);
  EXPECT_EQ("base_link", localBoundingBoxDebugMarker->markers[2].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, localBoundingBoxDebugMarker->markers[2].header.stamp);
  EXPECT_NEAR(2.5, localBoundingBoxDebugMarker->markers[2].scale.x, 1e-4);
  EXPECT_NEAR(2.5, localBoundingBoxDebugMarker->markers[2].scale.y, 1e-4);
  EXPECT_NEAR(2.5, localBoundingBoxDebugMarker->markers[2].scale.z, 1e-4);
  EXPECT_NEAR(-0.1, localBoundingBoxDebugMarker->markers[2].pose.position.x, 1e-6);
  EXPECT_NEAR(0, localBoundingBoxDebugMarker->markers[2].pose.position.y, 1e-6);
  EXPECT_NEAR(0, localBoundingBoxDebugMarker->markers[2].pose.position.z, 1e-6);
  EXPECT_NEAR(0, localBoundingBoxDebugMarker->markers[2].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, localBoundingBoxDebugMarker->markers[2].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, localBoundingBoxDebugMarker->markers[2].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, localBoundingBoxDebugMarker->markers[2].pose.orientation.w, 1e-6);
  EXPECT_LT(0, localBoundingBoxDebugMarker->markers[2].color.a);
  // make sure the marker has some color
  EXPECT_LT(0, localBoundingBoxDebugMarker->markers[2].color.r + localBoundingBoxDebugMarker->markers[2].color.g +
      localBoundingBoxDebugMarker->markers[2].color.b);
  EXPECT_EQ(visualization_msgs::Marker::CUBE, localBoundingBoxDebugMarker->markers[2].type);
  EXPECT_EQ(visualization_msgs::Marker::ADD, localBoundingBoxDebugMarker->markers[2].action);
  EXPECT_FALSE(localBoundingBoxDebugMarker->markers[2].ns.empty());
  EXPECT_EQ(1, localBoundingBoxDebugMarker->markers[2].frame_locked);
  EXPECT_EQ("base_link", localBoundingBoxDebugMarker->markers[3].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, localBoundingBoxDebugMarker->markers[3].header.stamp);
  EXPECT_NEAR(0.1, localBoundingBoxDebugMarker->markers[3].scale.x, 1e-5);
  EXPECT_NEAR(0.1, localBoundingBoxDebugMarker->markers[3].scale.y, 1e-5);
  EXPECT_NEAR(0.1, localBoundingBoxDebugMarker->markers[3].scale.z, 1e-5);
  EXPECT_NEAR(-1.5-0.122, localBoundingBoxDebugMarker->markers[3].pose.position.x, 1e-6);
  EXPECT_NEAR(0, localBoundingBoxDebugMarker->markers[3].pose.position.y, 1e-6);
  EXPECT_NEAR(0, localBoundingBoxDebugMarker->markers[3].pose.position.z, 1e-6);
  EXPECT_NEAR(0, localBoundingBoxDebugMarker->markers[3].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, localBoundingBoxDebugMarker->markers[3].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, localBoundingBoxDebugMarker->markers[3].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, localBoundingBoxDebugMarker->markers[3].pose.orientation.w, 1e-6);
  EXPECT_LT(0, localBoundingBoxDebugMarker->markers[3].color.a);
  // make sure the marker has some color
  EXPECT_LT(0, localBoundingBoxDebugMarker->markers[3].color.r + localBoundingBoxDebugMarker->markers[3].color.g +
      localBoundingBoxDebugMarker->markers[3].color.b);
  EXPECT_EQ(visualization_msgs::Marker::CUBE, localBoundingBoxDebugMarker->markers[3].type);
  EXPECT_EQ(visualization_msgs::Marker::ADD, localBoundingBoxDebugMarker->markers[3].action);
  EXPECT_FALSE(localBoundingBoxDebugMarker->markers[3].ns.empty());
  EXPECT_EQ(1, localBoundingBoxDebugMarker->markers[3].frame_locked);

  EXPECT_EQ(4, robotModelContainsTest->markers.size());
  EXPECT_EQ("laser", robotModelContainsTest->markers[0].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, robotModelContainsTest->markers[0].header.stamp);
  EXPECT_NEAR(2.75, robotModelContainsTest->markers[0].scale.x, 1e-5);
  EXPECT_NEAR(2.75, robotModelContainsTest->markers[0].scale.y, 1e-5);
  EXPECT_NEAR(2.75, robotModelContainsTest->markers[0].scale.z, 1e-5);
  EXPECT_NEAR(1.5, robotModelContainsTest->markers[0].pose.position.x, 1e-6);
  EXPECT_NEAR(0, robotModelContainsTest->markers[0].pose.position.y, 1e-6);
  EXPECT_NEAR(0, robotModelContainsTest->markers[0].pose.position.z, 1e-6);
  EXPECT_NEAR(0, robotModelContainsTest->markers[0].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, robotModelContainsTest->markers[0].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, robotModelContainsTest->markers[0].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, robotModelContainsTest->markers[0].pose.orientation.w, 1e-6);
  EXPECT_LT(0, robotModelContainsTest->markers[0].color.a);
  // make sure the marker has some color
  EXPECT_LT(0, robotModelContainsTest->markers[0].color.r + robotModelContainsTest->markers[0].color.g +
      robotModelContainsTest->markers[0].color.b);
  EXPECT_EQ(visualization_msgs::Marker::SPHERE, robotModelContainsTest->markers[0].type);
  EXPECT_EQ(visualization_msgs::Marker::ADD, robotModelContainsTest->markers[0].action);
  EXPECT_EQ("antenna-0", robotModelContainsTest->markers[0].ns);
  EXPECT_EQ(1, robotModelContainsTest->markers[0].frame_locked);
  EXPECT_EQ("laser", robotModelContainsTest->markers[1].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, robotModelContainsTest->markers[1].header.stamp);
  EXPECT_NEAR(1.84, robotModelContainsTest->markers[1].scale.x, 1e-5);
  EXPECT_NEAR(1.84, robotModelContainsTest->markers[1].scale.y, 1e-5);
  EXPECT_NEAR(1.84, robotModelContainsTest->markers[1].scale.z, 1e-5);
  EXPECT_NEAR(1.5, robotModelContainsTest->markers[1].pose.position.x, 1e-6);
  EXPECT_NEAR(0, robotModelContainsTest->markers[1].pose.position.y, 1e-6);
  EXPECT_NEAR(0, robotModelContainsTest->markers[1].pose.position.z, 1e-6);
  EXPECT_NEAR(0, robotModelContainsTest->markers[1].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, robotModelContainsTest->markers[1].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, robotModelContainsTest->markers[1].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, robotModelContainsTest->markers[1].pose.orientation.w, 1e-6);
  EXPECT_LT(0, robotModelContainsTest->markers[1].color.a);
  // make sure the marker has some color
  EXPECT_LT(0, robotModelContainsTest->markers[1].color.r + robotModelContainsTest->markers[1].color.g +
      robotModelContainsTest->markers[1].color.b);
  EXPECT_EQ(visualization_msgs::Marker::CUBE, robotModelContainsTest->markers[1].type);
  EXPECT_EQ(visualization_msgs::Marker::ADD, robotModelContainsTest->markers[1].action);
  EXPECT_EQ("base_link-0", robotModelContainsTest->markers[1].ns);
  EXPECT_EQ(1, robotModelContainsTest->markers[1].frame_locked);
  EXPECT_EQ("laser", robotModelContainsTest->markers[2].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, robotModelContainsTest->markers[2].header.stamp);
  EXPECT_NEAR(2.5, robotModelContainsTest->markers[2].scale.x, 1e-4);
  EXPECT_NEAR(2.5, robotModelContainsTest->markers[2].scale.y, 1e-4);
  EXPECT_NEAR(2.5, robotModelContainsTest->markers[2].scale.z, 1e-4);
  EXPECT_NEAR(1.5+0.122-0.1, robotModelContainsTest->markers[2].pose.position.x, 1e-6);
  EXPECT_NEAR(0, robotModelContainsTest->markers[2].pose.position.y, 1e-6);
  EXPECT_NEAR(0, robotModelContainsTest->markers[2].pose.position.z, 1e-6);
  EXPECT_NEAR(0, robotModelContainsTest->markers[2].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, robotModelContainsTest->markers[2].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, robotModelContainsTest->markers[2].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, robotModelContainsTest->markers[2].pose.orientation.w, 1e-6);
  EXPECT_LT(0, robotModelContainsTest->markers[2].color.a);
  // make sure the marker has some color
  EXPECT_LT(0, robotModelContainsTest->markers[2].color.r + robotModelContainsTest->markers[2].color.g +
      robotModelContainsTest->markers[2].color.b);
  EXPECT_EQ(visualization_msgs::Marker::CUBE, robotModelContainsTest->markers[2].type);
  EXPECT_EQ(visualization_msgs::Marker::ADD, robotModelContainsTest->markers[2].action);
  EXPECT_EQ("base_link-1", robotModelContainsTest->markers[2].ns);
  EXPECT_EQ(1, robotModelContainsTest->markers[2].frame_locked);
  EXPECT_EQ("laser", robotModelContainsTest->markers[3].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, robotModelContainsTest->markers[3].header.stamp);
  EXPECT_NEAR(0.1, robotModelContainsTest->markers[3].scale.x, 1e-5);
  EXPECT_NEAR(0.1, robotModelContainsTest->markers[3].scale.y, 1e-5);
  EXPECT_NEAR(0.1, robotModelContainsTest->markers[3].scale.z, 1e-5);
  EXPECT_NEAR(1.5-1.5, robotModelContainsTest->markers[3].pose.position.x, 1e-6);
  EXPECT_NEAR(0, robotModelContainsTest->markers[3].pose.position.y, 1e-6);
  EXPECT_NEAR(0, robotModelContainsTest->markers[3].pose.position.z, 1e-6);
  EXPECT_NEAR(0, robotModelContainsTest->markers[3].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, robotModelContainsTest->markers[3].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, robotModelContainsTest->markers[3].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, robotModelContainsTest->markers[3].pose.orientation.w, 1e-6);
  EXPECT_LT(0, robotModelContainsTest->markers[3].color.a);
  // make sure the marker has some color
  EXPECT_LT(0, robotModelContainsTest->markers[3].color.r + robotModelContainsTest->markers[3].color.g +
      robotModelContainsTest->markers[3].color.b);
  EXPECT_EQ(visualization_msgs::Marker::CUBE, robotModelContainsTest->markers[3].type);
  EXPECT_EQ(visualization_msgs::Marker::ADD, robotModelContainsTest->markers[3].action);
  EXPECT_EQ("laser-0", robotModelContainsTest->markers[3].ns);
  EXPECT_EQ(1, robotModelContainsTest->markers[3].frame_locked);

  EXPECT_EQ(2, robotModelShadowTest->markers.size());
  EXPECT_EQ("laser", robotModelShadowTest->markers[0].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, robotModelShadowTest->markers[0].header.stamp);
  EXPECT_NEAR(2.75, robotModelShadowTest->markers[0].scale.x, 1e-5);
  EXPECT_NEAR(2.75, robotModelShadowTest->markers[0].scale.y, 1e-5);
  EXPECT_NEAR(2.75, robotModelShadowTest->markers[0].scale.z, 1e-5);
  EXPECT_NEAR(1.5, robotModelShadowTest->markers[0].pose.position.x, 1e-6);
  EXPECT_NEAR(0, robotModelShadowTest->markers[0].pose.position.y, 1e-6);
  EXPECT_NEAR(0, robotModelShadowTest->markers[0].pose.position.z, 1e-6);
  EXPECT_NEAR(0, robotModelShadowTest->markers[0].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, robotModelShadowTest->markers[0].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, robotModelShadowTest->markers[0].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, robotModelShadowTest->markers[0].pose.orientation.w, 1e-6);
  EXPECT_LT(0, robotModelShadowTest->markers[0].color.a);
  // make sure the marker has some color
  EXPECT_LT(0, robotModelShadowTest->markers[0].color.r + robotModelShadowTest->markers[0].color.g +
      robotModelShadowTest->markers[0].color.b);
  EXPECT_EQ(visualization_msgs::Marker::SPHERE, robotModelShadowTest->markers[0].type);
  EXPECT_EQ(visualization_msgs::Marker::ADD, robotModelShadowTest->markers[0].action);
  EXPECT_EQ("antenna-0", robotModelShadowTest->markers[0].ns);
  EXPECT_EQ(1, robotModelShadowTest->markers[0].frame_locked);
  EXPECT_EQ("laser", robotModelShadowTest->markers[1].header.frame_id);
  EXPECT_EQ(cloud.header.stamp, robotModelShadowTest->markers[1].header.stamp);
  EXPECT_NEAR(2, robotModelShadowTest->markers[1].scale.x, 1e-5);
  EXPECT_NEAR(2, robotModelShadowTest->markers[1].scale.y, 1e-5);
  EXPECT_NEAR(2, robotModelShadowTest->markers[1].scale.z, 1e-5);
  EXPECT_NEAR(1.5, robotModelShadowTest->markers[1].pose.position.x, 1e-6);
  EXPECT_NEAR(0, robotModelShadowTest->markers[1].pose.position.y, 1e-6);
  EXPECT_NEAR(0, robotModelShadowTest->markers[1].pose.position.z, 1e-6);
  EXPECT_NEAR(0, robotModelShadowTest->markers[1].pose.orientation.x, 1e-6);
  EXPECT_NEAR(0, robotModelShadowTest->markers[1].pose.orientation.y, 1e-6);
  EXPECT_NEAR(0, robotModelShadowTest->markers[1].pose.orientation.z, 1e-6);
  EXPECT_NEAR(1, robotModelShadowTest->markers[1].pose.orientation.w, 1e-6);
  EXPECT_LT(0, robotModelShadowTest->markers[1].color.a);
  // make sure the marker has some color
  EXPECT_LT(0, robotModelShadowTest->markers[1].color.r + robotModelShadowTest->markers[1].color.g +
      robotModelShadowTest->markers[1].color.b);
  EXPECT_EQ(visualization_msgs::Marker::CUBE, robotModelShadowTest->markers[1].type);
  EXPECT_EQ(visualization_msgs::Marker::ADD, robotModelShadowTest->markers[1].action);
  EXPECT_EQ("base_link-0", robotModelShadowTest->markers[1].ns);
  EXPECT_EQ(1, robotModelShadowTest->markers[1].frame_locked);
}

TEST(RobotBodyFilter, UpdateLaserScan)
{
  ros::NodeHandle nh;

  auto filter = std::make_shared<RobotBodyFilterLaserScanTest>();
  auto filterBase = std::dynamic_pointer_cast<filters::FilterBase<sensor_msgs::LaserScan>>(filter);

  nh.setParam("test_robot_description", ROBOT_URDF);
  filterBase->configure("compute_mask_config_point_by_point", nh);

  sensor_msgs::LaserScan scan;
  scan.header.frame_id = "laser";
  scan.range_min = 0.1;
  scan.range_max = 10;
  scan.angle_min = -M_PI_2;
  scan.angle_max = M_PI_2;
  scan.angle_increment = M_PI / 18;
  scan.time_increment = 1.0f;
  scan.scan_time = 20.0f;
  for (size_t i = 0; i < 19; ++i)
  {
    scan.intensities.push_back(i);
    scan.ranges.push_back(1.5);
  }

  {
    geometry_msgs::TransformStamped tf;
    ros::Time now = ros::Time::now();
    scan.header.stamp = now;
    tf.transform.rotation.w = 1;
    for (double d = -5.0; d < 5.0; d += 0.1)
    {
      tf.header.stamp = now + ros::Duration(d);

      tf.transform.translation.x = 0.122;
      tf.header.frame_id = "odom";
      tf.child_frame_id = "base_link";
      ASSERT_TRUE(filter->tfBuffer->setTransform(tf, "test"));

      tf.transform.translation.x = -1.5 - 0.122;
      tf.header.frame_id = "base_link";
      tf.child_frame_id = "laser";
      ASSERT_TRUE(filter->tfBuffer->setTransform(tf, "test"));

      tf.transform.translation.x = 0.01864 - 0.122;
      tf.header.frame_id = "base_link";
      tf.child_frame_id = "antenna";
      ASSERT_TRUE(filter->tfBuffer->setTransform(tf, "test"));

      tf.header.stamp += ros::Duration(20);

      tf.transform.translation.x = 10.122;
      tf.header.frame_id = "odom";
      tf.child_frame_id = "base_link";
      ASSERT_TRUE(filter->tfBuffer->setTransform(tf, "test"));

      tf.transform.translation.x = -1.5 - 0.122;
      tf.header.frame_id = "base_link";
      tf.child_frame_id = "laser";
      ASSERT_TRUE(filter->tfBuffer->setTransform(tf, "test"));

      tf.transform.translation.x = 0.01864 - 0.122;
      tf.header.frame_id = "base_link";
      tf.child_frame_id = "antenna";
      ASSERT_TRUE(filter->tfBuffer->setTransform(tf, "test"));
    }
  }

  // give TF frames watchdog some time to catch up
  while (!filter->tfFramesWatchdog->isReachable("laser"))
    ros::WallDuration(0.01).sleep();
  while (!filter->tfFramesWatchdog->isReachable("base_link"))
    ros::WallDuration(0.01).sleep();

  sensor_msgs::LaserScan outScan;
  ASSERT_TRUE(filter->update(scan, outScan));

  ASSERT_EQ(19, outScan.ranges.size());
  EXPECT_EQ(1.5, outScan.ranges[0]); EXPECT_EQ(0, outScan.intensities[0]);
  EXPECT_EQ(1.5, outScan.ranges[1]); EXPECT_EQ(1, outScan.intensities[1]);
  EXPECT_EQ(1.5, outScan.ranges[2]); EXPECT_EQ(2, outScan.intensities[2]);
  EXPECT_NAN(outScan.ranges[3]); EXPECT_EQ(3, outScan.intensities[3]);
  EXPECT_NAN(outScan.ranges[4]); EXPECT_EQ(4, outScan.intensities[4]);
  EXPECT_NAN(outScan.ranges[5]); EXPECT_EQ(5, outScan.intensities[5]);
  EXPECT_NAN(outScan.ranges[6]); EXPECT_EQ(6, outScan.intensities[6]);
  EXPECT_NAN(outScan.ranges[7]); EXPECT_EQ(7, outScan.intensities[7]);
  EXPECT_NAN(outScan.ranges[8]); EXPECT_EQ(8, outScan.intensities[8]);
  EXPECT_NAN(outScan.ranges[9]); EXPECT_EQ(9, outScan.intensities[9]);
  EXPECT_NAN(outScan.ranges[10]); EXPECT_EQ(10, outScan.intensities[10]);
  EXPECT_NAN(outScan.ranges[11]); EXPECT_EQ(11, outScan.intensities[11]);
  EXPECT_NAN(outScan.ranges[12]); EXPECT_EQ(12, outScan.intensities[12]);
  EXPECT_NAN(outScan.ranges[13]); EXPECT_EQ(13, outScan.intensities[13]);
  EXPECT_NAN(outScan.ranges[14]); EXPECT_EQ(14, outScan.intensities[14]);
  EXPECT_NAN(outScan.ranges[15]); EXPECT_EQ(15, outScan.intensities[15]);
  EXPECT_EQ(1.5, outScan.ranges[16]); EXPECT_EQ(16, outScan.intensities[16]);
  EXPECT_EQ(1.5, outScan.ranges[17]); EXPECT_EQ(17, outScan.intensities[17]);
  EXPECT_EQ(1.5, outScan.ranges[18]); EXPECT_EQ(18, outScan.intensities[18]);
}

TEST(RobotBodyFilter, UpdatePointCloud2)
{
  ros::NodeHandle nh;

  auto filter = std::make_shared<RobotBodyFilterPointCloud2Test>();
  auto filterBase = std::dynamic_pointer_cast<filters::FilterBase<sensor_msgs::PointCloud2>>(filter);

  nh.setParam("test_robot_description", ROBOT_URDF);
  filterBase->configure("compute_mask_config_all_at_once", nh);

  Cloud cloud;
  cloud.header.frame_id = "laser";
  CloudModifier mod(cloud);
  mod.setPointCloud2Fields(3,
                           "x", 1, sensor_msgs::PointField::FLOAT32,
                           "y", 1, sensor_msgs::PointField::FLOAT32,
                           "z", 1, sensor_msgs::PointField::FLOAT32);
  mod.resize(12);
  cloud.width = 4;
  cloud.height = 3;
  cloud.row_step = cloud.width * cloud.point_step;

  {
    CloudIter x_it(cloud, "x");
    CloudIter y_it(cloud, "y");
    CloudIter z_it(cloud, "z");

    *x_it = 1.5 + -1.5; *y_it = 0; *z_it = 0; ++x_it, ++y_it, ++z_it; // pointSensor
    *x_it = 1.5 + -1.47; *y_it = 0; *z_it = 0; ++x_it, ++y_it, ++z_it; // pointSensor2
    *x_it = 1.5 + -1.42; *y_it = 0; *z_it = 0; ++x_it, ++y_it, ++z_it; // pointClipMin
    *x_it = 1.5 + 10; *y_it = 0; *z_it = 0; ++x_it, ++y_it, ++z_it; // pointClipMax
    *x_it = 1.5 + 0.95; *y_it = 0.95; *z_it = 0.95; ++x_it, ++y_it, ++z_it; // pointInBox
    *x_it = 1.5 + 1.35; *y_it = 0; *z_it = 0; ++x_it, ++y_it, ++z_it; // pointInSphere
    *x_it = 1.5 + 0; *y_it = 0; *z_it = 0; ++x_it, ++y_it, ++z_it; // pointInBoth
    *x_it = 1.5 + -0.25; *y_it = -2; *z_it = 2; ++x_it, ++y_it, ++z_it; // pointShadowBox
    *x_it = 1.5 + -0.560762; *y_it = 0; *z_it = 1.83871; ++x_it, ++y_it, ++z_it; // pointShadowSphere
    *x_it = 1.5 + 1.5; *y_it = 0; *z_it = 0; ++x_it, ++y_it, ++z_it; // pointShadowBoth
    *x_it = 1.5 + -3; *y_it = 0; *z_it = 0; ++x_it, ++y_it, ++z_it; // pointOutside
    *x_it = 1.5 + -4; *y_it = 0; *z_it = 0; ++x_it, ++y_it, ++z_it; // pointOutside2
  }

  {
    ros::Time now = ros::Time::now();
    cloud.header.stamp = now;
    geometry_msgs::TransformStamped tf;
    tf.transform.rotation.w = 1.0;
    for (double d = -5.0; d < 5.0; d += 0.1)
    {
      tf.header.stamp = now + ros::Duration(d);

      tf.transform.translation.x = 0.122;
      tf.header.frame_id = "odom";
      tf.child_frame_id = "base_link";
      filter->tfBuffer->setTransform(tf, "test");

      tf.transform.translation.x = -1.5 - 0.122;
      tf.header.frame_id = "base_link";
      tf.child_frame_id = "laser";
      filter->tfBuffer->setTransform(tf, "test");

      tf.transform.translation.x = 0.01864 - 0.122;
      tf.header.frame_id = "base_link";
      tf.child_frame_id = "antenna";
      filter->tfBuffer->setTransform(tf, "test");
    }
  }

  while (!filter->tfFramesWatchdog->isReachable("laser"))
    ros::WallDuration(0.01).sleep();
  while (!filter->tfFramesWatchdog->isReachable("base_link"))
    ros::WallDuration(0.01).sleep();

  sensor_msgs::PointCloud2 outCloud;
  filter->update(cloud, outCloud);

  ASSERT_EQ(num_points(cloud), num_points(outCloud));
  EXPECT_EQ("base_link", outCloud.header.frame_id);
  EXPECT_EQ(cloud.header.stamp, outCloud.header.stamp);
  EXPECT_EQ(cloud.height, outCloud.height);
  EXPECT_EQ(cloud.width, outCloud.width);
  EXPECT_EQ(cloud.fields, outCloud.fields);
  EXPECT_EQ(cloud.point_step, outCloud.point_step);
  EXPECT_EQ(cloud.row_step, outCloud.row_step);

  CloudConstIter x_it(outCloud, "x");
  CloudConstIter y_it(outCloud, "y");
  CloudConstIter z_it(outCloud, "z");

  // PointSensor
  EXPECT_NAN(*x_it); EXPECT_NAN(*y_it); EXPECT_NAN(*z_it);
  ++x_it, ++y_it, ++z_it;
  // PointSensor2
  EXPECT_NAN(*x_it); EXPECT_NAN(*y_it); EXPECT_NAN(*z_it);
  ++x_it, ++y_it, ++z_it;
  // PointClipMin
  EXPECT_NAN(*x_it); EXPECT_NAN(*y_it); EXPECT_NAN(*z_it);
  ++x_it, ++y_it, ++z_it;
  // PointClipMax
  EXPECT_NAN(*x_it); EXPECT_NAN(*y_it); EXPECT_NAN(*z_it);
  ++x_it, ++y_it, ++z_it;
  // PointInBox
  EXPECT_NAN(*x_it); EXPECT_NAN(*y_it); EXPECT_NAN(*z_it);
  ++x_it, ++y_it, ++z_it;
  // PointInSphere
  EXPECT_NAN(*x_it); EXPECT_NAN(*y_it); EXPECT_NAN(*z_it);
  ++x_it, ++y_it, ++z_it;
  // PointInBoth
  EXPECT_NAN(*x_it); EXPECT_NAN(*y_it); EXPECT_NAN(*z_it);
  ++x_it, ++y_it, ++z_it;
  // PointShadowBox
  EXPECT_NAN(*x_it); EXPECT_NAN(*y_it); EXPECT_NAN(*z_it);
  ++x_it, ++y_it, ++z_it;
  // PointShadowSphere
  EXPECT_NAN(*x_it); EXPECT_NAN(*y_it); EXPECT_NAN(*z_it);
  ++x_it, ++y_it, ++z_it;
  // PointShadowBoth
  EXPECT_NAN(*x_it); EXPECT_NAN(*y_it); EXPECT_NAN(*z_it);
  ++x_it, ++y_it, ++z_it;
  // PointOutside
  EXPECT_NEAR(-3.122, *x_it, 1e-5); EXPECT_NEAR(0, *y_it, 1e-6); EXPECT_NEAR(0, *z_it, 1e-6);
  ++x_it, ++y_it, ++z_it;
  // PointOutside2
  EXPECT_NEAR(-4.122, *x_it, 1e-5); EXPECT_NEAR(0, *y_it, 1e-6); EXPECT_NEAR(0, *z_it, 1e-6);
  ++x_it, ++y_it, ++z_it;

  // test with unorganized clouds

  filter->keepCloudsOrganized = false;

  filter->update(cloud, outCloud);

  ASSERT_EQ(2, num_points(outCloud));
  EXPECT_EQ("base_link", outCloud.header.frame_id);
  EXPECT_EQ(cloud.header.stamp, outCloud.header.stamp);
  EXPECT_EQ(1, outCloud.height);
  EXPECT_EQ(2, outCloud.width);
  EXPECT_EQ(cloud.fields, outCloud.fields);
  EXPECT_EQ(cloud.point_step, outCloud.point_step);

  x_it = CloudConstIter(outCloud, "x");
  y_it = CloudConstIter(outCloud, "y");
  z_it = CloudConstIter(outCloud, "z");

  // PointOutside
  EXPECT_NEAR(-3.122, *x_it, 1e-5); EXPECT_NEAR(0, *y_it, 1e-6); EXPECT_NEAR(0, *z_it, 1e-6);
  ++x_it, ++y_it, ++z_it;
  // PointOutside2
  EXPECT_NEAR(-4.122, *x_it, 1e-5); EXPECT_NEAR(0, *y_it, 1e-6); EXPECT_NEAR(0, *z_it, 1e-6);
  ++x_it, ++y_it, ++z_it;
}

  int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_robot_body_filter");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}