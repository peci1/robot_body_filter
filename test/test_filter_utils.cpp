#include "gtest/gtest.h"
#include <robot_body_filter/utils/filter_utils.hpp>

using namespace robot_body_filter;

class TestFilter : public robot_body_filter::FilterBase<std::string>
{
  public: bool update(const std::string &data_in, std::string &data_out) override
  {
    return false;
  }

  protected: bool configure() override
  {
    bool defaultUsed;
    EXPECT_EQ(false, this->getParamVerbose("debug/pcl/inside", true, "", &defaultUsed)); EXPECT_FALSE(defaultUsed);
    EXPECT_EQ(true, this->getParamVerbose("debug/pcl/nonexistent", true, "", &defaultUsed)); EXPECT_TRUE(defaultUsed);
    EXPECT_EQ(-1, this->getParamVerbose("test/negative", 1, "", &defaultUsed)); EXPECT_FALSE(defaultUsed);
    EXPECT_EQ(1, this->getParamVerbose("nonexistent", 1, "", &defaultUsed)); EXPECT_TRUE(defaultUsed);
    EXPECT_EQ(0.1, this->getParamVerbose("sensor/min_distance", 0.01, "", &defaultUsed)); EXPECT_FALSE(defaultUsed);
    EXPECT_EQ(0.01, this->getParamVerbose("nonexistent", 0.01, "", &defaultUsed)); EXPECT_TRUE(defaultUsed);
    EXPECT_EQ("odom", this->getParamVerbose("frames/fixed", std::string("fixed"), "", &defaultUsed)); EXPECT_FALSE(defaultUsed);
    EXPECT_EQ("fixed", this->getParamVerbose("nonexistent", std::string("fixed"), "", &defaultUsed)); EXPECT_TRUE(defaultUsed);
    EXPECT_EQ("odom", this->getParamVerbose("frames/fixed", "fixed", "", &defaultUsed)); EXPECT_FALSE(defaultUsed);
    EXPECT_EQ("fixed", this->getParamVerbose("nonexistent", "fixed", "", &defaultUsed)); EXPECT_TRUE(defaultUsed);
    EXPECT_EQ("test", this->getParamVerbose("test_value", "test", "", &defaultUsed)); EXPECT_TRUE(defaultUsed); // wrong value type
    EXPECT_EQ(1, this->getParamVerbose("frames/fixed", 1, "", &defaultUsed)); EXPECT_TRUE(defaultUsed); // wrong value type
    EXPECT_THROW(this->getParamVerbose("test/negative", static_cast<uint64_t>(1)),
      std::invalid_argument);
    EXPECT_EQ(1, this->getParamVerbose("nonexistent", static_cast<uint64_t>(1), "", &defaultUsed)); EXPECT_TRUE(defaultUsed);
    EXPECT_THROW(this->getParamVerbose("test/negative", static_cast<unsigned int>(1)),
      std::invalid_argument);
    EXPECT_EQ(1, this->getParamVerbose("non/existent", static_cast<unsigned int>(1), "", &defaultUsed)); EXPECT_TRUE(defaultUsed);
    EXPECT_EQ(ros::Duration(60), this->getParamVerbose("transforms/buffer_length",
        ros::Duration(30), "", &defaultUsed)); EXPECT_FALSE(defaultUsed);
    EXPECT_EQ(ros::Duration(30), this->getParamVerbose("nonexistent", ros::Duration(30), "", &defaultUsed)); EXPECT_TRUE(defaultUsed);
    EXPECT_EQ(std::vector<std::string>({"antenna", "base_link::big_collision_box"}),
              this->getParamVerbose("ignored_links/bounding_sphere", std::vector<std::string>(), "", &defaultUsed)); EXPECT_FALSE(defaultUsed);
    EXPECT_EQ(std::set<std::string>({"antenna", "base_link::big_collision_box"}),
        this->getParamVerboseSet<std::string>("ignored_links/bounding_sphere", {}, "", &defaultUsed)); EXPECT_FALSE(defaultUsed);
    EXPECT_EQ(std::set<double>({0, 1}),
        this->getParamVerboseSet<double>("nonexistent_set", {0, 1}, "", &defaultUsed)); EXPECT_TRUE(defaultUsed);
    EXPECT_EQ((std::map<std::string, double>({{"antenna::contains", 1.2}, {"antenna::bounding_sphere", 1.2}, {"antenna::bounding_box", 1.2}, {"*::big_collision_box::contains", 2.0}, {"*::big_collision_box::bounding_sphere", 2.0}, {"*::big_collision_box::bounding_box", 2.0}, {"*::big_collision_box::shadow", 3.0}})),
        this->getParamVerboseMap<double>("body_model/inflation/per_link/scale", {}, "", &defaultUsed)); EXPECT_FALSE(defaultUsed);
    EXPECT_EQ((std::map<std::string, double>({{"laser::shadow", 0.015}, {"base_link", 0.05}})),
        this->getParamVerboseMap<double>("body_model/inflation/per_link/padding", {}, "m", &defaultUsed)); EXPECT_FALSE(defaultUsed);
    EXPECT_EQ((std::map<std::string, double>({{"a", 1}, {"b", 2}})),
        this->getParamVerboseMap<double>("nonexistent_map", {{"a", 1}, {"b", 2}}, "m", &defaultUsed)); EXPECT_TRUE(defaultUsed);
    EXPECT_EQ((std::map<std::string, double>({{"a", 1}, {"b", 2}})),
        this->getParamVerboseMap<double>("body_model/inflation/per_link/all_wrong", {{"a", 1}, {"b", 2}}, "m", &defaultUsed)); EXPECT_TRUE(defaultUsed);
    return true;
  }
};

TEST(FilterUtils, getParamVerboseFromDict)
{
  ros::NodeHandle nh;

  auto filter = std::make_shared<TestFilter>();
  auto filterBase = std::dynamic_pointer_cast<filters::FilterBase<std::string>>(filter);

  filterBase->configure("test_dict_config", nh);
}

TEST(FilterUtils, getParamVerboseFromChain)
{
  ros::NodeHandle nh;

  auto filter = std::make_shared<TestFilter>();
  auto filterBase = std::dynamic_pointer_cast<filters::FilterBase<std::string>>(filter);

  XmlRpc::XmlRpcValue value;
  nh.getParam("test_chain_config", value);
  ASSERT_EQ(XmlRpc::XmlRpcValue::TypeArray, value.getType());
  ASSERT_EQ(1, value.size());

  filterBase->configure(value[0]);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_filter_utils");
  return RUN_ALL_TESTS();
}