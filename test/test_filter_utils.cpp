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
    EXPECT_EQ(false, this->getParamVerbose("debug/pcl/inside", true));
    EXPECT_EQ(true, this->getParamVerbose("debug/pcl/nonexistent", true));
    EXPECT_EQ(-1, this->getParamVerbose("test/negative", 1));
    EXPECT_EQ(1, this->getParamVerbose("nonexistent", 1));
    EXPECT_EQ(0.1, this->getParamVerbose("sensor/min_distance", 0.01));
    EXPECT_EQ(0.01, this->getParamVerbose("nonexistent", 0.01));
    EXPECT_EQ("odom", this->getParamVerbose("frames/fixed", std::string("fixed")));
    EXPECT_EQ("fixed", this->getParamVerbose("nonexistent", std::string("fixed")));
    EXPECT_EQ("odom", this->getParamVerbose("frames/fixed", "fixed"));
    EXPECT_EQ("fixed", this->getParamVerbose("nonexistent", "fixed"));
    EXPECT_THROW(this->getParamVerbose("test/negative", static_cast<uint64_t>(1)),
      std::invalid_argument);
    EXPECT_EQ(1, this->getParamVerbose("nonexistent", static_cast<uint64_t>(1)));
    EXPECT_THROW(this->getParamVerbose("test/negative", static_cast<unsigned int>(1)),
      std::invalid_argument);
    EXPECT_EQ(1, this->getParamVerbose("non/existent", static_cast<unsigned int>(1)));
    EXPECT_EQ(ros::Duration(60), this->getParamVerbose("transforms/buffer_length",
        ros::Duration(30)));
    EXPECT_EQ(ros::Duration(30), this->getParamVerbose("nonexistent", ros::Duration(30)));
    EXPECT_EQ(std::vector<std::string>({"antenna", "base_link::big_collision_box"}),
              this->getParamVerbose("ignored_links/bounding_sphere", std::vector<std::string>()));
    EXPECT_EQ(std::set<std::string>({"antenna", "base_link::big_collision_box"}),
        this->getParamVerboseSet<std::set<std::string>>("ignored_links/bounding_sphere"));
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