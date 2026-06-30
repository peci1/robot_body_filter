// SPDX-License-Identifier: BSD-3-Clause
// SPDX-FileCopyrightText: Czech Technical University in Prague

#include "gtest/gtest.h"

#include <cstdint>
#include <map>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include <filters/filter_base.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/utilities.hpp>
#include <robot_body_filter/utils/filter_utils.hpp>

class TestFilter : public robot_body_filter::FilterBase<std::string> {
public:
  bool update(const std::string&, std::string&) override {
    return false;
  }

protected:
  bool configure() override {
    bool default_used;
    EXPECT_EQ(false, getParamVerbose("debug.pcl.inside", true, "", &default_used)); EXPECT_FALSE(default_used);
    EXPECT_EQ(true, getParamVerbose("debug.pcl.nonexistent", true, "", &default_used)); EXPECT_TRUE(default_used);
    EXPECT_EQ(-1, getParamVerbose("test.negative", 1, "", &default_used)); EXPECT_FALSE(default_used);
    EXPECT_EQ(1, getParamVerbose("nonexistent", 1, "", &default_used)); EXPECT_TRUE(default_used);
    EXPECT_EQ(0.1, getParamVerbose("sensor.min_distance", 0.01, "", &default_used)); EXPECT_FALSE(default_used);
    EXPECT_EQ(0.01, getParamVerbose("nonexistent", 0.01, "", &default_used)); EXPECT_TRUE(default_used);
    EXPECT_EQ("odom", getParamVerbose("frames.fixed", std::string("fixed"), "", &default_used));
    EXPECT_FALSE(default_used);
    EXPECT_EQ("fixed", getParamVerbose("nonexistent", std::string("fixed"), "", &default_used));
    EXPECT_TRUE(default_used);
    EXPECT_EQ("odom", getParamVerbose("frames.fixed", "fixed", "", &default_used)); EXPECT_FALSE(default_used);
    EXPECT_EQ("fixed", getParamVerbose("nonexistent", "fixed", "", &default_used)); EXPECT_TRUE(default_used);
    EXPECT_EQ("test", getParamVerbose("test_value", "test", "", &default_used));
    EXPECT_TRUE(default_used);  // wrong value type
    EXPECT_EQ(1, getParamVerbose("frames.fixed", 1, "", &default_used));
    EXPECT_TRUE(default_used);  // wrong value type
    EXPECT_THROW(getParamVerbose("test.negative", static_cast<uint64_t>(1)), std::invalid_argument);
    EXPECT_EQ(1, getParamVerbose("nonexistent", static_cast<uint64_t>(1), "", &default_used));
    EXPECT_TRUE(default_used);
    EXPECT_THROW(getParamVerbose("test.negative", static_cast<unsigned int>(1)), std::invalid_argument);
    EXPECT_EQ(1, getParamVerbose("non.existent", static_cast<unsigned int>(1), "", &default_used));
    EXPECT_TRUE(default_used);
    EXPECT_EQ(rclcpp::Duration::from_seconds(60),
      getParamDuration("transforms.buffer_length", rclcpp::Duration::from_seconds(30), "", &default_used));
    EXPECT_FALSE(default_used);
    EXPECT_EQ(rclcpp::Duration::from_seconds(30),
      getParamDuration("nonexistent", rclcpp::Duration::from_seconds(30), "", &default_used));
    EXPECT_TRUE(default_used);
    EXPECT_EQ(std::vector<std::string>({"antenna", "base_link::big_collision_box"}),
      getParamVerbose("ignored_links.bounding_sphere", std::vector<std::string>(), "", &default_used));
    EXPECT_FALSE(default_used);
    EXPECT_EQ(std::set<std::string>({"antenna", "base_link::big_collision_box"}),
      getParamVerboseSet<std::string>("ignored_links.bounding_sphere", {}, "", &default_used));
    EXPECT_FALSE(default_used);
    EXPECT_EQ(std::set<double>({0, 1}),
      getParamVerboseSet<double>("nonexistent_set", {0, 1}, "", &default_used)); EXPECT_TRUE(default_used);
    EXPECT_EQ((std::map<std::string, double>({
      {"antenna::contains", 1.2}, {"antenna::bounding_sphere", 1.2}, {"antenna::bounding_box", 1.2},
      {"*::big_collision_box::contains", 2.0}, {"*::big_collision_box::bounding_sphere", 2.0},
      {"*::big_collision_box::bounding_box", 2.0}, {"*::big_collision_box::shadow", 3.0}
    })), getParamVerboseMap<double>("body_model.inflation.per_link.scale", {}, "", &default_used));
    EXPECT_FALSE(default_used);
    EXPECT_EQ((std::map<std::string, double>({{"laser::shadow", 0.015}, {"base_link", 0.05}})),
      getParamVerboseMap<double>("body_model.inflation.per_link.padding", {}, "m", &default_used));
    EXPECT_FALSE(default_used);
    EXPECT_EQ((std::map<std::string, double>({{"a", 1}, {"b", 2}})),
      getParamVerboseMap<double>("nonexistent_map", {{"a", 1}, {"b", 2}}, "m", &default_used));
    EXPECT_TRUE(default_used);
    EXPECT_EQ((std::map<std::string, double>({{"a", 1}, {"b", 2}})),
      getParamVerboseMap<double>("body_model.inflation.per_link.all_wrong",
        {{"a", 1}, {"b", 2}}, "m", &default_used));
    EXPECT_TRUE(default_used);
    return true;
  }
};

TEST(FilterUtils, getParamVerboseFromDict) {
  rclcpp::Node nh("test_chain_config");

  const auto filter = std::make_shared<TestFilter>();
  const auto filter_base = std::dynamic_pointer_cast<filters::FilterBase<std::string>>(filter);

  filter_base->configure(
    "filter1.params", "robot_body_filter", nh.get_node_logging_interface(), nh.get_node_parameters_interface());
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
