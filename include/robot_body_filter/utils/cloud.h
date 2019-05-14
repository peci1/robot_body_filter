//
// Created by Tomas Petricek on 3/28/19.
//

#ifndef ROBOT_BODY_FILTER_CLOUD_H
#define ROBOT_BODY_FILTER_CLOUD_H

#include <functional>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

namespace robot_body_filter
{

typedef sensor_msgs::PointCloud2 Cloud;
typedef sensor_msgs::PointCloud2Iterator<float> CloudIter;
typedef sensor_msgs::PointCloud2Iterator<int> CloudIndexIter;
typedef sensor_msgs::PointCloud2ConstIterator<float> CloudConstIter;
typedef sensor_msgs::PointCloud2ConstIterator<int> CloudIndexConstIter;
typedef sensor_msgs::PointCloud2Modifier CloudModifier;

size_t num_points(const Cloud &cloud);

void createFilteredCloud(const sensor_msgs::PointCloud2& in,
  std::function<bool(size_t, float, float, float)> filter,
  sensor_msgs::PointCloud2& out, bool keepOrganized);

}
#endif //ROBOT_BODY_FILTER_CLOUD_H
