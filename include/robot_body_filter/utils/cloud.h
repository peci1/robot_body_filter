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

/**
 * \brief Create a pointcloud that contains a subset of point of `IN` defined by
 * the filter `FILTER`. The result is saved into `OUT`. `FILTER` should be a boolean expression
 * which can use the following: `i`: index of the point, `x_it, y_it, z_it` iterators to XYZ coordinates.
 */
#define CREATE_FILTERED_CLOUD(IN, OUT, KEEP_ORGANIZED, FILTER) {\
  const auto inputIsOrganized = IN.height > 1; \
  const auto outIsOrganized = KEEP_ORGANIZED && inputIsOrganized; \
  \
  OUT.header = IN.header; \
  OUT.fields = IN.fields; \
  OUT.point_step = IN.point_step; \
  OUT.height = outIsOrganized ? IN.height : 1; \
  OUT.width = outIsOrganized ? IN.width : 0; \
  \
  OUT.data.resize(0); \
  OUT.data.reserve(IN.data.size()); \
  \
  CloudConstIter x_it(IN, "x"); \
  CloudConstIter y_it(IN, "y"); \
  CloudConstIter z_it(IN, "z"); \
  \
  const auto numPoints = num_points(IN); \
  \
  if (!outIsOrganized) { \
    for (size_t i = 0; i < numPoints; ++i, ++x_it, ++y_it, ++z_it) { \
      if (FILTER) { \
        size_t from = (i / IN.width) * IN.row_step + (i % IN.width) * IN.point_step; \
        size_t to = from + IN.point_step; \
        OUT.data.insert(OUT.data.end(), IN.data.begin() + from, \
                        IN.data.begin() + to); \
        OUT.width++; \
      } \
    } \
    OUT.is_dense = true; \
  } else { \
    OUT.data.insert(OUT.data.end(), IN.data.begin(), IN.data.end()); \
    \
    CloudIter x_out_it(OUT, "x"); \
    CloudIter y_out_it(OUT, "y"); \
    CloudIter z_out_it(OUT, "z"); \
    const auto invalidValue = std::numeric_limits<float>::quiet_NaN(); \
    \
    for (size_t i = 0; i < numPoints; ++i, ++x_it, ++y_it, ++z_it, ++x_out_it, ++y_out_it, ++z_out_it) { \
      if (FILTER) { \
        *x_out_it = *y_out_it = *z_out_it = invalidValue; \
        OUT.is_dense = false; \
      } \
    } \
  } \
  \
  OUT.row_step = OUT.width * OUT.point_step;\
}

}
#endif //ROBOT_BODY_FILTER_CLOUD_H
