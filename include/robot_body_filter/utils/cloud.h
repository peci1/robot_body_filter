//
// Created by Tomas Petricek on 3/28/19.
// Edited by Martin Pecka
//

#ifndef ROBOT_BODY_FILTER_CLOUD_H
#define ROBOT_BODY_FILTER_CLOUD_H

#include <functional>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <robot_body_filter/utils/cloud-impl.hpp>

namespace robot_body_filter
{

typedef sensor_msgs::PointCloud2 Cloud;
typedef sensor_msgs::PointCloud2Iterator<float> CloudIter;
typedef sensor_msgs::PointCloud2Iterator<int> CloudIndexIter;
typedef sensor_msgs::PointCloud2ConstIterator<float> CloudConstIter;
typedef sensor_msgs::PointCloud2ConstIterator<int> CloudIndexConstIter;
typedef sensor_msgs::PointCloud2Modifier CloudModifier;

// from cloud-impl.hpp
/**
 * GenericCloudIter and GenericCloudConstIter are iterators of fields of types unknown at compile time.
 *
 * The iterators allow you to dereference them into an unsigned char, which doesn't however need to be the actual data,
 * as they may span multiple bytes.
 *
 * It adds function getData() which returns a pointer to the current position in the uchar data stream. You can use
 * reinterpret_cast to transform the data into some desired type and get or set the value. Any kind of data safety is on
 * you.
 *
 * The non-const iterator also provides method copyData() which can copy the field data from another generic iterator.
 * This can be used to copy fields of types which are not known at compile time.
 */
typedef impl::GenericCloudIterator<unsigned char> GenericCloudIter;
typedef impl::GenericCloudConstIterator<unsigned char> GenericCloudConstIter;

size_t num_points(const Cloud &cloud);

/**
 * \brief Create a pointcloud that contains a subset of points of `IN` defined by
 * the filter `FILTER`. The result is saved into `OUT`. `FILTER` should be a boolean expression
 * which can use the following: `i`: index of the point, `x_it, y_it, z_it` iterators to XYZ coordinates.
 * Points for which FILTER is true are part of the final pointcloud.
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
  ::robot_body_filter::CloudConstIter x_it(IN, "x"); \
  ::robot_body_filter::CloudConstIter y_it(IN, "y"); \
  ::robot_body_filter::CloudConstIter z_it(IN, "z"); \
  \
  const auto numPoints = ::robot_body_filter::num_points(IN); \
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
    ::robot_body_filter::CloudIter x_out_it(OUT, "x"); \
    ::robot_body_filter::CloudIter y_out_it(OUT, "y"); \
    ::robot_body_filter::CloudIter z_out_it(OUT, "z"); \
    const auto invalidValue = std::numeric_limits<float>::quiet_NaN(); \
    \
    for (size_t i = 0; i < numPoints; ++i, ++x_it, ++y_it, ++z_it, ++x_out_it, ++y_out_it, ++z_out_it) { \
      if (!(FILTER)) { \
        *x_out_it = *y_out_it = *z_out_it = invalidValue; \
        OUT.is_dense = false; \
      } \
    } \
  } \
  \
  OUT.row_step = OUT.width * OUT.point_step;\
}

/**
 * Return true if the cloud contains a field with the given name.
 * @param cloud The cloud to search.
 * @param fieldName Name of the field.
 * @return Whether the field is there or not.
 */
bool hasField(const Cloud& cloud, const std::string& fieldName);

/**
 * Return the sensor_msgs::PointField with the given name.
 * @param cloud Cloud to extract the field from.
 * @param fieldName Name of the field.
 * @return Reference to the field.
 * @throws std::runtime_error if the field doesn't exist.
 */
sensor_msgs::PointField& getField(Cloud& cloud, const std::string& fieldName);

/**
 * Return the sensor_msgs::PointField with the given name.
 * @param cloud Cloud to extract the field from.
 * @param fieldName Name of the field.
 * @return Reference to the field.
 * @throws std::runtime_error if the field doesn't exist.
 */
const sensor_msgs::PointField& getField(const Cloud& cloud, const std::string& fieldName);

/**
 * Return the size (in bytes) of a sensor_msgs::PointField datatype.
 * @param datatype The datatype (one of sensor_msgs::PointField::(U?INT(8|16|32)|FLOAT(32|64)) constants).
 * @return Size of the datatype in bytes.
 * @throws std::runtime_error if wrong datatype is passed.
 */
size_t sizeOfPointField(int datatype);

/**
 * Return the size (in bytes) of the data represented by the sensor_msgs::PointField.
 * @param field The pointfield specification.
 * @return Size of the data.
 * @throws std::runtime_error if wrong datatype is passed.
 */
size_t sizeOfPointField(const sensor_msgs::PointField& field);

/**
 * Copy data belonging to the given field from `in` cloud to `out` cloud.
 * @param in The input cloud.
 * @param out The ouptut cloud. It has to be resized to contain at least that many points as the input cloud. It also
 *            has to have the given field present already.
 * @param fieldName Name of the field whose data should be copied.
 * @throws std::runtime_error If the output cloud is smaller (in nr. of points) than the input cloud.
 * @throws std::runtime_error If the given field doesn't exist in either of the clouds.
 */
void copyChannelData(const Cloud& in, Cloud& out, const std::string& fieldName);

}
#endif //ROBOT_BODY_FILTER_CLOUD_H
