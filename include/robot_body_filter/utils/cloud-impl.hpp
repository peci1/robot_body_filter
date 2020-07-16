#pragma once

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

namespace robot_body_filter
{

namespace impl {
template<typename T, typename TT, typename U, typename C, template<typename> class V>
class GenericCloudIteratorBase : public sensor_msgs::impl::PointCloud2IteratorBase<T, TT, U, C, V> {
public:
  GenericCloudIteratorBase(C &cloudMsg, const std::string &fieldName);

  inline size_t getFieldSize() { return this->fieldSize; }

  U* getData() const;

protected:
  size_t fieldSize = 0;
};

template<typename T>
class GenericCloudConstIterator
    : public GenericCloudIteratorBase<unsigned char, const unsigned char, const unsigned char, const sensor_msgs::PointCloud2, GenericCloudConstIterator> {
public:
  GenericCloudConstIterator(const sensor_msgs::PointCloud2 &cloud_msg, const std::string &field_name) :
      GenericCloudIteratorBase<T, const T, const unsigned char, const sensor_msgs::PointCloud2, GenericCloudConstIterator>::GenericCloudIteratorBase(
          cloud_msg, field_name) {}
};

template<typename T>
class GenericCloudIterator
    : public GenericCloudIteratorBase<unsigned char, unsigned char, unsigned char, sensor_msgs::PointCloud2, GenericCloudIterator> {
public:
  GenericCloudIterator(sensor_msgs::PointCloud2 &cloud_msg, const std::string &field_name) :
      GenericCloudIteratorBase<T, T, unsigned char, sensor_msgs::PointCloud2, GenericCloudIterator>::GenericCloudIteratorBase(
          cloud_msg, field_name) {}

  void copyData(const GenericCloudConstIterator<T> &otherIter) const;
  void copyData(const GenericCloudIterator<T> &otherIter) const;
};

}
}
