#include <sstream>
#define private protected
#include <sensor_msgs/point_cloud2_iterator.h>
#undef private

#include <robot_body_filter/utils/cloud.h>

namespace robot_body_filter
{

size_t num_points(const Cloud &cloud)
{
  return size_t(cloud.height) * cloud.width;
}

bool hasField(const Cloud &cloud, const std::string &fieldName) {
  for (const auto &field : cloud.fields) {
    if (field.name == fieldName)
      return true;
  }
  return false;
}

sensor_msgs::PointField& getField(Cloud& cloud, const std::string& fieldName) {
  for (auto &field : cloud.fields) {
    if (field.name == fieldName)
      return field;
  }
  throw std::runtime_error(std::string("Field ") + fieldName + " does not exist.");
}

const sensor_msgs::PointField& getField(const Cloud& cloud, const std::string& fieldName) {
  for (const auto &field : cloud.fields) {
    if (field.name == fieldName)
      return field;
  }
  throw std::runtime_error(std::string("Field ") + fieldName + " does not exist.");
}

size_t sizeOfPointField(const sensor_msgs::PointField& field)
{
  return sizeOfPointField(field.datatype);
}

size_t sizeOfPointField(int datatype)
{
  if ((datatype == sensor_msgs::PointField::INT8) || (datatype == sensor_msgs::PointField::UINT8))
    return 1u;
  else if ((datatype == sensor_msgs::PointField::INT16) || (datatype == sensor_msgs::PointField::UINT16))
    return 2u;
  else if ((datatype == sensor_msgs::PointField::INT32) || (datatype == sensor_msgs::PointField::UINT32) ||
           (datatype == sensor_msgs::PointField::FLOAT32))
    return 4u;
  else if (datatype == sensor_msgs::PointField::FLOAT64)
    return 8u;
  else
    throw std::runtime_error(std::string("PointField of type ") + std::to_string(datatype) + " does not exist");
}

void copyChannelData(const Cloud& in, Cloud& out, const std::string& fieldName) {
  if (num_points(out) < num_points(in))
    throw std::runtime_error("Output cloud needs to be resized to fit the number of points of the input cloud.");

  GenericCloudConstIter dataIn(in, fieldName);
  GenericCloudIter dataOut(out, fieldName);
  for (; dataIn != dataIn.end(); ++dataIn, ++dataOut)
    dataOut.copyData(dataIn);
}


namespace impl {
template<typename T, typename TT, typename U, typename C, template<typename> class V>
GenericCloudIteratorBase<T, TT, U, C, V>::GenericCloudIteratorBase(C& cloudMsg, const std::string& fieldName)
    : sensor_msgs::impl::PointCloud2IteratorBase<T, TT, U, C, V>(cloudMsg, fieldName) {

  this->fieldSize = sizeOfPointField(getField(cloudMsg, fieldName));
}

template<typename T, typename TT, typename U, typename C, template<typename> class V>
U* GenericCloudIteratorBase<T, TT, U, C, V>::getData() const {
  return this->data_;
}

template<typename T>
void GenericCloudIterator<T>::copyData(const GenericCloudConstIterator<T>& otherIter) const {
  memcpy(this->getData(), otherIter.getData(), this->fieldSize);
}

template<typename T>
void GenericCloudIterator<T>::copyData(const GenericCloudIterator<T>& otherIter) const {
  memcpy(this->getData(), otherIter.getData(), this->fieldSize);
}

// explicitly instantiate
template class GenericCloudIteratorBase<unsigned char, unsigned char, unsigned char, sensor_msgs::PointCloud2, GenericCloudIterator>;
template class GenericCloudIteratorBase<unsigned char, const unsigned char, const unsigned char, const sensor_msgs::PointCloud2, GenericCloudConstIterator>;
template class GenericCloudIterator<unsigned char>;
template class GenericCloudConstIterator<unsigned char>;
}

}
