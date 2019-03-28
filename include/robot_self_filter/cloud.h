//
// Created by Tomas Petricek on 3/28/19.
//

#ifndef ROBOT_SELF_FILTER_CLOUD_H
#define ROBOT_SELF_FILTER_CLOUD_H

namespace robot_self_filter
{

typedef sensor_msgs::PointCloud2 Cloud;
typedef sensor_msgs::PointCloud2Iterator<float> CloudIter;
typedef sensor_msgs::PointCloud2ConstIterator<float> CloudConstIter;
typedef sensor_msgs::PointCloud2Modifier CloudModifier;

size_t num_points(const Cloud &cloud)
{
    return size_t(cloud.height) * cloud.width;
}

}
#endif //ROBOT_SELF_FILTER_CLOUD_H
