#ifndef ROBOT_BODY_FILTER_TF2_EIGEN_H
#define ROBOT_BODY_FILTER_TF2_EIGEN_H

#include <Eigen/Geometry>
#include <geometry_msgs/Point32.h>

namespace tf2 {
void toMsg(const Eigen::Vector3d& in, geometry_msgs::Point32& out);
}

#endif //ROBOT_BODY_FILTER_TF2_EIGEN_H
