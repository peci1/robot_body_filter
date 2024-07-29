#ifndef ROBOT_BODY_FILTER_TF2_EIGEN_H
#define ROBOT_BODY_FILTER_TF2_EIGEN_H

#include <Eigen/Geometry>
#include <geometry_msgs/msg/point32.hpp>

namespace tf2 {
void toMsg(const Eigen::Vector3d& in, geometry_msgs::msg::Point32& out);
}

#endif //ROBOT_BODY_FILTER_TF2_EIGEN_H
