#include <robot_body_filter/utils/tf2_eigen.h>

namespace tf2 {
void toMsg(const Eigen::Vector3d& in, geometry_msgs::Point32& out)
{
  out.x = in.x();
  out.y = in.y();
  out.z = in.z();
}
}