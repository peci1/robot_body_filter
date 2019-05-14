#ifndef ROBOT_SELF_FILTER_URDF_EIGEN_HPP
#define ROBOT_SELF_FILTER_URDF_EIGEN_HPP

#include <Eigen/Core>
#include <urdf_model/types.h>

namespace robot_self_filter
{

/**
 * \brief Helper function to convert URDF pose to Eigen transform.
 *
 * \param pose The pose to convert.
 * \return The corresponding Eigen transform.
 */
inline Eigen::Isometry3d urdfPose2EigenTransform(const urdf::Pose &pose) {
  return Eigen::Translation3d(pose.position.x, pose.position.y, pose.position.z) *
      Eigen::Quaterniond(pose.rotation.w, pose.rotation.x, pose.rotation.y, pose.rotation.z);
  // who the ### decided that W is put first in Eigen::Quaternion constructor???
}

}

#endif //ROBOT_SELF_FILTER_URDF_EIGEN_HPP
