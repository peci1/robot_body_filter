#include <robot_body_filter/utils/obb.h>

#include <fcl/BV/OBB.h>

namespace bodies
{

fcl::Vec3f toFcl(const Eigen::Vector3d& eigenVec)
{
  fcl::Vec3f result;
  Eigen::Map<Eigen::MatrixXd>(result.data.vs, 3, 1) = eigenVec;
  return result;
}

Eigen::Vector3d fromFcl(const fcl::Vec3f& fclVec)
{
  return Eigen::Map<const Eigen::MatrixXd>(fclVec.data.vs, 3, 1);
}

class OBBPrivate : public fcl::OBB
{
public:
  using fcl::OBB::OBB;
};

OBB::OBB()
{
  this->obb_.reset(new OBBPrivate);
}

OBB::OBB(const OBB& other) : OBB()
{
  *obb_ = *other.obb_;
}

OBB& OBB::operator=(const OBB& other)
{
  *obb_ = *other.obb_;
}

OBB::OBB( const Eigen::Isometry3d& pose, const Eigen::Vector3d& extents) : OBB()
{
  setPoseAndExtents(pose, extents);
}

void OBB::setPoseAndExtents(const Eigen::Isometry3d& pose, const Eigen::Vector3d& extents)
{
  const auto rotation = pose.linear();

  obb_->axis[0] = toFcl(rotation.col(0));
  obb_->axis[1] = toFcl(rotation.col(1));
  obb_->axis[2] = toFcl(rotation.col(2));

  obb_->To = toFcl(pose.translation());

  obb_->extent = { extents[0] / 2.0, extents[1] / 2.0, extents[2] / 2.0};
}

void OBB::getExtents(Eigen::Vector3d& extents) const
{
  extents = 2 * fromFcl(obb_->extent);
}

Eigen::Vector3d OBB::getExtents() const
{
  Eigen::Vector3d extents;
  getExtents(extents);
  return extents;
}

void OBB::getPose(Eigen::Isometry3d& pose) const
{
  pose = Eigen::Isometry3d::Identity();
  pose.translation() = fromFcl(obb_->To);
  pose.linear().col(0) = fromFcl(obb_->axis[0]);
  pose.linear().col(1) = fromFcl(obb_->axis[1]);
  pose.linear().col(2) = fromFcl(obb_->axis[2]);
}

Eigen::Isometry3d OBB::getPose() const
{
  Eigen::Isometry3d pose;
  getPose(pose);
  return pose;
}

OBB* OBB::extendApprox(const OBB& box)
{
  if (this->getExtents() == Eigen::Vector3d::Zero())
  {
    *obb_ = *box.obb_;
    return this;
  }

  if (this->contains(box))
    return this;

  if (box.contains(*this)) {
    *obb_ = *box.obb_;
    return this;
  }
  
  *this->obb_ += *box.obb_;
  return this;
}

bool OBB::contains(const Eigen::Vector3d& point) const
{
  return obb_->contain(toFcl(point));
}

bool OBB::overlaps(const bodies::OBB& other) const
{
  return obb_->overlap(*other.obb_);
}

EigenSTL::vector_Vector3d OBB::computeVertices() const
{
  const auto e = getExtents() / 2;
  EigenSTL::vector_Vector3d result = {
      { -e[0], -e[1], -e[2] },
      { -e[0], -e[1],  e[2] },
      { -e[0],  e[1], -e[2] },
      { -e[0],  e[1],  e[2] },
      {  e[0], -e[1], -e[2] },
      {  e[0], -e[1],  e[2] },
      {  e[0],  e[1], -e[2] },
      {  e[0],  e[1],  e[2] },
  };

  const auto pose = getPose();
  for (size_t i = 0; i < result.size(); ++i) {
    result[i] = pose * result[i];
  }

  return result;
}

bool OBB::contains(const OBB& obb) const
{
  for (const auto& v : computeVertices()) {
    if (!contains(v))
      return false;
  }
  return true;
}


OBB::~OBB() = default;

}