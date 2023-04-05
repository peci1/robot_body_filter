#include <robot_body_filter/utils/bodies.h>

#include <geometric_shapes/bodies.h>
#include <geometric_shapes/mesh_operations.h>

namespace bodies
{

void computeBoundingBoxAt(const bodies::Body *body,
                          AxisAlignedBoundingBox &bbox,
                          const Eigen::Isometry3d &pose)
{
  bbox.setEmpty();

  if (body == nullptr)
    return;

  switch (body->getType()) {
    case shapes::SPHERE:
    {
      bodies::Sphere copy(*static_cast<const bodies::Sphere*>(body));
      copy.setPose(pose);
      copy.computeBoundingBox(bbox);
    }
      break;
    case shapes::CYLINDER:
    {
      bodies::Cylinder copy(*static_cast<const bodies::Cylinder*>(body));
      copy.setPose(pose);
      copy.computeBoundingBox(bbox);
    }
      break;
    case shapes::BOX:
    {
      bodies::Box copy(*static_cast<const bodies::Box*>(body));
      copy.setPose(pose);
      copy.computeBoundingBox(bbox);
    }
      break;
    case shapes::MESH:
    {
      // TODO this makes dynamic allocations for the ConvexMesh object, though mesh data are
      // shallow-copied, so performance shouldn't be that bad.
      // ConvexMesh doesn't allow copy-construction as the other bodies because it contains
      // a unique_ptr.
      const auto copy = body->cloneAt(pose);
      copy->computeBoundingBox(bbox);
    }
      break;
    case shapes::PLANE:
    case shapes::CONE:
    case shapes::UNKNOWN_SHAPE:
    case shapes::OCTREE:
      throw std::runtime_error("Unsupported geometric body type.");
  }
}

}
