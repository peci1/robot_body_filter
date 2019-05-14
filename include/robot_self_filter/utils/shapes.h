#ifndef ROBOT_SELF_FILTER_SHAPES_H
#define ROBOT_SELF_FILTER_SHAPES_H

#include <urdf_model/types.h>
#include <geometric_shapes/shapes.h>

namespace robot_self_filter
{

/** \brief Consruct a masking shape out of the given URDF geometry.
 *
 * Just a helper function to convert urdf::Geometry to the corresponding shapes::Shape.
 *
 * Limitation: non-uniformly scaled meshes are not supported (and are scaled uniformly by
 * the first scaling coefficient). Should be overcome once shapes::Shape supports non-uniform scaling.
 *
 * \param geometry The URDF geometry object to convert.
 */
shapes::ShapeConstPtr constructShape(const urdf::Geometry& geometry);

}

#endif //ROBOT_SELF_FILTER_SHAPES_H
