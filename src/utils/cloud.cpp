#include <robot_body_filter/utils/cloud.h>

namespace robot_body_filter
{

size_t num_points(const Cloud &cloud)
{
  return size_t(cloud.height) * cloud.width;
}

}
