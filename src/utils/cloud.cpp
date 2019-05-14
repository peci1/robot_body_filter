#include <robot_body_filter/utils/cloud.h>

namespace robot_body_filter
{

size_t num_points(const Cloud &cloud)
{
  return size_t(cloud.height) * cloud.width;
}

void createFilteredCloud(const sensor_msgs::PointCloud2& in,
    std::function<bool(size_t, float, float, float)> filter,
    sensor_msgs::PointCloud2& out, const bool keepOrganized)
{
  const auto inputIsOrganized = in.height > 1;
  const auto outIsOrganized = keepOrganized && inputIsOrganized;

  out.header = in.header;
  out.fields = in.fields;
  out.point_step = in.point_step;
  out.height = outIsOrganized ? in.height : 1;
  out.width = outIsOrganized ? in.width : 0;

  out.data.resize(0);
  out.data.reserve(in.data.size());

  CloudConstIter x_it(in, "x");
  CloudConstIter y_it(in, "y");
  CloudConstIter z_it(in, "z");

  if (!outIsOrganized) {
    // Iterate over the points and copy corresponding data chunks.
    for (size_t i = 0; i < num_points(in); ++i, ++x_it, ++y_it, ++z_it) {
      if (filter(i, *x_it, *y_it, *z_it)) {
        size_t from = (i / in.width) * in.row_step + (i % in.width) * in.point_step;
        size_t to = from + in.point_step;
        out.data.insert(out.data.end(), in.data.begin() + from,
                        in.data.begin() + to);
        out.width++;
      }
    }
    out.is_dense = true;
  } else {
    // copy the cloud to output
    out.data.insert(out.data.end(), in.data.begin(), in.data.end());

    CloudIter x_out_it(out, "x");
    CloudIter y_out_it(out, "y");
    CloudIter z_out_it(out, "z");
    const auto invalidValue = std::numeric_limits<float>::quiet_NaN();

    for (size_t i = 0; i < num_points(in); ++i, ++x_it, ++y_it, ++z_it, ++x_out_it, ++y_out_it, ++z_out_it) {
      if (!filter(i, *x_it, *y_it, *z_it)) {
        *x_out_it = *y_out_it = *z_out_it = invalidValue;
        out.is_dense = false;
      }
    }
  }

  out.row_step = out.width * out.point_step;
}

}
