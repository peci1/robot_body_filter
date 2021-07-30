#include <robot_body_filter/utils/crop_box.h>
#include <pcl/filters/filter_indices.h>

void robot_body_filter::CropBoxPointCloud2::applyFilter(pcl::PCLPointCloud2 &output)
{
  if (!this->keep_organized_) {
    pcl::CropBox<pcl::PCLPointCloud2>::applyFilter(output);
  } else {
    // this is basically a copy-paste from
    // https://github.com/PointCloudLibrary/pcl/blob/pcl-1.10.1/filters/src/crop_box.cpp
    // BSD license, Copyright (c) 2015, Google, Inc.
    // minor modifications Martin Pecka, 2020
    const auto temp = extract_removed_indices_;
    extract_removed_indices_ = true;
    std::vector<int> indices;
    pcl::CropBox<pcl::PCLPointCloud2>::applyFilter(indices);
    extract_removed_indices_ = temp;
    PCL_DEBUG("[pcl::CropBox<pcl::PCLPointCloud2>::applyFilter] Removing %lu points of "
              "%lu points.\n", removed_indices_->size(), input_->height * input_->width);

    output = *input_;

    // Get x, y, z fields. We should not just assume that they are the first fields of each point
    std::vector<decltype(pcl::PCLPointField::offset)> offsets;
    for (const pcl::PCLPointField& field : input_->fields)
    {
      if (field.name == "x" || field.name == "y" || field.name == "z")
        offsets.push_back(field.offset);
    }
    PCL_DEBUG("[pcl::CropBox<pcl::PCLPointCloud2>::applyFilter] Found %lu fields called "
              "'x', 'y', or 'z'.\n", offsets.size());

    // For every "removed" point, set the x, y, z fields to user_filter_value_
    for (const auto ri : *removed_indices_) // ri = removed index
    {
      auto pt_data = reinterpret_cast<std::uint8_t*>(&output.data[ri * static_cast<size_t>(output.point_step)]);
      for (const auto &offset : offsets)
      {
        memcpy(pt_data + offset, &user_filter_value_, sizeof(float));
      }
    }
    if (!std::isfinite(user_filter_value_))
    {
      PCL_DEBUG("[pcl::CropBox<pcl::PCLPointCloud2>::applyFilter] user_filter_value_ is %f, which "
                "is not finite, so the is_dense field of the output will be set to false.\n",
                user_filter_value_);
      output.is_dense = false;
    }
  }
}
