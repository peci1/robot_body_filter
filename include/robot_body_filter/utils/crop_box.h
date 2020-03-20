#ifndef ROBOT_BODY_FILTER_CROP_BOX_H
#define ROBOT_BODY_FILTER_CROP_BOX_H

#include <pcl/filters/crop_box.h>

namespace robot_body_filter
{

/**
 * \brief Filter a box out of a pointcloud.
 * \note This is a workaround for bug https://github.com/PointCloudLibrary/pcl/issues/3471 .
 *       When all supported platforms have at least PCL 1.10, this class can be removed.
 */
class CropBoxPointCloud2 : public pcl::CropBox<pcl::PCLPointCloud2>
{
  void applyFilter(pcl::PCLPointCloud2 &output) override;
};

}

#endif //ROBOT_BODY_FILTER_CROP_BOX_H
