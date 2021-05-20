#ifndef ROBOT_BODY_FILTER_CROP_BOX_H
#define ROBOT_BODY_FILTER_CROP_BOX_H

#include <pcl/filters/crop_box.h>

namespace robot_body_filter
{

#if PCL_VERSION_COMPARE(<, 1, 10, 0)

/**
 * \brief Filter a box out of a pointcloud.
 * \note This is a workaround for bug https://github.com/PointCloudLibrary/pcl/issues/3471 .
 *       When all supported platforms have at least PCL 1.10, this class can be removed.
 */
class CropBoxPointCloud2 : public pcl::CropBox<pcl::PCLPointCloud2>
{
  void applyFilter(pcl::PCLPointCloud2 &output) override;
};

#else

typedef pcl::CropBox<pcl::PCLPointCloud2> CropBoxPointCloud2;

#endif

}

#endif //ROBOT_BODY_FILTER_CROP_BOX_H
