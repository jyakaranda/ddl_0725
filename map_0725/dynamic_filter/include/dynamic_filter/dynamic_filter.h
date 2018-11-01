#ifndef __DYNAMIC_FILTER__
#define __DYNAMIC_FILTER__

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class DynamicFilter
{
public:
  /**
   * @brief Filter dynamic objects from static objects using ndt.
   * 
   * @param pc1 1st input point cloud
   * @param pc2 2nd input point cloud
   * @param out output point cloud
   */
  void filterDynamic(const PointCloudT::ConstPtr &pc1, const PointCloudT::ConstPtr &pc2, PointCloudT::Ptr out);

private:
};

#endif