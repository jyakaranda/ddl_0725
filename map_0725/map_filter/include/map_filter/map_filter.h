#ifndef __MAP_FILTER__
#define __MAP_FILTER__
#include <pcl_ros/io/pcd_io.h>
#include <pcl_ros/io/bag_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/filters/statistical_outlier_removal.h>
#include <pcl/registration/icp.h>
#include <pcl/console/time.h>
#include <ros/ros.h>
#include <ros/duration.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <string>
#include <vector>

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

namespace map
{
class Filter
{
public:
  virtual void cloudFilter(const std::vector<PointCloudT::Ptr> v_cloud_in, const int iterations, PointCloudT::Ptr cloud_match, PointCloudT::Ptr cloud_out);
};

class ICPFilter : public Filter
{
public:
  ICPFilter();
  ~ICPFilter();
  void cloudFilter(const std::vector<PointCloudT::Ptr> v_cloud_in, const int iterations, PointCloudT::Ptr cloud_match, PointCloudT::Ptr cloud_out);

private:
  PointCloudT::Ptr cloud_in;
  PointCloudT::Ptr cloud_tr;
  PointCloudT::Ptr cloud_match;
  PointCloudT::Ptr cloud_out;
  int iterations;
};

class DynamicFilter : public Filter
{
};

class SparseFilter : public Filter
{

};
}; // namespace map

#endif