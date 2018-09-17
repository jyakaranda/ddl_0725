#include "map_filter/map_filter.h"

namespace map
{
void ICPFilter::cloudFilter(const std::vector<PointCloudT::Ptr> v_cloud_in, const int iterations, PointCloudT::Ptr cloud_match, PointCloudT::Ptr cloud_out)
{
  Eigen::Matrix4d trans_matrix = Eigen::Matrix4d::Identity();
  PointCloudT::Ptr cloud_icp(new PointCloudT);
  PointCloudT::Ptr cloud_tr(new PointCloudT);
  pcl::console::TicToc time;

  pcl::transformPointCloud(*v_cloud_in[0], *cloud_icp, trans_matrix);
  *cloud_tr = *cloud_icp;

  time.tic();
  pcl::IterativeClosestPoint<PointT, PointT> icp;
  icp.setMaximumIterations(iterations);
  icp.setInputSource(cloud_icp);
  icp.setInputTarget(v_cloud_in[0]);
  icp.align(*cloud_icp);
  icp.setMaximumIterations(1);
  std::cout << "Applied " << iterations << " ICP iterations in " << time.toc() << " ms." << std::endl;

  if (icp.hasConverged())
  {
    std::cout << "ICP has converged, score is " << icp.getFitnessScore() << std::endl;
    trans_matrix = icp.getFinalTransformation().cast<double>();
  }
  else
  {
    PCL_ERROR("ICP has not converged.\n");
    return;
  }

  
}
} // namespace map