#ifndef __NDT_LOCALZATION__
#define __NDT_LOCALZATION__

#include <ros/ros.h>
#include <ros/duration.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>

#include <iostream>
#include <iomanip>
#include <string>
#include <sstream>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/thread/thread.hpp>
#include <pthread.h>
#include <chrono>

#include <ndt_gpu/NormalDistributionsTransform.h>
#include <pcl_omp_registration/ndt.h>
#ifndef USE_OMP
#define USE_OMP
#endif

#include "user_protocol.h"
#include "utils.hpp"

#define METHOD_PCL 0
#define METHOD_CUDA 1
#define METHOD_OMP 2

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class NDTLocalization
{
public:
  NDTLocalization(ros::NodeHandle nh, ros::NodeHandle pnh) : nh_(nh), pnh_(pnh)
  {
  }
  ~NDTLocalization();
  /**
   * @brief Initialize. 
   * 
   * @return true 
   * @return false 
   */
  bool init();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  tf::TransformBroadcaster tf_broadcaster_;
  tf::TransformListener tf_listener_;

  ros::Publisher pub_current_pose_;
  geometry_msgs::PoseStamped msg_current_pose_;
  ros::Publisher pub_marker_loc_conf_;
  ros::Publisher pub_marker_trans_prob_;

  ros::Subscriber sub_odom_;
  nav_msgs::Odometry::ConstPtr msg_odom_; // under odom frame
  ros::Subscriber sub_map_;
  PointCloudT model_pc_;
  ros::Subscriber sub_initial_pose_;
  pose initial_pose_; // under map frame
  ros::Subscriber sub_point_cloud_;
  PointCloudT data_pc_;

  pose current_pose_;
  pose pre_pose_;
  pose current_pose_odom_;
  pose pre_pose_odom_;
  pose predict_pose_odom_;
  pose offset_odom_;
  ros::Time pre_odom_time_;
  pose current_pose_imu_;
  pose pre_pose_imu_;
  pose predict_pose_imu_;
  pose offset_imu_;
  ros::Time pre_imu_time_;
  Eigen::Matrix4f tf_btol_;
  tf::Transform current_map2odom_;

  bool pose_init_;
  bool odom_init_;
  bool map_init_;
  int model_pc_num_;
  pthread_mutex_t mutex;

#ifdef CUDA_FOUND
  std::shared_ptr<gpu::GNormalDistributionsTransform> anh_gpu_ndt_ptr;
#endif
#ifdef USE_OMP
  pcl_omp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> omp_ndt_;
#endif

  pcl::NormalDistributionsTransform<PointT, PointT> ndt_;
  bool has_converged_;
  double fitness_score_;
  double trans_probability_;
  int iteration_;
  double predict_pose_error_;

  std::string param_odom_frame_;
  std::string param_map_frame_;
  std::string param_base_frame_;
  std::string param_laser_frame_;
  double param_tf_timeout_;
  double param_odom_timeout_;
  bool param_use_odom_;
  double param_predict_error_thresh_;

  double param_ndt_resolution_;
  int param_ndt_max_iterations_;
  double param_ndt_step_size_;
  double param_ndt_epsilon_;
  int param_method_type_;

  // debug use
  bool param_debug_;
  bool rawodom_init_;
  ros::Publisher pub_rawodom_;
  nav_msgs::Odometry msg_rawodom_;

  /**
   * @brief Save motion data to get a rough pose estimation to give NDT-matching a initial transformation matrix.
   * 
   * @param msg 
   */
  void odomCB(const nav_msgs::Odometry::ConstPtr &msg);

  /**
   * @brief Save model points(better to be filtered) for latter use.
   * 
   * @param msg 
   */
  void mapCB(const sensor_msgs::PointCloud2::ConstPtr &msg);

  /**
   * @brief Set a rough pose estimation by manual. 
   * 
   * @param msg 
   */
  void initialPoseCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);

  /**
   * @brief Get measured data points, estimate current pose using 3D-NDT-matching.
   * 
   * @param msg 
   */
  void pointCloudCB(const sensor_msgs::PointCloud2::ConstPtr &msg);

  static bool pubMarkerText(const ros::Publisher pub, const geometry_msgs::Pose pose, const ros::Time stamp, const std::string frame, const std::string text);

  static bool pubMarkerCylinder(const ros::Publisher pub, const geometry_msgs::Pose pose, const ros::Time stamp, const std::string frame, const geometry_msgs::Vector3 scale);

  static bool pubMarkerCube(const ros::Publisher pub, const geometry_msgs::Pose pose, const ros::Time stamp, const std::string frame, const geometry_msgs::Vector3 scale);
};

#endif