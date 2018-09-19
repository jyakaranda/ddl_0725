#include "ndt_localization/ndt_localization.h"

/** Initialize. 
 * 
 */
bool NDTLocalization::init()
{
  ROS_INFO("Start init NDTLocalization");

  pose_init_ = false;

  pnh_.param<std::string>("map_frame", param_map_frame_, std::string("/map"));
  pnh_.param<std::string>("odom_frame", param_odom_frame_, std::string("/odom"));
  pnh_.param<std::string>("base_frame", param_base_frame_, std::string("/base_link"));
  pnh_.param<std::string>("laser_frame", param_laser_frame_, std::string("/laser"));
  pnh_.param<double>("tf_timeout", param_tf_timeout_, 0.15);

  sub_initial_pose_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, boost::bind(&NDTLocalization::initialPoseCB, this, _1));
  sub_map_ = nh_.subscribe<sensor_msgs::PointCloud2>("/map/point_cloud", 1, boost::bind(&NDTLocalization::mapCB, this, _1));
  sub_odom_ = nh_.subscribe<nav_msgs::Odometry>("/odom/imu", 500, boost::bind(&NDTLocalization::odomCB, this, _1));

  ROS_INFO("End init NDTLocalization");
  return true;
}

/** Set a rough pose estimation by manual. 
 */
void NDTLocalization::initialPoseCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
  geometryPose2Pose(msg->pose.pose, initial_pose_);

  current_pose_ = initial_pose_;
  pre_pose_ = current_pose_;
  pose_init_ = true;

  ROS_INFO("Current pose initialized.");
}

/** Save model points(better to be filtered) for latter use.
 * 1. caculate pdf(mean, covariance) for each voxel grid in model
 */
void NDTLocalization::mapCB(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  // 最好是滤波之后的点云地图
}

/** Save motion data to get a rough pose estimation to give NDT-matching a initial transformation matrix.
 */
void NDTLocalization::odomCB(const nav_msgs::Odometry::ConstPtr &msg)
{
  geometryPose2Pose(msg->pose.pose, current_odom_pose_);
}

/** Get measured data points, estimate current pose using 3D-NDT-matching.
 * 1. get data points
 * 2. match data points to model points(map)
 * 2.1 caculate score function: put the point to corresponding pdf, and sum it up
 * 2.2 optimize transformation matrix(position) using Newton method until score function is converged
 */
void NDTLocalization::pointCloudCB(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  // TODO: main function

  // publish map->odom using map->laser and odom->laser
  tf::StampedTransform transform1;
  try
  {
    tf_listener_.waitForTransform(param_odom_frame_, param_laser_frame_, ros::Time(0), ros::Duration(param_tf_timeout_), ros::Duration(param_tf_timeout_ / 3));
    tf_listener_.lookupTransform(param_odom_frame_, param_laser_frame_, ros::Time(0), transform1);
  }
  catch (const tf::TransformException &ex)
  {
    ROS_ERROR("Long time waiting for tf in pointCloudCB: %s", ex.what());
    // TODO: do some stuff
    return;
  }
  tf::Transform transform2(tf::Quaternion(1, 2, 3, 4), tf::Vector3(1, 2, 3));
  tf_broadcaster_.sendTransform(tf::StampedTransform(transform2 * transform1.inverse(), msg->header.stamp, param_map_frame_, param_odom_frame_));
}
