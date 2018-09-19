#include "ndt_localization/ndt_localization.h"

bool NDTLocalization::init()
{
  ROS_INFO("Start init NDTLocalization");

  pose_init_ = false;

  sub_initial_pose_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, boost::bind(&NDTLocalization::initialPoseCB, this, _1));
  sub_map_ = nh_.subscribe<sensor_msgs::PointCloud2>("/map/point_cloud", 1, boost::bind(&NDTLocalization::mapCB, this, _1));
  sub_odom_ = nh_.subscribe<nav_msgs::Odometry>("/odom/imu", 500, boost::bind(&NDTLocalization::odomCB, this, _1));

  ROS_INFO("End init NDTLocalization");
  return true;
}

void NDTLocalization::initialPoseCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
  initial_pose_.x = msg->pose.pose.position.x;
  initial_pose_.y = msg->pose.pose.position.y;
  initial_pose_.z = msg->pose.pose.position.z;
  tf::Matrix3x3(tf::Quaternion(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w))
      .getEulerYPR(initial_pose_.yaw, initial_pose_.pitch, initial_pose_.roll);

  current_pose_ = initial_pose_;
  pre_pose_ = current_pose_;
  pose_init_ = true;

  ROS_INFO("Current pose initialized.");
}

void NDTLocalization::mapCB(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
}

void NDTLocalization::odomCB(const nav_msgs::Odometry::ConstPtr &msg)
{
}
