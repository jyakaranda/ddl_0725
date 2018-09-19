#include "ndt_localization/ndt_localization.h"

bool NDTLocalization::init()
{

  sub_initial_pose_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, boost::bind(&NDTLocalization::initialPoseCB, this, _1));
  sub_map_ = nh_.subscribe<sensor_msgs::PointCloud2>("/map/point_cloud", 1, boost::bind(&NDTLocalization::mapCB, this, _1));
  sub_odom_ = nh_.subscribe<nav_msgs::Odometry>("/odom/imu", 500, boost::bind(&NDTLocalization::odomCB, this, _1));


  return true;
}

void NDTLocalization::initialPoseCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){

}

void NDTLocalization::mapCB(const sensor_msgs::PointCloud2::ConstPtr& msg){
  
}
