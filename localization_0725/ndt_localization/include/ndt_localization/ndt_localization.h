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

#include <iostream>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <boost/thread/thread.hpp>

#include "user_protocol.h"

class NDTLocalization
{
public:
  NDTLocalization(ros::NodeHandle nh, ros::NodeHandle pnh) : nh_(nh), pnh_(pnh)
  {
  }
  ~NDTLocalization();
  bool init();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Publisher pub_current_pose_;
  geometry_msgs::PoseStamped msg_current_pose_;

  ros::Subscriber sub_odom_;
  ros::Subscriber sub_map_;
  ros::Subscriber sub_initial_pose_;

  pose current_pose_;
  pose pre_pose_;
  pose initial_pose_;

  bool pose_init_;

  void odomCB(const nav_msgs::Odometry::ConstPtr &msg);
  void mapCB(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void initialPoseCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg);
};

#endif