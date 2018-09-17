#ifndef LOAM_TO_2D
#define LOAM_TO_2D

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>


// description: subscribe point cloud published from loam, and generate corresponding occupied grid map(2d).

namespace map_0725
{

class LoamTo2d
{
public:
  LoamTo2d(ros::NodeHandle &nh, ros::NodeHandle &pnh);
  ~LoamTo2d() {}
  bool init();
  bool load_params();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_loam;
  ros::Subscriber sub_odom;
  ros::Publisher pub_3d_map;
  ros::Publisher pub_proj_map;
  ros::Publisher pub_odom;
  tf::TransformListener tf_listener;
  tf::TransformBroadcaster tf_broadcaster;

  void pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr &msg);
  void odomCB(const nav_msgs::OdometryConstPtr &msg);     // 更改 odom 及 odom -> base_link 的坐标
  void raw_to_map(const pcl::PointXYZI &in, pcl::PointXYZI &out);

  double param_resolution;
  double param_width;
  double param_height;
  double param_origin_x;
  double param_origin_y;
  std::string param_global_frame;
  std::string param_base_frame;
  std::string param_laser_frame;
  std::string param_odom_frame;

  // projected grid map from laser_cloud_map
  nav_msgs::OccupancyGrid grid_map_;
  nav_msgs::Odometry msg_odom;
  tf::StampedTransform trans_laser_odom;
  double min_x_;
  double min_y_;
  double max_x_;
  double max_y_;
  bool lock;

  pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_raw;
  pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_map;
};
} // namespace map_0725

#endif