#ifndef __LOCAL_PLANNER_DWA__
#define __LOCAL_PLANNER_DWA__

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <dynamic_reconfigure/server.h>

#include <cmath>
#include <vector>
#include <pthread.h>
#include <Eigen/Core>

#include <base_local_planner/trajectory.h>
#include <local_planner_dwa/DWAConfig.h>

class DWAPlanner
{
public:
  DWAPlanner();
  ~DWAPlanner();
  bool init();
  bool generateTrajectories(const geometry_msgs::Pose &pose, const geometry_msgs::Pose &goal, const nav_msgs::Odometry &odom);
  bool findBestPath(nav_msgs::Path &path);

private:
  pthread_mutex_t mutex_;
  pthread_mutex_t mutex_path_;
  int closest_pose_i_;                               // 全局路径中距离当前位置最近的点的 index
  std::vector<geometry_msgs::Pose> ref_global_path_; // reference global path，局部路径所跟踪的路径
  nav_msgs::OccupancyGrid global_local_costmap_;

  double param_time_period_;      // 时间片，假设速度在该时间片内恒定
  double param_acc_lin_limit_;    // 加速度限制
  double param_acc_ang_limit_;    // 转向限制
  double param_obstacle_margin_;  // 距离障碍物的 margin
  double param_footprint_radius_; // 半径，暂且先将机器人抽象为一个圆
  double param_max_vel_x_;        // 最大线速度
  double param_min_vel_x_;
  double param_max_ang_z_;        // 最大转向角度
  double param_min_ang_z_;
  double param_timeout_;          // 位置信息、速度信息等的超时限制
  double param_sim_time_;
  int param_vx_samples_;
  int param_vth_samples_;
  double param_sim_granularity_;
  double param_angular_sim_granularity_;
  std::string param_frame_localcostmap_;
  std::string param_frame_globalcostmap_;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_pose_; // 获取机器人的当前位置
  geometry_msgs::PoseStamped msg_cur_pose_;
  ros::Subscriber sub_odom_; // 获取机器人当前速度信息
  nav_msgs::Odometry msg_cur_odom_;
  ros::Subscriber sub_global_path_; // 获取全局路径
  nav_msgs::Path msg_global_path_;
  ros::Subscriber sub_local_map_; // 获取局部地图
  nav_msgs::OccupancyGrid msg_local_map_;
  ros::Subscriber sub_global_map_;
  nav_msgs::OccupancyGrid msg_global_map_;
  dynamic_reconfigure::Server<local_planner_dwa::DWAConfig> cfg_server_;
  dynamic_reconfigure::Server<local_planner_dwa::DWAConfig>::CallbackType cfg_cb_;
  tf::TransformListener tf_listener_;

  ros::Publisher pub_local_path_;
  ros::Publisher pub_local_global_path_; // 发布全局路径在局部地图中的映射
  ros::Publisher pub_cmd_vel_;

  void poseCB(const geometry_msgs::PoseStamped::ConstPtr &msg);
  void odomCB(const nav_msgs::Odometry::ConstPtr &msg);
  void globalPathCB(const nav_msgs::Path::ConstPtr &msg);
  void localMapCB(const nav_msgs::OccupancyGrid::ConstPtr &msg);
  void DWAPlanner::globalCostmapCB(const nav_msgs::OccupancyGrid::ConstPtr &msg);
  void cfgCB(const local_planner_dwa::DWAConfig &config, uint32_t level);

  bool isOutofMap(const double x, const double y, const nav_msgs::OccupancyGrid &map);
  int getCostofMap(const double x, const double y, const nav_msgs::OccupancyGrid &map);
};

#endif