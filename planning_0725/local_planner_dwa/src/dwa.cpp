#include "local_planner_dwa/dwa.h"

DWAPlanner::DWAPlanner()
{
}

DWAPlanner::~DWAPlanner()
{
}

bool DWAPlanner::init()
{
  pnh_.param<double>("time_period", param_time_period_, 0.1);
  pnh_.param<double>("acc_lin_limit", param_acc_lin_limit_, 0.5);
  pnh_.param<double>("acc_ang_limit", param_acc_ang_limit_, 0.1);
  pnh_.param<double>("obstacle_margin", param_obstacle_margin_, 0.4);
  pnh_.param<double>("footprint_radius", param_footprint_radius_, 0.3);
  pnh_.param<double>("max_vel_x", param_max_vel_x_, 1.5);
  pnh_.param<double>("min_vel_x", param_min_vel_x_, 0.);
  pnh_.param<double>("max_ang_z", param_max_ang_z_, 1.57);
  pnh_.param<double>("min_ang_z", param_min_ang_z_, 0.);
  pnh_.param<double>("timeout", param_timeout_, 0.3);
  pnh_.param<double>("sim_time", param_sim_time_, 0.1);
  pnh_.param<int>("vx_samples", param_vx_samples_, 5);
  pnh_.param<int>("vth_samples", param_vth_samples_, 20);
  pnh_.param<std::string>("local_costmap", param_frame_localcostmap_, std::string("/laser"));
  pnh_.param<std::string>("global_costmap", param_frame_globalcostmap_, std::string("map"));

  sub_pose_ = nh_.subscribe<geometry_msgs::PoseStamped>("/ndt/current_pose", 50, boost::bind(&DWAPlanner::poseCB, this, _1));
  sub_odom_ = nh_.subscribe<nav_msgs::Odometry>("/odom/imu", 50, boost::bind(&DWAPlanner::odomCB, this, _1));
  sub_global_path_ = nh_.subscribe<nav_msgs::Path>("/global_path", 1, boost::bind(&DWAPlanner::globalPathCB, this, _1));
  sub_local_map_ = nh_.subscribe<nav_msgs::OccupancyGrid>("/local_costmap", 1, boost::bind(&DWAPlanner::localMapCB, this, _1));
  sub_global_map_ = nh_.subscribe<nav_msgs::OccupancyGrid>("/global_costmap", 1, boost::bind(&DWAPlanner::globalCostmapCB, this, _1));
  cfg_cb_ = boost::bind(&DWAPlanner::cfg_cb_, this, _1, _2);
  cfg_server_.setCallback(cfg_cb_);
  pub_cmd_vel_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  pub_local_path_ = nh_.advertise<nav_msgs::Path>("/local_path", 1);
  pub_local_global_path_ = nh_.advertise<nav_msgs::Path>("/local_global_path", 1);
}

bool DWAPlanner::generateTrajectories(const geometry_msgs::Pose &pose, const geometry_msgs::Pose &goal, const nav_msgs::Odometry &odom)
{
  std::vector<base_local_planner::Trajectory> v_trajs;

  double max_vel_x, max_ang_z, min_vel_x, min_ang_z;
  max_vel_x = std::max(std::min(hypot(pose.position.x - goal.position.x, pose.position.y - goal.position.y) / param_sim_time_, param_min_vel_x_), param_max_vel_x_);
  max_vel_x = std::min(max_vel_x, odom.twist.twist.linear.x + param_acc_lin_limit_ * param_sim_time_);
  min_vel_x = std::max(param_min_vel_x_, odom.twist.twist.linear.x - param_acc_lin_limit_ * param_sim_time_);
  max_ang_z = std::min(param_max_ang_z_, odom.twist.twist.angular.z + param_acc_ang_limit_ * param_sim_time_);
  min_ang_z = std::max(param_min_ang_z_, odom.twist.twist.angular.z - param_acc_ang_limit_ * param_sim_time_);
  std::vector<geometry_msgs::Point> v_samples;
  geometry_msgs::Point vel_tmp;
  // dynamic window
  for (int i = 0; i < param_vx_samples_; i++)
  {
    vel_tmp.x = min_vel_x + (max_vel_x - min_vel_x) / (param_vx_samples_ - 1);
    for (int j = 0; j < param_vth_samples_; j++)
    {
      vel_tmp.z = min_ang_z + (max_ang_z - min_ang_z) / (param_vth_samples_ - 1);
      v_samples.push_back(vel_tmp);
    }
  }
  nav_msgs::OccupancyGrid global_map = msg_global_map_;

  geometry_msgs::Pose pose_tmp;

  double best_score = -1.;
  base_local_planner::Trajectory best_traj;
  // 根据当前位置、速度、采样速度产生路径
  for (int i = 0; i < v_samples.size(); i++)
  {
    pose_tmp.position.x = pose.position.x;
    pose_tmp.position.y = pose.position.y;
    pose_tmp.position.z = tf::getYaw(tf::Quaternion(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w));

    int num_step = std::ceil(std::max(v_samples[i].x * param_sim_time_ / param_sim_granularity_, std::fabs(v_samples[i].z * param_sim_time_ / param_angular_sim_granularity_)));
    double target_vel = v_samples[i].x, target_ang = v_samples[i].z;
    base_local_planner::Trajectory tmp_traj;
    tmp_traj.time_delta_ = param_sim_time_ / num_step;
    tmp_traj.xv_ = target_vel;
    tmp_traj.thetav_ = target_ang;
    bool flag = true;

    for (int j = 0; j < num_step; j++)
    {
      if (isOutofMap(pose_tmp.position.x, pose_tmp.position.y, global_map))
      {
        flag = false;
        break;
      }
      tmp_traj.addPoint(pose_tmp.position.x, pose_tmp.position.y, pose_tmp.position.z);
      pose_tmp.position.x = pose_tmp.position.x + (target_vel * cos(pose_tmp.position.z)) * tmp_traj.time_delta_;
      pose_tmp.position.y = pose_tmp.position.y + (target_vel * sin(pose_tmp.position.z)) * tmp_traj.time_delta_;
      pose_tmp.position.z = pose_tmp.position.z + target_ang * tmp_traj.time_delta_;
    }

    if (!flag)
    {
      continue;
    }

    // TODO: 判断路径好坏
    // 1. target heading, 2. clearance, 3. velocity, 4. smoothing
    double cost_2 = -1, cost_3 = 0., cost_4 = 0.;
    double px, py, pth;
    for (int j = 0; j < tmp_traj.getPointsSize(); j++)
    {
      tmp_traj.getPoint(j, px, py, pth);
      // 2. obstacle_cost of footprint
      for (int x = (px - param_footprint_radius_ / 2.) / global_map.info.resolution; x < (px + param_footprint_radius_ / 2.) / global_map.info.resolution; x++)
      {
        for (int y = (py - param_footprint_radius_ / 2.) / global_map.info.resolution; y < (py + param_footprint_radius_ / 2.) / global_map.info.resolution; y++)
        {
          cost_2 += getCostofMap(x, y, global_map);
        }
      }
      // cost_2 += getCostofMap(px, py, global_map);
      if (cost_2 >= 100)
      {
        flag = false;
        break;
      }
    }
    // 3. velocity
    cost_3 -= tmp_traj.xv_;
    cost_4 += std::fabs(tmp_traj.thetav_);

    double score = -cost_2 - cost_3 / param_max_vel_x_ - cost_4 / param_max_ang_z_;
    if (score > best_score)
    {
      best_score = score;
      best_traj = tmp_traj;
    }

    if (!flag)
    {
      continue;
    }
    v_trajs.push_back(tmp_traj);
  }

  // TODO: viz trajs
}

bool DWAPlanner::findBestPath(nav_msgs::Path &path)
{
}

void DWAPlanner::poseCB(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  msg_cur_pose_ = *msg;
  if (msg_global_path_.poses.size() == 0)
  {
    ROS_WARN("dwa_poseCB: Global path not set.");
    closest_pose_i_ = -1;
    return;
  }
  pthread_mutex_lock(&mutex_);
  // TODO: 更新路径
  int cur_closest_i;
  double cur_min_dist = 10000.0;
  double dist;
  // 全局路径中距离当前位置最近的点，非全局最优
  for (int i = closest_pose_i_ - 10 > 0 ? closest_pose_i_ - 10 : 0; i < msg_global_path_.poses.size() && i < cur_closest_i + 10; i++)
  {
    dist = std::sqrt(std::pow(msg->pose.position.x - msg_global_path_.poses[i].pose.position.x, 2) + std::pow(msg->pose.position.y - msg_global_path_.poses[i].pose.position.y, 2));
    if (dist < cur_min_dist)
    {
      cur_min_dist = dist;
      cur_closest_i = i;
    }
  }
  closest_pose_i_ = cur_closest_i;
  pthread_mutex_unlock(&mutex_);
}

void DWAPlanner::odomCB(const nav_msgs::Odometry::ConstPtr &msg)
{
  msg_cur_odom_ = *msg;
}

/**
 * @brief 将全局路径投影到局部地图中
 * 
 * @param msg 
 */
void DWAPlanner::globalPathCB(const nav_msgs::Path::ConstPtr &msg)
{
  if (msg_global_path_.poses.size() == msg->poses.size())
  {
    // 简单认为路径相同
    return;
  }
  pthread_mutex_lock(&mutex_path_);
  msg_global_path_ = *msg;
  pthread_mutex_unlock(&mutex_path_);
  closest_pose_i_ = -1;
  ROS_INFO("dwa_globalPathCB: new global path set.");
}

// 将 local costmap 投影到 map 中
void DWAPlanner::localMapCB(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
  msg_local_map_ = *msg;
  if (closest_pose_i_ < 0)
  {
    ROS_WARN("dwa_localMapCB: closest_pose in global path not found.");
    return;
  }
  tf::StampedTransform tf_g2l_;
  try
  {
    tf_listener_.waitForTransform(param_frame_globalcostmap_, msg->header.frame_id, msg->header.stamp, ros::Duration(param_timeout_));
    tf_listener_.lookupTransform(param_frame_globalcostmap_, msg->header.frame_id, msg->header.stamp, tf_g2l_);
  }
  catch (const tf::TransformException &ex)
  {
    ROS_ERROR("Error waiting transform in dwa: %s", ex.what());
    return;
  }

  pthread_mutex_lock(&mutex_path_);
  // 投影局部地图到全局坐标系下
  // TODO: 这是不对的
  tf::Pose tf_pose;
  tf::poseMsgToTF(msg->info.origin, tf_pose);
  tf_pose = tf_g2l_ * tf_pose;
  global_local_costmap_ = *msg;
  global_local_costmap_.header.frame_id = param_frame_globalcostmap_;
  geometry_msgs::Pose pose;
  tf::poseTFToMsg(tf_pose, pose);
  global_local_costmap_.info.origin = pose;

  // 更新 ref_global_path
  // geometry_msgs::Pose pose;
  // tf::Stamped<tf::Pose> tf_pose;
  // ref_global_path_.clear();
  // // ref_global_path_.resize(msg_global_path_.poses.size() - closest_pose_i_);
  // for (int i = closest_pose_i_; i < msg_global_path_.poses.size(); i++)
  // {
  //   // 投影全局路径到局部地图中
  //   tf::poseMsgToTF(msg_global_path_.poses[i].pose, tf_pose);
  //   tf::poseTFToMsg(tf_g2l_.inverse() * tf_pose, pose);
  //   int ix = std::floor((pose.position.x - msg->info.origin.position.x) / msg->info.resolution);
  //   int iy = std::floor((pose.position.y - msg->info.origin.position.y) / msg->info.resolution);
  //   if (ix < 0 || ix >= msg->info.width || iy < 0 || iy >= msg->info.height)
  //   {
  //     break;
  //   }
  //   ref_global_path_.push_back(pose);
  // }
  pthread_mutex_unlock(&mutex_path_);
}

void DWAPlanner::globalCostmapCB(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
  msg_global_map_ = *msg;
}

void DWAPlanner::cfgCB(const local_planner_dwa::DWAConfig &config, uint32_t level)
{
  param_time_period_ = config.time_period;
  param_acc_lin_limit_ = config.acc_lin_limit;
  param_acc_ang_limit_ = config.acc_ang_limit;
  param_obstacle_margin_ = config.obstacle_margin;
  param_footprint_radius_ = config.footprint_radius;
  param_max_vel_x_ = config.max_vel_x;
  param_max_ang_z_ = config.max_ang_z;
  param_time_period_ = config.timeout;
  ROS_INFO("new DWAConfig set.");
}

bool DWAPlanner::isOutofMap(const double x, const double y, const nav_msgs::OccupancyGrid &map)
{
  int i_x = (x - map.info.origin.position.x) / map.info.resolution,
      i_y = (y - map.info.origin.position.y) / map.info.resolution;
  return !(i_x >= 0 && i_x < map.info.width && i_y >= 0 && i_y < map.info.height);
}

int DWAPlanner::getCostofMap(const double x, const double y, const nav_msgs::OccupancyGrid &map)
{
  int i_x = (x - map.info.origin.position.x) / map.info.resolution,
      i_y = (y - map.info.origin.position.y) / map.info.resolution;
  if (!(i_x >= 0 && i_x < map.info.width && i_y >= 0 && i_y < map.info.height))
  {
    return 10000;
  }
  return map.data[i_y * map.info.width + i_x];
}