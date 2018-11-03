/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "search_info_ros.h"

namespace astar_planner
{
SearchInfo::SearchInfo()
    : map_set_(false), start_set_(false), goal_set_(false), path_set_(false), closest_waypoint_index_(-1), obstacle_waypoint_index_(-1), start_waypoint_index_(-1), goal_waypoint_index_(-1), state_(""), upper_bound_distance_(-1)
{
  ros::NodeHandle pnh_("~");
  pnh_.param<std::string>("map_frame", map_frame_, "map");
  pnh_.param<int>("obstacle_detect_count", obstacle_detect_count_, 10);
  pnh_.param<int>("avoid_distance", avoid_distance_, 13);
  pnh_.param<double>("avoid_velocity_limit_mps", avoid_velocity_limit_mps_, 4.166);
  pnh_.param<double>("upper_bound_ratio", upper_bound_ratio_, 1.04);
  pnh_.param<bool>("avoidance", avoidance_, false);
  pnh_.param<bool>("change_path", change_path_, true);
}

SearchInfo::~SearchInfo()
{
}

double SearchInfo::calcPathLength(const nav_msgs::Path &lane, const int start_waypoint_index,
                                  const int goal_waypoint_index) const
{ //计算路径总距离(从start_waypoint到goal_waypoint)
  if (lane.poses.size() <= 1)
    return 0;

  // calulate the length of the path
  double dist_sum = 0;
  for (int i = start_waypoint_index; i < goal_waypoint_index; i++)
  {
    geometry_msgs::Pose p1 = lane.poses[i].pose;
    geometry_msgs::Pose p2 = lane.poses[i + 1].pose;

    dist_sum += astar_planner::calcDistance(p1.position.x, p1.position.y, p2.position.x, p2.position.y);
  }

  // return the path lengh
  return dist_sum;
}

void SearchInfo::mapCallback(const nav_msgs::OccupancyGridConstPtr &msg)
{
  map_ = *msg; //回调函数给Map赋值

  std::string map_frame = map_frame_;
  std::string ogm_frame = msg->header.frame_id;

  // Set transform between map frame and OccupancyGrid frame
  tf::StampedTransform map2ogm_frame;
  try
  {
    //得到ogm_frame在map_frame坐标系中的坐标
    tf_listener_.lookupTransform(map_frame, ogm_frame, ros::Time(0), map2ogm_frame);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    return;
  }

  // Set transform between map frame and the origin of OccupancyGrid
  tf::Transform map2ogm;
  geometry_msgs::Pose ogm_in_map = astar_planner::transformPose(map_.info.origin, map2ogm_frame);
  //map_info.origin 为ogm_frame坐标系下的起点
  //ogm_in_map 为map_frame坐标系下的起点
  tf::poseMsgToTF(ogm_in_map, map2ogm);
  ogm2map_ = map2ogm.inverse();
  ROS_INFO("Subcscribed map!");
  ROS_INFO("map_x = %d,map_y = %d", static_cast<int>(map_.info.width), static_cast<int>(map_.info.height));
  map_set_ = true;
}

void SearchInfo::currentPoseCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
  current_pose_.pose = msg->pose;
  current_pose_.header = msg->header;
  ROS_INFO("Subcscribed current pose!");
  double min = 10000000.0;
  int min_i = -1;
  nav_msgs::Path pub_msg;

  for (int i = 0; i < current_waypoints_.poses.size(); i++)
  {
    double distance = getPlaneDistance(current_waypoints_.poses.at(i).pose.position, msg->pose.position);
    if (distance < min)
    {
      min = distance;
      min_i = i;
    }
  }
  closest_waypoint_index_ = min;
  return;
}

void SearchInfo::currentVelocityCallback(const geometry_msgs::TwistStampedConstPtr &msg)
{
  current_velocity_mps_ = msg->twist.linear.x;
}

void SearchInfo::goalCallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
  if (!map_set_)
    return;

  ROS_INFO("Subcscribed goal pose!");

  std::string map_frame = map_frame_;
  std::string goal_frame = msg->header.frame_id;

  // Get transform of map to the frame of goal pose
  tf::StampedTransform map2world;
  try
  {
    tf_listener_.lookupTransform(map_frame, goal_frame, ros::Time(0), map2world);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    return;
  }

  // Set goal pose
  geometry_msgs::Pose pose_msg = msg->pose;
  goal_pose_global_.pose = astar_planner::transformPose(pose_msg, map2world);
  goal_pose_global_.header = msg->header;
  goal_pose_local_.pose = astar_planner::transformPose(goal_pose_global_.pose, ogm2map_);
  goal_pose_local_.header = goal_pose_global_.header;

  goal_set_ = true;

  // Get transform of map to the frame of start pose
  std::string start_frame = current_pose_.header.frame_id;
  tf::StampedTransform map2start_frame;
  try
  {
    tf_listener_.lookupTransform(map_frame_, start_frame, ros::Time(0), map2start_frame);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
    return;
  }

  // Set start pose
  start_pose_global_.pose = astar_planner::transformPose(current_pose_.pose, map2start_frame);
  start_pose_global_.header = current_pose_.header;
  start_pose_local_.pose = astar_planner::transformPose(start_pose_global_.pose, ogm2map_);
  start_pose_local_.header = start_pose_global_.header;

  start_set_ = true;
}

// get waypoints
void SearchInfo::waypointsCallback(const nav_msgs::PathConstPtr &msg)
{
  subscribed_waypoints_ = *msg;

  if (!path_set_)
  {
    current_waypoints_ = *msg;
    path_set_ = true;
  }
}

void SearchInfo::closestWaypointCallback(const std_msgs::Int32ConstPtr &msg)
{
  closest_waypoint_index_ = msg->data;
}

void SearchInfo::obstacleWaypointCallback(const nav_msgs::PathConstPtr &msg)
{
  // not always avoid AND current state is not avoidance
  if (!avoidance_)
  {
    ROS_WARN("current state is not OBSTACLE_AVOIDANCE");
    return;
  }

  for (int i = 0; i < msg->poses.size(); i++)
  {
    if (msg->poses.at(i).pose.position.z == -1)
    {
      obstacle_waypoint_index_ = i;
      break;
    }
  }

  // there are no obstacles
  if (obstacle_waypoint_index_ < 0 || closest_waypoint_index_ < 0 || current_waypoints_.poses.empty())
  {
    return;
  }

  // msg->data : local index
  // closest   : global index
  // Conver local index to global index
  //obstacle_waypoint_index_ = msg->data + closest_waypoint_index_;

  // Handle when detecting sensor noise as an obstacle
  static int prev_obstacle_waypoint_index = -1;
  static int obstacle_count = 0;
  int same_obstacle_threshold = 2;
  if (obstacle_waypoint_index_ >= prev_obstacle_waypoint_index - same_obstacle_threshold &&
      obstacle_waypoint_index_ <= prev_obstacle_waypoint_index + same_obstacle_threshold)
  {
    obstacle_count++;
  }
  else
  {
    obstacle_count = 1;
  }

  prev_obstacle_waypoint_index = obstacle_waypoint_index_;

  if (obstacle_count < obstacle_detect_count_)
    return;

  // not debug mode
  if (change_path_)
    obstacle_count = 0;

  // Decide start and goal waypoints for planning
  start_waypoint_index_ = obstacle_waypoint_index_ - avoid_distance_; //蔽障开始索引
  goal_waypoint_index_ = obstacle_waypoint_index_ + avoid_distance_;  //蔽障结束索引

  // Handle out of range
  if (start_waypoint_index_ < 0)
    start_waypoint_index_ = 0;

  // Handle out of range
  if (goal_waypoint_index_ >= static_cast<int>(getCurrentWaypoints().poses.size()))
    goal_waypoint_index_ = getCurrentWaypoints().poses.size() - 1;

  double original_path_length = calcPathLength(current_waypoints_, start_waypoint_index_, goal_waypoint_index_);
  upper_bound_distance_ = original_path_length * upper_bound_ratio_;

  // Do not avoid if (the obstacle is too close || current velocity is too fast)
  if (closest_waypoint_index_ + 1 > start_waypoint_index_)
  {
    ROS_WARN("The obstacle is too close!");
    return;
  }

  // apply velocity limit for avoiding
  if (current_velocity_mps_ > avoid_velocity_limit_mps_)
  {
    ROS_WARN("Velocity of the vehicle exceeds the avoid velocity limit");
    return;
  }

  // Set start pose
  start_pose_global_ = current_waypoints_.poses[start_waypoint_index_];
  start_pose_local_.pose = astar_planner::transformPose(start_pose_global_.pose, ogm2map_);
  start_set_ = true;

  // Set transit pose
  // TODO:
  double actual_car_width = 2.5; // [m]
  geometry_msgs::Pose relative_transit_pose;
  // TODO: always right avoidance ???
  relative_transit_pose.position.y -= actual_car_width;
  relative_transit_pose.orientation = current_waypoints_.poses[obstacle_waypoint_index_].pose.orientation;
  tf::Pose obstacle_pose_tf;
  tf::poseMsgToTF(current_waypoints_.poses[obstacle_waypoint_index_].pose, obstacle_pose_tf);

  transit_pose_global_.pose = astar_planner::transformPose(relative_transit_pose, obstacle_pose_tf);
  transit_pose_local_.pose = astar_planner::transformPose(transit_pose_global_.pose, ogm2map_);

  // Set goal pose
  goal_pose_global_ = current_waypoints_.poses[goal_waypoint_index_];
  goal_pose_local_.pose = astar_planner::transformPose(goal_pose_global_.pose, ogm2map_);

  goal_set_ = true;
}

void SearchInfo::stateCallback(const std_msgs::StringConstPtr &msg)
{
  state_ = msg->data;
}

void SearchInfo::reset()
{
  //map_set_ = false;
  start_set_ = false;
  goal_set_ = false;
  obstacle_waypoint_index_ = -1;
}
} // namespace astar_planner
