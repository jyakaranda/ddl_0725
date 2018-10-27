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

#include "pure_pursuit_core.h"

namespace waypoint_follower
{
// Constructor
PurePursuitNode::PurePursuitNode()
    : pnh_("~"), pp_(), LOOP_RATE_(30), is_waypoint_set_(false), is_pose_set_(false), const_lookahead_distance_(4.0), const_velocity_(1.0), lookahead_distance_ratio_(2.0), minimum_lookahead_distance_(6.0)
{
  initForROS();

  // initialize for PurePursuit
  pp_.setLinearInterpolationParameter(is_linear_interpolation_);
}

// Destructor
PurePursuitNode::~PurePursuitNode()
{
}

void PurePursuitNode::initForROS()
{
  // ros parameter settings
  pnh_.param("is_linear_interpolation", is_linear_interpolation_, bool(true)); //是否进行线性插值
  // ROS_INFO_STREAM("is_linear_interpolation : " << is_linear_interpolation_);
  pnh_.param("const_lookahead_distance", const_lookahead_distance_, 4.0);
  pnh_.param("const_velocity", const_velocity_, 1.0);
  pnh_.param("lookahead_distance_ratio", lookahead_distance_ratio_, 2.0);
  pnh_.param("minimum_lookahead_distance", minimum_lookahead_distance_, 6.0);

  // setup subscriber
  sub1_ = nh_.subscribe("final_waypoints", 10, &PurePursuitNode::callbackFromWayPoints, this); //订阅路径规划出来的路径点
  sub2_ = nh_.subscribe("sim_pose", 10, &PurePursuitNode::callbackFromCurrentPose, this);  //订阅机器人发布的姿态

  // setup publisher
  pub1_ = nh_.advertise<geometry_msgs::TwistStamped>("ctrl_cmd", 10); //发布机器人运动信息
  pub3_ = nh_.advertise<geometry_msgs::Pose>("/cur_pose", 10); //发布机器人当前姿态
  pub11_ = nh_.advertise<visualization_msgs::Marker>("next_waypoint_mark", 0);
  pub12_ = nh_.advertise<visualization_msgs::Marker>("next_target_mark", 0);
  pub13_ = nh_.advertise<visualization_msgs::Marker>("search_circle_mark", 0);
  pub14_ = nh_.advertise<visualization_msgs::Marker>("line_point_mark", 0); // debug tool
  pub15_ = nh_.advertise<visualization_msgs::Marker>("trajectory_circle_mark", 0);
  pub16_ = nh_.advertise<std_msgs::Float32>("angular_gravity", 0);
  pub17_ = nh_.advertise<std_msgs::Float32>("deviation_of_current_position", 0);
  // pub7_ = nh.advertise<std_msgs::Bool>("wf_stat", 0);
}

void PurePursuitNode::run()
{
  ROS_INFO_STREAM("pure pursuit start");
  ros::Rate loop_rate(LOOP_RATE_);
  while (ros::ok())
  {
    ros::spinOnce();
    //当订阅话题都收到数据时，开始pure_pursuit算法
    if (!is_pose_set_ || !is_waypoint_set_)
    {
      ROS_WARN("Necessary topics are not subscribed yet ... ");
      loop_rate.sleep();
      continue;
    }
    current_linear_velocity_ = const_velocity_;
    pp_.setCurrentVelocity(current_linear_velocity_);

    pp_.setLookaheadDistance(computeLookaheadDistance()); //计算机器人的预瞄距离，主要和无人车当前的速度，转向角度有关
    pp_.setMinimumLookaheadDistance(minimum_lookahead_distance_);

    double kappa = 0;
    bool can_get_curvature = pp_.canGetCurvature(&kappa); //计算机器人的曲率
    publishTwistStamped(can_get_curvature, kappa);

    // for visualization with Rviz
    pub11_.publish(displayNextWaypoint(pp_.getPoseOfNextWaypoint()));
    pub13_.publish(displaySearchRadius(pp_.getCurrentPose().position, pp_.getLookaheadDistance()));
    pub12_.publish(displayNextTarget(pp_.getPoseOfNextTarget()));
    pub15_.publish(displayTrajectoryCircle(
        waypoint_follower::generateTrajectoryCircle(pp_.getPoseOfNextTarget(), pp_.getCurrentPose())));
    std_msgs::Float32 angular_gravity_msg;
    angular_gravity_msg.data = computeAngularGravity(computeCommandVelocity(), kappa);
    pub16_.publish(angular_gravity_msg);

    publishDeviationCurrentPosition(pp_.getCurrentPose().position, pp_.getCurrentWaypoints());

    is_pose_set_ = false;
    loop_rate.sleep();
  }
}

void PurePursuitNode::publishTwistStamped(const bool &can_get_curvature, const double &kappa) const
{
  geometry_msgs::TwistStamped ts;
  ts.header.stamp = ros::Time::now();
  ts.twist.linear.x = can_get_curvature ? computeCommandVelocity() : 0;
  ts.twist.angular.z = can_get_curvature ? kappa * ts.twist.linear.x : 0;
  pub1_.publish(ts);

  pub3_.publish(pp_.getCurrentPose());
}

double PurePursuitNode::computeLookaheadDistance() const
{

  double maximum_lookahead_distance = current_linear_velocity_ * 10; //最大预瞄距离为当前速度的10倍
  double ld = current_linear_velocity_ * lookahead_distance_ratio_;  //设置预瞄距离，线性比率为2

  return ld < minimum_lookahead_distance_ ? minimum_lookahead_distance_ : ld > maximum_lookahead_distance ? maximum_lookahead_distance : ld;
}

double PurePursuitNode::computeCommandVelocity() const
{
  return command_linear_velocity_;
}

double PurePursuitNode::computeAngularGravity(double velocity, double kappa) const
{
  const double gravity = 9.80665;
  return (velocity * velocity) / (1.0 / kappa * gravity);
}


void PurePursuitNode::publishDeviationCurrentPosition(const geometry_msgs::Point &point,
                                                      const std::vector<geometry_msgs::PoseStamped> &waypoints) const
{
  // Calculate the deviation of current position from the waypoint approximate line

  if (waypoints.size() < 3)
  {
    return;
  }

  double a, b, c;
  bool linear_flag_in =
      getLinearEquation(waypoints.at(2).pose.position, waypoints.at(1).pose.position, &a, &b, &c);

  std_msgs::Float32 msg;
  msg.data = getDistanceBetweenLineAndPoint(point, a, b, c);

  pub17_.publish(msg);
}

void PurePursuitNode::callbackFromCurrentPose(const geometry_msgs::PoseStampedConstPtr &msg)
{
  pp_.setCurrentPose(msg->pose);
  is_pose_set_ = true;
}

void PurePursuitNode::callbackFromWayPoints(const nav_msgs::PathConstPtr &msg)
{
    if (!msg->poses.empty())
      command_linear_velocity_ = current_linear_velocity_;
    else
      command_linear_velocity_ = 0;

    pp_.setCurrentWaypoints(msg->poses);
    is_waypoint_set_ = true;
}



} // namespace waypoint_follower
