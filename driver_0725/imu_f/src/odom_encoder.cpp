#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <math.h>
#include <sstream>
#include "std_msgs/String.h"
#include "odom_encoder.h"

OdomEncoder::OdomEncoder(
    ros::NodeHandle nh_,
    ros::NodeHandle pnh_) : nh_(nh_),
                            pnh_(pnh_)
{
  twist_sub = new message_filters::Subscriber<geometry_msgs::TwistStamped>(nh_, "/imu/twist", 500);
  imu_sub = new message_filters::Subscriber<sensor_msgs::Imu>(nh_, "/imu/data", 500);
  hall_sub = new message_filters::Subscriber<geometry_msgs::TwistStamped>(nh_, "/wheel_circles", 500);
  sync_ = new message_filters::Synchronizer<mySyncPolicy>(mySyncPolicy(10), *imu_sub,*twist_sub,*hall_sub);
  sync_->registerCallback(boost::bind(&OdomEncoder::callback, this, _1, _2,_3));
  odom_pub = nh_.advertise<nav_msgs::Odometry>("/imu/odom", 500);
  ROS_INFO("Odom(Encoder) Node ready");
}

void OdomEncoder::callback(const sensor_msgs::Imu::ConstPtr &imu_msg, const geometry_msgs::TwistStamped::ConstPtr &twist_msg,const geometry_msgs::TwistStamped::ConstPtr &hall_msg)
{
  ros::Time time = twist_msg->header.stamp;
  //callback every time the robot's angular velocity is received
  if (!flag_first)
  {
    last_time = time;
    cur_time = last_time;
    flag_first = !flag_first;
  }
  else
  {
    last_time = cur_time;
    cur_time = time;
  }

  diff_time = (cur_time - last_time).toSec();
  //this block is to filter out imu noise
  double vel_ang_z = imu_msg->angular_velocity.z;
  
  double vel_z;
  double vel_ang_x;
  double vel_ang_y;
  
  if (vel_ang_z > -0.01 && vel_ang_z < 0.01)
    vel_ang_z = 0;
  
  double odom_linear_cali = 1.0;
  double odom_angular_cali = 1.0; 
  
  double delta_z_ang = vel_ang_z * diff_time;
  imu_z_delta += delta_z_ang * odom_angular_cali;
  
  double vel_x = hall_msg->twist.linear.x * cos(imu_z_delta);
  double vel_y = hall_msg->twist.linear.x * sin(imu_z_delta);
  
  if (vel_x > -0.01 && vel_x < 0.01)
    vel_x = 0;
  if (vel_y > -0.01 && vel_y < 0.01)
    vel_y = 0;
  //ROS_INFO("vel_diff_time: %f, vel_x: %f", vel_diff_time, vel_x);

  double delta_x = (vel_x * cos(imu_z_delta) - vel_y * sin(imu_z_delta)) * diff_time;
  double delta_y = (vel_x * sin(imu_z_delta) + vel_y * cos(imu_z_delta)) * diff_time;
  
  pos_x += delta_x * odom_linear_cali;
  pos_y += delta_y * odom_linear_cali;
  

  //calculate robot's heading in quarternion angle
  //ROS has a function to calculate yaw in quaternion angle
  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(imu_z_delta);
  ROS_INFO("imu_z_delta=%f", imu_z_delta);

  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";

  //robot's position in x,y, and z
  odom_trans.transform.translation.x = pos_x;
  odom_trans.transform.translation.y = pos_y;
  odom_trans.transform.translation.z = 0;

  //robot's heading in quaternion
  odom_trans.transform.rotation = odom_quat;
  odom_trans.header.stamp = cur_time;

  //publish robot's tf using odom_trans object
  odom_broadcaster.sendTransform(odom_trans);

  nav_msgs::Odometry odom;
  odom.header.stamp = cur_time;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_link";
  odom.pose.pose.position.x = pos_x;
  odom.pose.pose.position.y = pos_y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;
  odom.twist.twist.linear.x = vel_x;
  odom.twist.twist.linear.y = vel_y;
  odom.twist.twist.linear.z = 0;
  odom.twist.twist.angular.x = 0;
  odom.twist.twist.angular.y = 0;
  odom.twist.twist.angular.z = vel_ang_z;


  odom_pub.publish(odom);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "odom_encoder");
  ros::NodeHandle nh, pnh("~");
  OdomEncoder OdomEncoder(nh, pnh);
  ros::spin();
  return 0;
}
