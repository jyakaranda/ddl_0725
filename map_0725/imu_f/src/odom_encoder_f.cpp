#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <math.h>
#include <sstream>
#include "std_msgs/String.h"

double vel_x = 0;
double vel_y = 0;
double vel_ang_z = 0;

double imu_diff_time;
double vel_diff_time;

void imu_callback( const sensor_msgs::Imu& imu)
{
  //callback every time the robot's angular velocity is received
	static ros::Time last_imu_time = ros::Time(0);
	ros::Time cur_imu_time = ros::Time::now();

	imu_diff_time = (cur_imu_time - last_imu_time).toSec();
	//this block is to filter out imu noise
  if(imu.angular_velocity.z > -0.001 && imu.angular_velocity.z < 0.001)
  {
    vel_ang_z = 0;
  }else
  {
    vel_ang_z = imu.angular_velocity.z;
  }
	last_imu_time = cur_imu_time;
}

void vel_callback( const geometry_msgs::Twist& vel) 
{
  //callback every time the robot's linear velocity is received
  static ros::Time last_vel_time = ros::Time(0);
  ros::Time cur_vel_time = ros::Time::now();

  vel_diff_time = (cur_vel_time - last_vel_time).toSec();
  vel_x = vel.linear.x;
  vel_y = vel.linear.y;
  vel_ang_z = vel.angular.z;
  last_vel_time = cur_vel_time;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odom_encoder");
  ros::NodeHandle n;
  ros::NodeHandle nh("~");
  ros::Subscriber sub = n.subscribe("/imu/vel_raw", 50, vel_callback);
  ros::Subscriber imu_sub = n.subscribe("/imu/data", 50, imu_callback);
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 50);
  //tf::TransformBroadcaster odom_broadcaster;

  double odom_linear_cali = 1.0;
  double odom_angular_cali = 1.0;
  nh.param<double>("odom_linear_scale_correction", odom_linear_cali, odom_linear_cali);
  nh.param<double>("odom_angular_scale_correction", odom_angular_cali, odom_angular_cali);

  double pos_x = 0;
  double pos_y = 0;
  double imu_z_delta = 0;
  ROS_INFO("Odom(Encoder) Node ready");
  ros::Rate loop_rate(15);
  while(n.ok())
  {
    ros::Time current_time = ros::Time::now();

    //calculate angular displacement  θ = ω * t
    double delta_z_ang = vel_ang_z * imu_diff_time;
    double delta_x = (vel_x * cos(imu_z_delta) - vel_y * sin(imu_z_delta)) * vel_diff_time;
    double delta_y = (vel_x * sin(imu_z_delta) + vel_y * cos(imu_z_delta)) * vel_diff_time;

    //calculate current position of the robot
    pos_x += delta_x * odom_linear_cali;
    pos_y += delta_y;
    imu_z_delta += delta_z_ang * odom_angular_cali;

    //calculate robot's heading in quarternion angle
    //ROS has a function to calculate yaw in quaternion angle
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(imu_z_delta);

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    //robot's position in x,y, and z
    odom_trans.transform.translation.x = pos_x;
    odom_trans.transform.translation.y = pos_y;
    odom_trans.transform.translation.z = 0;

    //robot's heading in quaternion
    odom_trans.transform.rotation = odom_quat;
    odom_trans.header.stamp = current_time;

    //publish robot's tf using odom_trans object

    //odom_broadcaster.sendTransform(odom_trans);

    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    //robot's position in x,y, and z
    odom.pose.pose.position.x = pos_x;
    odom.pose.pose.position.y = pos_y;
    odom.pose.pose.position.z = 0.0;
    
    //robot's heading in quaternion
    odom.pose.pose.orientation = odom_quat;

    //linear speed from encoders
    odom.twist.twist.linear.x = vel_x;
    odom.twist.twist.linear.y = vel_y;
    odom.twist.twist.linear.z = 0.0;
    
    //angular speed from IMU
    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = vel_ang_z;

    //include covariance matrix here

    odom_pub.publish(odom);

		ros::spinOnce();
		loop_rate.sleep();
  }
}
