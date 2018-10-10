
#include "odom_encoder.h"

void OdomEncoder::imu_callback(const geometry_msgs::Twist &imu_msg)
{
  
  vel_ang_z = imu_msg.angular.z;
  
  if (vel_ang_z < 0.001 && vel_ang_z > -0.001)
    vel_ang_z = 0;

  //vel_x = imu_msg.linear.x;
  //vel_y = imu_msg.linear.y;
  
  //this block is to filter out imu noise
  
  //if (vel_x < 0.001 && vel_x > -0.001)
  //  vel_x = 0;
  //if (vel_y < 0.001 && vel_y > -0.001)
  //  vel_y = 0;
}

void OdomEncoder::hall_callback(const geometry_msgs::Twist &hall_msg)
{
  vel_x = hall_msg.linear.x;
  vel_y = 0;
}

OdomEncoder::OdomEncoder(ros::NodeHandle nh, ros::NodeHandle pnh) : nh_(nh), pnh_(pnh)
{
  imu_sub = nh_.subscribe("/imu", 50, &OdomEncoder::imu_callback, this);
  hall_sub = nh_.subscribe("/wheel_circles", 50, &OdomEncoder::hall_callback,this);
  odom_pub = nh_.advertise<nav_msgs::Odometry>("/odom", 50);

  ROS_INFO("Odom(Encoder) Node ready");
  //tf::TransformBroadcaster odom_broadcaster;
  double odom_linear_cali = 1.0;
  double odom_angular_cali = 1.0;
  nh_.param<double>("odom_linear_scale_correction", odom_linear_cali, odom_linear_cali);
  nh_.param<double>("odom_angular_scale_correction", odom_angular_cali, odom_angular_cali);
  ros::Rate loop_rate(45);
  while (nh.ok())
  {
    if(!flag_first)
    {
      last_time = ros::Time::now();
      current_time = last_time;
      flag_first = !flag_first;
    }
    else
    {
      last_time = current_time;
      current_time = ros::Time::now();
    }

    diff_time = (current_time-last_time).toSec();
    //calculate angular displacement  θ = ω * t
    double delta_z_ang = vel_ang_z * diff_time;
    ang_z += delta_z_ang * odom_angular_cali;

    double delta_x = (vel_x * cos(ang_z) - vel_y * sin(ang_z)) * diff_time;
    double delta_y = (vel_x * sin(ang_z) + vel_y * cos(ang_z)) * diff_time;
    //calculate current position of the robot
    pos_x += delta_x * odom_linear_cali;
    pos_y += delta_y *odom_linear_cali;
    

    //calculate robot's heading in quarternion angle
    //ROS has a function to calculate yaw in quaternion angle
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(ang_z);

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

    odom_broadcaster.sendTransform(odom_trans);

    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "base_link";
    //odom.child_frame_id = "base_link";
    //robot's position in x,y, and z
    odom.pose.pose.position.x = pos_x;
    odom.pose.pose.position.y = pos_y;
    odom.pose.pose.position.z = 0.0;

    //robot's heading in quaternion
    odom.pose.pose.orientation = odom_quat;

    //linear speed from encoders
    odom.twist.twist.linear.x = vel_x;
    odom.twist.twist.linear.y = vel_y;
    odom.twist.twist.linear.z = 0;

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

int main(int argc, char **argv)
{

  ros::init(argc, argv, "odom_encoder");
  ros::NodeHandle nh,pnh("~");
  OdomEncoder OdomEncoder(nh,pnh);
  return 0;
}
