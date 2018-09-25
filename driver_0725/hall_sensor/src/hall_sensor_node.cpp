#include <ros/ros.h>
#include <ros/duration.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/TwistStamped.h>

#define PI 3.1415926

bool pre_pre_flag;
bool pre_flag;
bool cur_flag;
int count;
int param_mag_num;
double param_rate;
double param_wheel_radius;
std::string param_wheel_frame;
geometry_msgs::TwistStamped msg_twist;
ros::Duration d;
ros::Time start;
ros::Time now;
bool init;
ros::Publisher pub_circles;

void hallSensorCB(const std_msgs::Bool::ConstPtr &msg)
{
  if (!init)
  {
    init = true;
    start = ros::Time::now();
    return;
  }
  cur_flag = msg->data;
  if (pre_flag != cur_flag)
  {
    count++;
    // ROS_INFO("count: %d", count);
  }

  pre_pre_flag = pre_flag;
  pre_flag = cur_flag;
  now = ros::Time::now();
  if ((now - start) > d)
  {
    msg_twist.header.stamp = now;
    msg_twist.twist.linear.x = count * PI * param_wheel_radius / (param_mag_num * (now - start).toSec());
    pub_circles.publish(msg_twist);
    ROS_INFO("count: %d", count);
    start = now;
    count = 0;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hall_sensor_node");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  ros::Subscriber sub_hall_sensor;

  pnh.param<int>("mag_num", param_mag_num, 1);
  pnh.param<double>("rate", param_rate, 0.1);
  pnh.param<double>("wheel_radius", param_wheel_radius, 0.075);
  pnh.param<std::string>("wheel_frame", param_wheel_frame, std::string("/bl_wheel"));
  sub_hall_sensor = nh.subscribe("/hall_sensor", 1000, hallSensorCB);
  pub_circles = nh.advertise<geometry_msgs::TwistStamped>("/wheel_circles", 10);
  d.fromSec(param_rate);
  msg_twist.header.frame_id = param_wheel_frame;

  ros::spin();
  return 0;
}