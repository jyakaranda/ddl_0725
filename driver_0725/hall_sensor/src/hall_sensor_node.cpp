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
double param_Q;
double param_R;
geometry_msgs::TwistStamped msg_twist;
ros::Duration d;
ros::Time start;
ros::Time now;
bool init;
ros::Publisher pub_circles;
double pre_x, pre_p, pre_pre_x;
double cur_x, cur_p;

/**
 * @brief One dimensional kalman filter, simple assume that velocity is constant.
 * 
 * @param pre_x 
 * @param pre_p 
 * @param z 
 * @param cur_x 
 * @param cur_p 
 */
void kalmanFilter(const double pre_x, const double pre_p, const double z, double &cur_x, double cur_p)
{
  double tmp_x = pre_x;
  double tmp_p = pre_p + param_Q;
  double y = z - tmp_x;
  double K = tmp_p / (tmp_p + param_R);
  cur_x = tmp_x + K * y;
  cur_p = (1 - K) * tmp_p;
}

void hallSensorCB(const std_msgs::Bool::ConstPtr &msg)
{
  now = ros::Time::now();
  if (!init)
  {
    init = true;
    pre_x = 0;
    pre_p = param_Q / 10.;
    start = now;
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
  if ((now - start) > d)
  {
    count /= 2;
    double raw_vel = count * 2 * PI * param_wheel_radius / (param_mag_num * (now - start).toSec());
    kalmanFilter(pre_x, pre_p, raw_vel, cur_x, cur_p);
    if (std::fabs(pre_pre_x + pre_x + cur_x) < 0.0001)
    {
      cur_x = 0.;
    }
    else
    {
      // Smooth filter, in the premise that velocity won't change drastically.
      cur_x = (std::pow(pre_pre_x, 2) + std::pow(pre_x, 2) + std::pow(cur_x, 2)) / (pre_pre_x + pre_x + cur_x);
    }
    msg_twist.header.stamp = now;
    msg_twist.twist.linear.x = cur_x;
    msg_twist.twist.linear.y = raw_vel;
    msg_twist.twist.angular.x = cur_x;
    msg_twist.twist.angular.y = count;
    pub_circles.publish(msg_twist);
    ROS_INFO("count: %d", count);
    pre_pre_x = pre_x;
    pre_x = cur_x;
    pre_p = cur_p;
    start = ros::Time::now();
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
  // TODO need to be adjust for better filtering, just assigned based on experience for now.
  pnh.param<double>("Q", param_Q, 1.5);
  pnh.param<double>("R", param_R, 1.0);
  param_Q = std::pow(param_mag_num * param_Q / 3.0, 2);
  param_R = std::pow(param_mag_num * param_R / 3.0, 2);
  sub_hall_sensor = nh.subscribe("/hall_sensor", 1000, hallSensorCB);
  pub_circles = nh.advertise<geometry_msgs::TwistStamped>("/wheel_circles", 10);
  d.fromSec(param_rate);
  msg_twist.header.frame_id = param_wheel_frame;

  ros::spin();
  return 0;
}