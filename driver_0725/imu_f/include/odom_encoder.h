#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <math.h>
#include <sstream>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

class OdomEncoder
{
public:
  OdomEncoder(ros::NodeHandle nh_, ros::NodeHandle pnh_);
  void callback(const sensor_msgs::Imu::ConstPtr &imu_msg, const geometry_msgs::TwistStamped::ConstPtr &twist_msg,const geometry_msgs::TwistStamped::ConstPtr &hall_msg);
  virtual ~OdomEncoder(void){};  
  

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, geometry_msgs::TwistStamped,geometry_msgs::TwistStamped> mySyncPolicy;
  message_filters::Synchronizer<mySyncPolicy> *sync_;
  message_filters::Subscriber<sensor_msgs::Imu> *imu_sub; //订阅不同的输入topic
  message_filters::Subscriber<geometry_msgs::TwistStamped> *twist_sub;
  message_filters::Subscriber<geometry_msgs::TwistStamped> *hall_sub;
  ros::Publisher odom_pub;

  tf::TransformBroadcaster odom_broadcaster;

  double pos_x;
  double pos_y;
  double imu_z_delta;
  double diff_time;
  ros::Time last_time, cur_time;

  bool flag_first = false;
};
