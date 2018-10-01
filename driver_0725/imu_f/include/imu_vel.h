#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/TwistStamped.h>

class ImuVel
{
private:
  ros::NodeHandle nh_, pnh_;
  static const double GRAVITY = -9.81; // [m/s/s]
  bool flag_first = false;
  ros::Time end, begin;
  // ROS pub/sub
  ros::Publisher pub;
  ros::Subscriber sub;

  double vel_x;
  double vel_y;

public:
  void vel_callback(const sensor_msgs::ImuConstPtr &imu_msg);

  ImuVel(ros::NodeHandle nh, ros::NodeHandle pnh);
  virtual ~ImuVel(void){};
};