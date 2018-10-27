#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <math.h>
#include <sstream>
#include <std_msgs/String.h>
#define PI 3.1415926
class OdomEncoder
{
  public:
    OdomEncoder(ros::NodeHandle nh_, ros::NodeHandle pnh_);
    void imu_callback(const geometry_msgs::Twist &imu_msg);
    void hall_callback(const geometry_msgs::TwistStamped &hall_msg);
    virtual ~OdomEncoder(void){};

  private:
    ros::NodeHandle nh_, pnh_;
    ros::Subscriber imu_sub;
    ros::Subscriber hall_sub;
    ros::Publisher odom_pub;

    tf::TransformBroadcaster odom_broadcaster;

    double vel_x;
    double vel_y;
    double vel_ang_z;

    double pos_x;
    double pos_y;
    double ang_z;
    double odom_linear_cali ;
    double odom_angular_cali;

    ros::Time last_time, current_time;
    double diff_time;

    bool flag_first = false;
};