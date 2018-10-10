#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include "RawImu.h"

class ImuBridge
{
  private:
    ros::NodeHandle nh_, pnh_;

    bool use_accelerometer_, use_gyroscope_, use_magnetometer_;
    bool use_mag_msg_;
    static const double GRAVITY = -9.81;  // [m/s/s]

    bool perform_calibration_, is_calibrated_;
    int calibration_samples_;
    std::map<std::string,double> acceleration_bias_, gyroscope_bias_;

    // Covariance
    double linear_acc_stdev_, angular_vel_stdev_, magnetic_field_stdev_;
    boost::array<double, 9> linear_acc_covar_;
    boost::array<double, 9> angular_vel_covar_;
    boost::array<double, 9> magnetic_field_covar_;

    // Used for mag scaling
    double mag_x_min_, mag_x_max_;  //  [T]
    double mag_y_min_, mag_y_max_;
    double mag_z_min_, mag_z_max_;

    // ROS pub/sub
    ros::Publisher imu_pub_;
    ros::Publisher mag_pub_;
    ros::Subscriber raw_sub_;

    // ROS services
    ros::ServiceServer imu_cal_srv_;

    // ROS member functions
    void rawCallback(const ros_arduino_msgs::RawImuConstPtr& raw_msg);
    bool calibrateCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    // Non-ROS member functions
    void fillRowMajor(boost::array<double, 9> & covar, double stdev);

  public:
    ImuBridge(ros::NodeHandle nh, ros::NodeHandle pnh);
    virtual ~ImuBridge(void){};
};

