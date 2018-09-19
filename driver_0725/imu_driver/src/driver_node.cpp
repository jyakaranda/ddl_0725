#include "imu_driver/itg3205.h"
#include "imu_driver/adxl345.h"
#include <ros/ros.h>
#include <ros/time.h>
#include <ros/duration.h>
#include "imu_driver/RawImu.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "imu_driver_node");
  ROS_INFO("Start driver_node.");
  ITG3205 gyro_node(1);
  if (!gyro_node.openITG3205())
  {
    ROS_ERROR("Failed to open ITG3205.");
    return -1;
  }
  if (!gyro_node.init())
  {
    ROS_ERROR("Failed to init ITG3205");
    return -1;
  }

  ADXL345 accel_node(1, 16);
  if (!accel_node.openADXL345())
  {
    ROS_ERROR("Failed to open ADXL345");
    return -1;
  }
  if (!accel_node.init())
  {
    ROS_ERROR("Failed to init ADXL345");
    return -1;
  }

  ROS_INFO("HW579 init ok.");

  ros::NodeHandle nh;
  ros::Publisher pub_raw_imu = nh.advertise<ros_arduino_msgs::RawImu>("/raw_imu", 50);

  ros_arduino_msgs::RawImu pub_msg_imu_raw;

  ros::Duration duration(0.02);
  int count = 0;
  while (ros::ok())
  {
    imu_data gyro = gyro_node.getGyro();
    imu_data temp = gyro_node.getTemp();
    usleep(5);
    imu_data accel = accel_node.getAccel();
    if (count++ % 30 == 0)
    {
      ROS_INFO("gyro");
      ROS_INFO("x: %f, y: %f, z: %f; temp: %.2f", gyro.gyro.x, gyro.gyro.y, gyro.gyro.z, temp.temp);
      ROS_INFO("accel");
      ROS_INFO("x: %f, y: %f, z: %f", accel.accel.x, accel.accel.y, accel.accel.z);
    }

    pub_msg_imu_raw.header.frame_id = "/imu_link";
    pub_msg_imu_raw.header.stamp = ros::Time::now();
    pub_msg_imu_raw.accelerometer = true;
    pub_msg_imu_raw.gyroscope = true;
    pub_msg_imu_raw.raw_linear_acceleration.x = accel.accel.x;
    pub_msg_imu_raw.raw_linear_acceleration.y = accel.accel.y;
    pub_msg_imu_raw.raw_linear_acceleration.z = accel.accel.z;
    pub_msg_imu_raw.raw_angular_velocity.x = gyro.gyro.x;
    pub_msg_imu_raw.raw_angular_velocity.y = gyro.gyro.y;
    pub_msg_imu_raw.raw_angular_velocity.z = gyro.gyro.z;
    pub_raw_imu.publish(pub_msg_imu_raw);

    duration.sleep();
    ros::spinOnce();
  }

  return 0;
}