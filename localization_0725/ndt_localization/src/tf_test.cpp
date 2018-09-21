/**
 * @brief 
 * 
 * @file tf_test.cpp
 * @author your name
 * @date 2018-09-20
 */
// #define TF_EULER_DEFAULT_ZYX
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <ros/duration.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tf_test");
  tf::TransformBroadcaster broadcaster;
  ros::Duration d(1);

  while (ros::ok())
  {
    tf::Quaternion q(0.785, 0, 0);
    double yaw, pitch, roll;
    q.setRPY(0, 0, 0.785);
    tf::Matrix3x3(q).getEulerYPR(yaw, pitch, roll);

    ROS_INFO("yaw: %f, pitch: %f, roll: %f", yaw, pitch, roll);
    broadcaster.sendTransform(tf::StampedTransform(tf::Transform(q, tf::Vector3(0, 1, 0)), ros::Time::now(), "/map", "/odom"));
    d.sleep();
  }

  ros::spin();
  return 0;
}