#ifndef __USER_PROTOCOL__
#define __USER_PROTOCOL__

#include <geometry_msgs/Pose.h>

typedef struct
{
  double x, y, z;
  double pitch, roll, yaw;
} pose;

bool geometryPose2Pose(const geometry_msgs::Pose from, pose &to)
{
  to.x = from.position.x;
  to.y = from.position.y;
  to.z = from.position.z;
  tf::Matrix3x3(tf::Quaternion(from.orientation.x, from.orientation.y, from.orientation.z, from.orientation.w))
      .getEulerYPR(to.yaw, to.pitch, to.roll);

  return true;
}

bool pose2GeometryPose(geometry_msgs::Pose to, const pose from)
{
  to.position.x = from.x;
  to.position.y = from.y;
  to.position.z = from.z
  to.orientation = tf::createQuaternionMsgFromRollPitchYaw(from.roll, from.pitch, from.yaw);

  return true;
}

#endif