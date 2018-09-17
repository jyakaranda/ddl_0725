#include "imu_bridge_f.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "xrobot_imu_bridge");
  ros::NodeHandle nh, pnh("~");
  ImuBridge imu_bridge(nh, pnh);

  ros::spin();

  return 0;
}
