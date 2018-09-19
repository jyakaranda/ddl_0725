#include <pf_localization/pf_localization.h>
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "localization_node");

    ROS_INFO("Starting localization node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    localization::pf_localization pf(nh, pnh);
    if(pf.init()){
        ros::spin();
    }

    return 0;
}