#include <ros/ros.h>
#include <ethernet_driver/ethernet_driver.h>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "ethernet_node");
    ros::NodeHandle n;
    ros::NodeHandle np("~");
    ethernet_driver::EthernetDriver ethernet(n, np);

    if (!ethernet.initialize())
    {
        ROS_ERROR("cannot initialize EthernetDriver!!!");
        return 0;
    }

    while (ros::ok() && ethernet.polling())
    {
        ros::spinOnce();
    }
    ros::spin();

    return 0;
}