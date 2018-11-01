#include <get_2d_map/loam_to_2d.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "main");
    ROS_INFO("get_2d_map");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    map_0725::LoamTo2d node(nh, pnh);

    if (node.init())
    {
        ros::spin();
    }

    return 0;
}