#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include "waypoint_follower/libwaypoint_follower.h"

nav_msgs::Path raw_;
ros::Publisher pub_;
bool is_path;

void pathcallback(const nav_msgs::Path msg)
{
    raw_ = msg;
    is_path = true;
    //TODO 
    //path_hz should < pose_hz
}
void posecallback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    if (!is_path)
    {
        ROS_WARN("No pose subscriber");
        return;
    }

    double min = 10000000.0;
    int min_i = -1;
    nav_msgs::Path pub_msg;

    for (int i = 0; i < raw_.poses.size(); i++)
    {
        double distance = getPlaneDistance(raw_.poses.at(i).pose.position, msg->pose.position);
        if (distance < min)
        {
            min = distance;
            min_i = i;
        }
    }
    ROS_INFO("update path");
    pub_msg.header = raw_.header;
    for (int i = 0; min_i < raw_.poses.size(); i++, min_i++)
    {
        pub_msg.poses.push_back(raw_.poses.at(min_i));
    }
    ROS_INFO("raw_i = %d ,pub_size() = %d", min_i, pub_msg.poses.size());

    pub_.publish(pub_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pub_path");
    ros::NodeHandle nh;

    ros::Subscriber pose_sub = nh.subscribe("sim_pose", 100, posecallback);
    ros::Subscriber path_sub = nh.subscribe("/loaded_trajectory/recorded_path", 100, pathcallback);
    pub_ = nh.advertise<nav_msgs::Path>("final_waypoints", 100);
    ros::spin();
    return 0;
}