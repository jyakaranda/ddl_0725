#include <ros/ros.h>
#include <ros/duration.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>

ros::Publisher pub_costmap;
nav_msgs::OccupancyGridPtr msg_costmap;
double param_width;
double param_height;
double param_resolution;

void pointsCB(const sensor_msgs::PointCloud2ConstPtr &msg)
{
}

void mapCB(const sensor_msgs::PointCloud2ConstPtr &msg)
{
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gen_costmap");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    pnh.param<double>("width", param_width, 50.);
    pnh.param<double>("height", param_height, 50.);
    pnh.param<double>("resolution", param_resolution, 0.1);

    ros::Subscriber sub_points = nh.subscribe("/lslidar_point_cloud", 10, pointsCB);
    // ros::Subscriber sub_map = nh.subscribe("/pc_map", 1, mapCB);
    pub_costmap = nh.advertise<nav_msgs::OccupancyGrid>("/costmap", 1);

    return 0;
}