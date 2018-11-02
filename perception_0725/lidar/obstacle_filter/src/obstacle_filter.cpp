#include "obstacle_filter.h"

ObstacleFilter::ObstacleFilter() : pnh_("~")
{
}

ObstacleFilter::~ObstacleFilter()
{
}

bool ObstacleFilter::init()
{
    sub_raw_pc_ = nh_.subscribe<sensor_msgs::PointCloud2>("/lslidar_point_cloud", 10, boost::bind(&ObstacleFilter::rawPointCloudCB, this, _1));
    pub_obstacle_ = nh_.advertise<sensor_msgs::PointCloud2>("/obstacle_pc", 2);
}

void ObstacleFilter::rawPointCloudCB(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
}

void ObstacleFilter::run()
{
}
