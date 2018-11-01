#include <get_2d_map/loam_to_2d.h>

namespace map_0725
{
LoamTo2d::LoamTo2d(ros::NodeHandle &nh, ros::NodeHandle &pnh) : nh_(nh), pnh_(pnh), laser_cloud_raw(new pcl::PointCloud<pcl::PointXYZI>()), laser_cloud_map(new pcl::PointCloud<pcl::PointXYZI>())
{
}

bool LoamTo2d::init()
{
    ROS_INFO("init LoamTo2d");
    if (!load_params())
    {
        ROS_ERROR("LoamTo2d cannot load params");
        return false;
    }

    lock = false;
    sub_loam = nh_.subscribe<sensor_msgs::PointCloud2>("/loam_velodyne", 5, boost::bind(&LoamTo2d::pointcloud_callback, this, _1));
    sub_odom = nh_.subscribe<nav_msgs::Odometry>("/integrated_to_init", 100, boost::bind(&LoamTo2d::odomCB, this, _1));
    pub_3d_map = nh_.advertise<sensor_msgs::PointCloud2>("/loam_3d_map", 5);
    pub_proj_map = nh_.advertise<nav_msgs::OccupancyGrid>("/loam_2d_grid", 1);
    pub_odom = nh_.advertise<nav_msgs::Odometry>("/odom", 1);

    grid_map_.info.height = param_height;
    grid_map_.info.width = param_width;
    grid_map_.info.resolution = param_resolution;
    grid_map_.info.origin.position.x = 0;
    grid_map_.info.origin.position.y = 0;
    grid_map_.info.origin.position.z = 0;
    grid_map_.info.origin.orientation.x = 0;
    grid_map_.info.origin.orientation.y = 0;
    grid_map_.info.origin.orientation.z = 0;
    grid_map_.info.origin.orientation.w = 1;
    grid_map_.header.frame_id = param_global_frame;
    grid_map_.data.resize(ceil(grid_map_.info.width * grid_map_.info.height / grid_map_.info.resolution));

    ROS_INFO("init OK");
    return true;
}

bool LoamTo2d::load_params()
{
    // grid map meta info
    pnh_.param("resolution", param_resolution, 0.05);
    pnh_.param("width", param_width, 10.);
    pnh_.param("height", param_height, 10.);
    pnh_.param("global_frame", param_global_frame, std::string("/map"));
    pnh_.param("odom_frame", param_odom_frame, std::string("/odom"));
    pnh_.param("laser_frame", param_laser_frame, std::string("/laser"));
    pnh_.param("robot_base_frame", param_base_frame, std::string("/base_link"));

    return true;
}

void LoamTo2d::pointcloud_callback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    if (lock)
    {
        return;
    }

    if(pub_3d_map.getNumSubscribers() == 0){
        return;
    }

    lock = true;

    laser_cloud_raw->clear();
    laser_cloud_map->clear();
    pcl::fromROSMsg(*msg, *laser_cloud_raw);

    pcl::PointXYZI tmp;
    size_t pointNum = laser_cloud_raw->size();
    for (int i = 0; i < pointNum; i++)
    {
        raw_to_map(laser_cloud_raw->points[i], tmp);
        laser_cloud_map->push_back(tmp);
    }
    ROS_INFO("/loam_velodyne callback, point num: %d", laser_cloud_map->size());
    sensor_msgs::PointCloud2 mapMsg;
    pcl::toROSMsg(*laser_cloud_map, mapMsg);
    mapMsg.header.frame_id = param_global_frame;
    mapMsg.header.stamp = ros::Time::now();
    pub_3d_map.publish(mapMsg);

    // TODO: publish projected grid map
    lock = false;
}

void LoamTo2d::odomCB(const nav_msgs::OdometryConstPtr &msg){
    msg_odom.header.stamp = msg->header.stamp;
    msg_odom.header.frame_id = param_odom_frame;
    msg_odom.pose.pose.position.x = msg->pose.pose.position.z;
    msg_odom.pose.pose.position.y = msg->pose.pose.position.x;
    // msg_odom.pose.pose.position.z = msg->pose.pose.position.y;
    msg_odom.pose.pose.position.z = 0;
    double pitch, roll, yaw;
    tf::Matrix3x3(tf::Quaternion(msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.x, msg->pose.pose.orientation.w)).getEulerYPR(yaw, pitch, roll);
    tf::Quaternion q(0, 0, roll);
    msg_odom.pose.pose.orientation.w = q.getW();
    msg_odom.pose.pose.orientation.x = q.getX();
    msg_odom.pose.pose.orientation.y = q.getY();
    msg_odom.pose.pose.orientation.z = q.getZ();

    pub_odom.publish(msg_odom);

    trans_laser_odom.stamp_ = msg->header.stamp;
    trans_laser_odom.setRotation(tf::Quaternion(msg_odom.pose.pose.orientation.x, msg_odom.pose.pose.orientation.y, msg_odom.pose.pose.orientation.z, msg_odom.pose.pose.orientation.w));
    trans_laser_odom.setOrigin(tf::Vector3(msg_odom.pose.pose.position.x, msg_odom.pose.pose.position.y, msg_odom.pose.pose.position.z));
    trans_laser_odom.frame_id_ = param_odom_frame;
    trans_laser_odom.child_frame_id_ = param_laser_frame;

    tf_broadcaster.sendTransform(trans_laser_odom);
}

void LoamTo2d::raw_to_map(const pcl::PointXYZI &in, pcl::PointXYZI &out)
{
    out.x = in.z;
    out.y = in.x;
    out.z = in.y;
    out.intensity = in.intensity;
}

} // namespace map_0725