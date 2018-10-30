#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <ros/duration.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <dynamic_reconfigure/server.h>
#include <gen_costmap/GenCostmapConfig.h>

ros::Publisher pub_costmap_;
nav_msgs::OccupancyGrid::Ptr msg_costmap_;
tf::TransformListener tf_listener_;
tf::StampedTransform tf_base2laser_;
tf::StampedTransform tf_map2base_;

int param_frame_num_;
double param_confidence_;
double param_xh_;
double param_yh_;
double param_zh_;
double param_xl_;
double param_yl_;
double param_zl_;
double param_resolution_;
std::string param_frame_map_;
std::string param_frame_base_;
std::string param_frame_laser_;
int g_width_;
int g_height_;

void pointsCB(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    setOccupancyGrid(msg_costmap_);
}

void cfgCB(const gen_costmap::GenCostmapConfig &config, uint32_t level)
{
    param_frame_num_ = config.frame_num;
    param_confidence_ = config.confidence;
    param_xh_ = config.xh;
    param_yh_ = config.yh;
    param_zh_ = config.zh;
    param_xl_ = config.xl;
    param_yl_ = config.yl;
    param_zl_ = config.zl;
    param_resolution_ = config.resolution;
    ROS_INFO("new GenCostmapConfig set.");
}

void setOccupancyGrid(nav_msg_::OccupancyGrid::Ptr og)
{
    try
    {
        tf_listener_.waitForTransform(param_map_frame_, param_base_frame_, ros::Time(0), ros::Duration(0.1));
        tf_map2base_ = tf_listener_.lookupTransform(param_map_frame_, param_base_frame_, ros::Time(0));
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("Error waiting for transform: %s", ex.what());
    }

    g_width_ = std::ceil((param_xh_ - param_xl_) / param_resolution_);
    g_height_ = std::ceil((param_yh_ - param_yl_) / param_resolution_);
    og->header.frame_id = param_frame_map_;
    og->info.resolution = param_resolution;
    og->info.width = g_width_;
    og->info.height = g_height_;
    og->data = std::vector<int>(g_width * g_height, 0);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gen_costmap");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    pnh.param<int>("frame_num", param_frame_num_, 1);
    pnh.param<double>("confidence", param_confidence_, 0.5);
    pnh.param<double>("resolution", param_resolution_, 0.1);
    pnh.param<double>("xh", param_xh_, 50.);
    pnh.param<double>("yh", param_yh_, 10.);
    pnh.param<double>("zh", param_zh_, 2.);
    pnh.param<double>("xl", param_xl_, -50.);
    pnh.param<double>("yl", param_yl_, -10.);
    pnh.param<double>("zl", param_zl_, -2.);
    pnh.param<std::string>("map_frame", param_frame_map_, std::string("map"));
    pnh.param<std::string>("base_frame", param_frame_base_, std::string("/base_link"));
    pnh.param<std::string>("laser_frame", param_frame_laser_, std::string("/laser"));

    ros::Subscriber sub_points = nh.subscribe("/lslidar_point_cloud", 10, pointsCB);
    // ros::Subscriber sub_map = nh.subscribe("/pc_map", 1, mapCB);
    pub_costmap_ = nh.advertise<nav_msgs::OccupancyGrid>("/costmap", 1);
    dynamic_reconfigure::Server<gen_costmap::GenCostmapConfig> cfg_server;
    dynamic_reconfigure::Server<gen_costmap::GenCostmapConfig>::CallbackType cfg_callback = boost::bind(&cfgCB, _1, _2);
    cfg_server.setCallback(cfg_callback);

    try
    {
        tf_listener_.waitForTransform(param_base_frame_, param_laser_frame, ros::Time(0), ros::Duration(1.0));
        tf_base2laser_ = tf_listener_.lookupTransform(param_base_frame, param_laser_frame_, ros::Time(0));
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("Error waiting for transform in gen_costmap: %s", ex.what());
    }

    ros::spin();
    return 0;
}