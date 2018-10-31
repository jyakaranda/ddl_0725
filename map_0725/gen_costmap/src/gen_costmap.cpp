#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <ros/duration.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <dynamic_reconfigure/server.h>
#include <gen_costmap/GenCostmapConfig.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <Eigen/Geometry>

#include <vector>

ros::Publisher pub_costmap_;
nav_msgs::OccupancyGrid msg_costmap_;
// tf::TransformListener tf_listener_;      // 这玩意居然是个 nodehandle ，如果在全局定义的话会报错：You must call ros::init() before creating the first NodeHandle
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
int param_cost_base_; // 每个投影到栅格地图中的点，都将其 cost 加上这个值
double param_delta_;
bool param_filter_;
std::string param_frame_map_;
std::string param_frame_base_;
std::string param_frame_laser_;

int g_width_;
int g_height_;
std::vector<int> g_costmap_;
int g_count_;
Eigen::Matrix4f g_tf_b2l_;

bool setOccupancyGrid(nav_msgs::OccupancyGrid *og)
{
    // tf::TransformListener tf_listener_;
    // try
    // {
    //     tf_listener_.waitForTransform(param_frame_map_, param_frame_base_, ros::Time(0), ros::Duration(0.1));
    //     tf_listener_.lookupTransform(param_frame_map_, param_frame_base_, ros::Time(0), tf_map2base_);
    // }
    // catch (tf::TransformException &ex)
    // {
    //     ROS_ERROR("Error waiting for transform: %s", ex.what());
    //     return false;
    // }

    int width = std::ceil((param_xh_ - param_xl_) / param_resolution_);
    int height = std::ceil((param_yh_ - param_yl_) / param_resolution_);
    if (width == g_width_ && height == g_height_)
    {
        g_costmap_.assign(g_costmap_.size(), 0);
    }
    else
    {
        g_width_ = width;
        g_height_ = height;
        g_costmap_.resize(g_width_ * g_height_);
    }
    // TODO: bug
    og->header.frame_id = param_frame_base_; // frame 为 base_link
    og->info.resolution = param_resolution_;
    og->info.width = g_width_;
    og->info.height = g_height_;
    // og->info.origin.position.x = tf_map2base_.getOrigin().getX() - param_xl_;
    // og->info.origin.position.y = tf_map2base_.getOrigin().getY() - param_yl_;
    // og->info.origin.position.z = 0;
    // tf::Quaternion tmp_q;
    // double roll, pitch, yaw;
    // tf::Matrix3x3(tf_map2base_.getRotation()).getRPY(roll, pitch, yaw);
    // tmp_q.setRPY(0., 0., yaw);
    // tf::quaternionTFToMsg(tmp_q, og->info.origin.orientation);
    og->info.origin.position.x = param_xl_;
    og->info.origin.position.y = param_yl_;
    og->info.origin.orientation.w = 1.0;
    return true;
}

std::vector<int> filterCostMap(std::vector<int> &cost_map)
{
    std::vector<int> filtered_cost_map(cost_map.size(), 0);

    // cells around reference (x, y)
    std::vector<std::pair<int, int>> neighborhood{
        std::make_pair(-1, -1),
        std::make_pair(0, -1),
        std::make_pair(1, -1),
        std::make_pair(-1, 0),
        std::make_pair(1, 0),
        std::make_pair(-1, 1),
        std::make_pair(0, 1),
        std::make_pair(1, 1),
    };

    for (size_t size = cost_map.size(), i = 0; i < size; i++)
    {
        int ref_cost = cost_map[i];

        int ref_x = i % g_width_;
        int ref_y = (i - ref_x) / g_height_;

        // we don't have to filter if the cost is 0
        if (ref_cost <= 0)
        {
            filtered_cost_map[i] = ref_cost;
            continue;
        }

        filtered_cost_map[i] += ref_cost;

        // increase the cost for each neighborhood cell
        for (const auto &n : neighborhood)
        {
            int neighbor_x = ref_x + n.first;
            int neighbor_y = ref_y + n.second;

            if (neighbor_x < 0 || neighbor_x >= g_width_ || neighbor_y < 0 || neighbor_y >= g_height_)
                continue;

            int neighbor_index = neighbor_x + neighbor_y * g_width_;
            filtered_cost_map[neighbor_index] += ref_cost;
        }
    }

    // handle the cost over 100
    for (auto &cost : filtered_cost_map)
    {
        if (cost > 100)
            cost = 100;
    }

    return filtered_cost_map;
}

void pointsCB(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    if (g_count_ % param_frame_num_ == 0)
    {
        // ROS_INFO("gen new costmap.");
        if (!setOccupancyGrid(&msg_costmap_))
        {
            ROS_WARN("cannot gen costmap");
            return;
        }
    }
    g_count_++;

    pcl::PointCloud<pcl::PointXYZ> pc_laser, pc_base;
    pcl::fromROSMsg(*msg, pc_laser);
    // TODO: 应转换到 map
    pcl::transformPointCloud(pc_laser, pc_base, g_tf_b2l_); // 将 laser 坐标的点云转换到 base_link 坐标下，因为局部 costmap 是以 base_link 为基准的

    for (const auto &p : pc_base.points)
    {
        int i_x = std::floor((p.x - msg_costmap_.info.origin.position.x) / msg_costmap_.info.resolution);
        int i_y = std::floor((p.y - msg_costmap_.info.origin.position.y) / msg_costmap_.info.resolution);
        double z = p.z;
        // TODO: 判断 z 不对
        if ((i_x < 0 || i_x >= g_width_) || (i_y < 0 || i_y >= g_height_) || (z < param_zl_ || z > param_zh_) || z <= (0.425 - msg_costmap_.info.origin.position.z))
        {
            continue;
        }
        g_costmap_[i_y * g_width_ + i_x] += 1;
    }

    if (g_count_ % param_frame_num_ == 0)
    {
        // got param_frame_num_ frames of pointcloud, publish
        g_count_ = 0;
        for (auto &i : g_costmap_)
        {
            if (i > 0)
            {
                if (i * 1.0 / param_frame_num_ >= param_confidence_)
                {
                    i = 100;
                }
                else
                {
                    i = 20;
                }
            }
        }
        if (param_filter_)
        {
            g_costmap_ = filterCostMap(g_costmap_);
        }
        // ROS_INFO("publish new costmap.");
        msg_costmap_.data.insert(msg_costmap_.data.end(), g_costmap_.begin(), g_costmap_.end());
        msg_costmap_.header.stamp = msg->header.stamp;
        pub_costmap_.publish(msg_costmap_);
        msg_costmap_.data.clear();
    }
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
    param_cost_base_ = config.cost_base;
    param_delta_ = config.delta;
    param_filter_ = config.use_filter;
    ROS_INFO("new GenCostmapConfig set.");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "local_costmap");
    ROS_INFO("Start gen_costmap.");
    std::cout << "what" << std::endl;
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    tf::TransformListener tf_listener_;
    try
    {
        tf_listener_.waitForTransform(param_frame_base_, param_frame_laser_, ros::Time(0), ros::Duration(1.0));
        tf_listener_.lookupTransform(param_frame_base_, param_frame_laser_, ros::Time(0), tf_base2laser_);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("Error waiting for transform in gen_costmap: %s", ex.what());
        return -1;
    }
    double roll, pitch, yaw;
    tf::Matrix3x3(tf_base2laser_.getRotation()).getRPY(roll, pitch, yaw);
    Eigen::Translation3f tl_b2l(tf_base2laser_.getOrigin().getX(), tf_base2laser_.getOrigin().getY(), tf_base2laser_.getOrigin().getZ()); // tl: translation
    Eigen::AngleAxisf rot_x_b2l(roll, Eigen::Vector3f::UnitX());                                                                          // rot: rotation
    Eigen::AngleAxisf rot_y_b2l(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rot_z_b2l(yaw, Eigen::Vector3f::UnitZ());
    g_tf_b2l_ = (tl_b2l * rot_z_b2l * rot_y_b2l * rot_x_b2l).matrix();

    pnh.param<int>("frame_num", param_frame_num_, 1);
    pnh.param<double>("confidence", param_confidence_, 0.5);
    pnh.param<double>("resolution", param_resolution_, 0.1);
    pnh.param<double>("xh", param_xh_, 50.);
    pnh.param<double>("yh", param_yh_, 10.);
    pnh.param<double>("zh", param_zh_, 2.);
    pnh.param<double>("xl", param_xl_, -50.);
    pnh.param<double>("yl", param_yl_, -10.);
    pnh.param<double>("zl", param_zl_, -2.);
    pnh.param<int>("cost_base", param_cost_base_, 20);
    pnh.param<double>("delta", param_delta_, 0.05);
    pnh.param<bool>("use_filter", param_filter_, true);
    pnh.param<std::string>("map_frame", param_frame_map_, std::string("map"));
    pnh.param<std::string>("base_frame", param_frame_base_, std::string("/base_link"));
    pnh.param<std::string>("laser_frame", param_frame_laser_, std::string("/laser"));

    ros::Subscriber sub_points = nh.subscribe("/lslidar_point_cloud", 10, pointsCB);
    // ros::Subscriber sub_map = nh.subscribe("/pc_map", 1, mapCB);
    pub_costmap_ = nh.advertise<nav_msgs::OccupancyGrid>("/costmap", 1);
    dynamic_reconfigure::Server<gen_costmap::GenCostmapConfig> cfg_server;
    dynamic_reconfigure::Server<gen_costmap::GenCostmapConfig>::CallbackType cfg_callback = boost::bind(&cfgCB, _1, _2);
    cfg_server.setCallback(cfg_callback);

    ros::spin();
    return 0;
}