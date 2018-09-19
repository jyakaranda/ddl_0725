#ifndef __PF_LOCALIZATION__
#define __PF_LOCALIZATION__

#include <cstdio>
#include <ctime>
#include <random>
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PointStamped.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/GetMap.h>
#include <ros/service.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include "base_localization.h"
#include "pf_localization/pf/pf.h"
#include "pf_localization/pf/pf_kdtree.h"
#include "RangeLib.h"


namespace localization
{
#define PI 3.1415926
double ang2deg(const double ang)
{
    return ang * PI / 180.;
}
double deg2ang(const double deg)
{
    return deg / PI * 180.;
}
class pf_localization : public LocalizationBase
{
  public:
    pf_localization(ros::NodeHandle nh, ros::NodeHandle pnh) : nh_(nh), pnh_(pnh)
    {
    }
    ~pf_localization();
    bool init();
    bool loadParams();
    bool run(){}
    bool stop(){}
    void publishPoseBroadcastTF(const geometry_msgs::PoseWithCovarianceStampedConstPtr localization){}

  private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf2_broadcaster_;
    tf::TransformListener tf_listener_;
    tf::TransformBroadcaster tf_broadcaster_;
    
    tf::StampedTransform transform_;
    ros::Subscriber sub_initpose_;
    ros::Subscriber sub_clicked_;
    ros::Subscriber sub_scan_;
    ros::Subscriber sub_odom_;
    ros::Publisher pub_odom_;
    ros::Publisher pub_particle_;
    ros::ServiceClient getmap_call_;
    nav_msgs::GetMap srv_getmap_;
    nav_msgs::OccupancyGrid static_map_;
    sensor_msgs::LaserScan scan_msg_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr pointclouds_;

    // 初始位置估计
    void initalposeCB(const geometry_msgs::PoseWithCovarianceStampedConstPtr &initpose);
    void clickedPointCB(const geometry_msgs::PointStampedConstPtr &point);
    void odomCB(const nav_msgs::OdometryConstPtr &msg);
    // update
    void laserCB(const sensor_msgs::LaserScanConstPtr &msg);
    // void pointcloudCB(const sensor_msgs::PointCloud2ConstPtr &pc);
    void getMap();
    void precomputeSensorModel();
    void initializeGlobal();
    // 1. 重要性采样
    void resampleParticles();
    // 2. 预测 motion model
    void motionModel(const pf_vector_t &action);
    // 3. 更新 sensor model
    void sensorModel();
    // 4. 归一化
    void normalize();
    void publishTF(const pf_vector_t &mean);
    void visualize();
    void update();

    pf_vector_t *getFromGuassian(const pf_vector_t mean, const pf_matrix_t cov);
    double randomUniform(double min, double max);

    pf_t* pf_;
    pf_vector_t pose_mean_;
    pf_matrix_t pose_cov_;
    // pf 是否初始化
    bool pf_initialized_;
    int resample_count;

    // 粒子数目
    int param_max_particles_;
    int param_min_particles_;
    // 运动更新阈值
    double param_update_dist_;
    double param_update_angle_;
    // 各个 coords frame_id，用于之后的 tf 转换
    std::string param_laser_frame_id_;
    std::string param_odom_frame_id_;
    std::string param_base_link_frame_id_;
    std::string param_map_frame_id_;
    bool param_publish_odom_;
    int param_max_viz_particles_;
    int param_resample_interval_;

    double param_max_range_;
    std::string param_range_method_;
    int param_rangelib_var_;
    int param_td_;    // theta_discret
    double param_z_short_;
    double param_z_max_;
    double param_z_rand_;
    double param_sigma_hit_;
    double param_z_hit_;
    double param_c_r_;
    int param_downsample_size_;
    double param_motion_dev_x_;
    double param_motion_dev_y_;
    double param_motion_dev_theta_;
    double param_squash_fac_;

    ranges::OMap* omap_;
    ranges::RangeMethod* rm_;
    int max_range_px_;
    int table_width_;
    uint8_t* permissible_region_;
    double* sensor_model_table_;
    float* queries_;
    float* ranges_;
    float* tiled_angles_;
    double* weights_;
    int num_rays_;
    bool map_initialized_;
    bool first_sensor_update_;

    bool lidar_initialized_;
    float* downsample_ranges_;
    float* downsample_angles_;

    bool odom_initialized_;
    pf_vector_t odom_data_;         // 控制信息，即与上一时刻的相对移动距离和角度
    pf_vector_t last_pose_;
    ros::Time last_stamp_;
    bool first_odom_;

    // 正态分布生成器
    std::default_random_engine generator_[3];
    std::normal_distribution<double> guassian_[3];

    // 连续均匀分布生成器
    std::random_device rd_;
    std::mt19937 gen_;
    std::uniform_real_distribution<double> uniform_;

    boost::recursive_mutex config_mutex_;

    int count_;
};
} // namespace localization

#endif
