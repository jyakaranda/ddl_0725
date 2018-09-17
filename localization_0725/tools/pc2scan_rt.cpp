#include <sstream>
#include <boost/filesystem.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include "pcl_ros/transforms.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

typedef sensor_msgs::PointCloud2 PointCloud;
typedef PointCloud::Ptr PointCloudPtr;
typedef PointCloud::ConstPtr PointCloudConstPtr;
typedef sensor_msgs::LaserScan LaserScan;
typedef LaserScan::Ptr LaserScanPtr;
typedef LaserScan::ConstPtr LaserScanConstPtr;

ros::Subscriber sub_pc;
ros::Publisher pub_scan;

int point_num = 2000;
double angle_base = 2 * M_PI / point_num;
int count = 0;

void pcCB(const PointCloudConstPtr &msg)
{
    int cloud_size = 0;
    float angle = 0.;
    int scan_id = 0;
    int point_idx = 0;
    double ori = 0.;
    bool halfPassed = false;
    float distance = 0.;

    pcl::PointCloud<pcl::PointXYZI> pc_pcl;
    pcl::fromROSMsg(*msg, pc_pcl);
    LaserScan scan;
    cloud_size = pc_pcl.size();
    if (cloud_size < 100)
    {
        return;
    }

    scan.header.frame_id = std::string("/laser");
    scan.header.stamp = msg->header.stamp;
    scan.header.seq = msg->header.seq;
    scan.angle_min = 0;
    scan.angle_max = 2 * M_PI;
    scan.angle_increment = (scan.angle_max - scan.angle_min) / point_num;
    scan.range_min = 0.05;
    scan.range_max = 10.;
    scan.ranges.reserve(point_num);
    scan.ranges.assign(point_num, std::numeric_limits<float>::infinity());
    scan.intensities.reserve(point_num);
    scan.intensities.assign(point_num, std::numeric_limits<float>::infinity());

    for (int i = 0; i < cloud_size; i++)
    {
        if (!std::isfinite(pc_pcl.points[i].x) ||
            !std::isfinite(pc_pcl.points[i].y) ||
            !std::isfinite(pc_pcl.points[i].z))
        {
            continue;
        }

        distance = std::sqrt(pc_pcl.points[i].x * pc_pcl.points[i].x + pc_pcl.points[i].y * pc_pcl.points[i].y + pc_pcl.points[i].z * pc_pcl.points[i].z);
        if (distance < scan.range_min || distance > scan.range_max)
        {
            continue;
        }

        angle = -atan(pc_pcl.points[i].z / (std::sqrt(pc_pcl.points[i].x * pc_pcl.points[i].x + pc_pcl.points[i].y * pc_pcl.points[i].y)));
        scan_id = (int)((angle * 180 / M_PI + 15.) * (16 - 1) / 30. + 0.5);
        if (scan_id != 6)
        {
            continue;
        }

        ori = atan2(pc_pcl.points[i].y, pc_pcl.points[i].x);
        if (ori < 0)
        {
            ori += 2 * M_PI;
        }
        if (ori < 0 || ori >= 2 * M_PI)
        {
            ROS_INFO("Outrage ori: %.4f.", ori);
            continue;
        }

        point_idx = ori / angle_base;

        scan.ranges[point_idx] = distance;
        scan.intensities[point_idx] = pc_pcl.points[i].intensity;
    }

    pub_scan.publish(scan);

}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "pc2scan_rt");
    ROS_INFO("Starting pc2scan_rt...");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    sub_pc = nh.subscribe<PointCloud>("/lslidar_point_cloud", 5, pcCB);
    pub_scan = nh.advertise<LaserScan>("/scan", 5);

    ROS_INFO("Successfully end.");
    ros::spin();

    return 0;
}