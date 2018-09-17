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

int main(int argc, char **argv)
{

    ros::init(argc, argv, "pc2scan");
    ROS_INFO("Starting pc2scan...");

    if (argc < 4)
    {
        std::cerr << "Syntax is: " << argv[0] << " <file_in.bag> <topic> <file_out.bag> [<target_frame>]" << std::endl;
        std::cerr << "Example: " << argv[0] << " data.bag /laser_tilt_cloud output.bag /base_link" << std::endl;
        return (-1);
    }

    rosbag::Bag bag;
    rosbag::Bag output_bag;
    rosbag::View view;
    rosbag::View::iterator view_it;

    try
    {
        bag.open(argv[1], rosbag::bagmode::Read);
        output_bag.open(argv[3], rosbag::bagmode::Write);
    }
    catch (rosbag::BagException e)
    {
        std::cerr << "Error opening file " << argv[1] << ". " << e.what() << std::endl;
        return (-1);
    }

    view.addQuery(bag, rosbag::TopicQuery(argv[2]));
    view_it = view.begin();

    // Add the PointCloud2 handler
    std::cerr << "Changing recorded sensor_msgs::PointCloud2 messages on topic " << argv[2] << " to " << argv[3] << std::endl;

    pcl::PointCloud<pcl::PointXYZI> pc_pcl;

    int point_num = 2000;
    double angle_base = 2 * M_PI / point_num;
    int count = 0;

    while (view_it != view.end())
    {
        int cloud_size = 0;
        float startOri = 0.;
        float endOri = 0.;
        float angle = 0.;
        int scan_id = 0;
        int point_idx = 0;
        double ori = 0.;
        bool halfPassed = false;
        float distance = 0.;

        PointCloudConstPtr pc = view_it->instantiate<PointCloud>();
        if (pc == NULL)
        {
            view_it++;
            continue;
        }
        pcl::fromROSMsg(*pc, pc_pcl);
        LaserScan scan;
        cloud_size = pc_pcl.size();
        if (cloud_size < 100)
        {
            continue;
        }
        startOri = -std::atan2(pc_pcl.points[0].y, pc_pcl.points[0].x);
        endOri = -std::atan2(pc_pcl.points[cloud_size - 1].y, pc_pcl.points[cloud_size - 1].x) + 2 * M_PI;

        scan.header.frame_id = std::string("/laser");
        scan.header.stamp = pc->header.stamp;
        scan.header.seq = pc->header.seq;
        scan.angle_min = 0;
        scan.angle_max = 2 * M_PI;
        scan.angle_increment = (scan.angle_max - scan.angle_min) / point_num;
        scan.range_min = 0.05;
        scan.range_max = 10.;
        scan.ranges.reserve(point_num);
        scan.ranges.assign(point_num, std::numeric_limits<float>::infinity());
        scan.intensities.reserve(point_num);
        scan.intensities.assign(point_num, std::numeric_limits<float>::infinity());

        if (endOri - startOri > 3 * M_PI)
        {
            endOri -= 2 * M_PI;
        }
        else if (endOri - startOri < M_PI)
        {
            endOri += 2 * M_PI;
        }

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

            ori = -atan2(pc_pcl.points[i].y, pc_pcl.points[i].x);
            if (ori < 0)
            {
                ori += 2 * M_PI;
            }
            // if (!halfPassed)
            // {
            //     if (ori < startOri - M_PI / 2)
            //     {
            //         ori += 2 * M_PI;
            //     }
            //     else if (ori > startOri + M_PI * 3 / 2)
            //     {
            //         ori -= 2 * M_PI;
            //     }

            //     if (ori - startOri > M_PI)
            //     {
            //         halfPassed = true;
            //     }
            // }
            // else
            // {
            //     ori += 2 * M_PI;

            //     if (ori < endOri - M_PI * 3 / 2)
            //     {
            //         ori += 2 * M_PI;
            //     }
            //     else if (ori > endOri + M_PI / 2)
            //     {
            //         ori -= 2 * M_PI;
            //     }
            // }

            if (ori < 0 || ori >= 2 * M_PI)
            {
                ROS_INFO("Outrage ori: %.4f.", ori);
                continue;
            }

            point_idx = ori / angle_base;

            scan.ranges[point_idx] = distance;
            scan.intensities[point_idx] = pc_pcl.points[i].intensity;
        }

        output_bag.write(std::string("/scan"), scan.header.stamp, scan);

        std::cerr << "laser scan saved to output.bag. count: " << count++ << ", seq: " << pc->header.seq << ", stamp: " << pc->header.stamp << std::endl;
        view_it++;
    }

    ROS_INFO("Successfully end.");

    return 0;
}