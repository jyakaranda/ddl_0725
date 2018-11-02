#ifndef __DDL_OBSTACLE_FILTER__
#define __DDL_OBSTACLE_FILTER__

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>

class ObstacleFilter
{
  public:
    ObstacleFilter();
    ~ObstacleFilter();
    void run();

  private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    tf::TransformListener tf_listener_;
    ros::Subscriber sub_raw_pc_;
    ros::Publisher pub_obstacle_;
    sensor_msgs::PointCloud2 msg_obstacle_;

    tf::StampedTransform tf_map2laser_;

    bool init();
    void rawPointCloudCB(const sensor_msgs::PointCloud2::ConstPtr &msg);
};

#endif