#include <ros/ros.h>
#include <ros/duration.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

double param_filter_x, param_filter_y, param_filter_z;
std::string param_frame_id;
ros::Publisher pub_filtered_pc;
sensor_msgs::PointCloud2 msg_filtered_pc;
ros::Subscriber sub_pc;

void pcCB(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  pcl::PointCloud<pcl::PointXYZ> pcl_raw;
  pcl::PointCloud<pcl::PointXYZ> pcl_filter;
  pcl::fromROSMsg(*msg, pcl_raw);

  for (int i = 0; i < pcl_raw.points.size(); i++)
  {
    if (std::fabs(pcl_raw.points[i].x) <= param_filter_x && std::fabs(pcl_raw.points[i].y) <= param_filter_y && std::fabs(pcl_raw.points[i].z) <= param_filter_z)
    {
      pcl_filter.points.push_back(pcl_raw.points[i]);
    }
  }

  pcl::toROSMsg(pcl_filter, msg_filtered_pc);
  msg_filtered_pc.header.stamp = msg->header.stamp;
  msg_filtered_pc.header.frame_id = param_frame_id;
  pub_filtered_pc.publish(msg_filtered_pc);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rt_filter_node");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  pnh.param<double>("filter_x", param_filter_x, 10);
  pnh.param<double>("filter_y", param_filter_y, 10);
  pnh.param<double>("filter_z", param_filter_z, 10);
  pnh.param<std::string>("frame_id", param_frame_id, "/laser");

  sub_pc = nh.subscribe<sensor_msgs::PointCloud2>("/raw_pc", 10, pcCB);
  pub_filtered_pc = nh.advertise<sensor_msgs::PointCloud2>("/filtered_pc", 10);
  msg_filtered_pc.header.frame_id = param_frame_id;

  ros::spin();

  return 0;
}