#include <ros/ros.h>
#include <ros/duration.h>
#include <sensor_msgs/PointCloud2.h>
#include <dynamic_reconfigure/server.h>
#include <map_filter/RTFilterConfig.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

double param_filter_xh, param_filter_yh, param_filter_zh;
double param_filter_xl, param_filter_yl, param_filter_zl;
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
    if ((pcl_raw.points[i].x <= param_filter_xh && pcl_raw.points[i].x >= param_filter_xl) && (pcl_raw.points[i].y <= param_filter_yh && pcl_raw.points[i].y >= param_filter_yl) && (pcl_raw.points[i].z <= param_filter_zh && pcl_raw.points[i].z >= param_filter_zl))
    {
      pcl_filter.points.push_back(pcl_raw.points[i]);
    }
  }

  pcl::toROSMsg(pcl_filter, msg_filtered_pc);
  msg_filtered_pc.header.stamp = msg->header.stamp;
  msg_filtered_pc.header.frame_id = param_frame_id;
  pub_filtered_pc.publish(msg_filtered_pc);
}

void cfgCB(const map_filter::RTFilterConfig &config, uint32_t level)
{
  param_filter_xh = config.filter_xh;
  param_filter_yh = config.filter_yh;
  param_filter_zh = config.filter_zh;
  param_filter_xl = config.filter_xl;
  param_filter_yl = config.filter_yl;
  param_filter_zl = config.filter_zl;
  ROS_INFO("Set new config: xh: %.2f, yh: %.2f, zh: %.2f; xl: %.2f, yl: %.2f, zl: %.2f", param_filter_xh, param_filter_yh, param_filter_zh, param_filter_xl, param_filter_yl, param_filter_zl);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "rt_filter_node");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  pnh.param<double>("filter_xh", param_filter_xh, 10);
  pnh.param<double>("filter_yh", param_filter_yh, 10);
  pnh.param<double>("filter_zh", param_filter_zh, 10);
  pnh.param<double>("filter_xl", param_filter_xl, 10);
  pnh.param<double>("filter_yl", param_filter_yl, 10);
  pnh.param<double>("filter_zl", param_filter_zl, 10);
  pnh.param<std::string>("frame_id", param_frame_id, "/laser");

  sub_pc = nh.subscribe<sensor_msgs::PointCloud2>("/raw_pc", 10, pcCB);
  pub_filtered_pc = nh.advertise<sensor_msgs::PointCloud2>("/filtered_pc", 10);
  dynamic_reconfigure::Server<map_filter::RTFilterConfig> cfg_server;
  dynamic_reconfigure::Server<map_filter::RTFilterConfig>::CallbackType cfg_callback = boost::bind(&cfgCB, _1, _2);
  cfg_server.setCallback(cfg_callback);
  msg_filtered_pc.header.frame_id = param_frame_id;

  ros::spin();

  return 0;
}