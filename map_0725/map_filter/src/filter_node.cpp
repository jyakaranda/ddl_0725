#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/filters/statistical_outlier_removal.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <dynamic_reconfigure/server.h>
#include <map_filter/FilterConfig.h>

pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
ros::Publisher pub_pc;
std::string param_frame_id;
int param_mean_k;
double param_stddev;

void pointcloudCB(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>()), cloud_out(new pcl::PointCloud<pcl::PointXYZ>());
  sensor_msgs::PointCloud2 msg_pc_filtered;
  pcl::fromROSMsg(*msg, *cloud_in);

  sor.setMeanK(param_mean_k);
  sor.setStddevMulThresh(param_stddev);
  sor.setInputCloud(cloud_in);
  sor.filter(*cloud_out);

  pcl::toROSMsg(*cloud_out, msg_pc_filtered);
  msg_pc_filtered.header = msg->header;
  msg_pc_filtered.header.frame_id = param_frame_id;

  pub_pc.publish(msg_pc_filtered);
  ROS_INFO("pc before filter: %d\npc after filter: %d", cloud_in->size(), cloud_out->size());
}

void cfgCB(const map_filter::FilterConfig &config, uint32_t level)
{
  param_mean_k = config.mean_k;
  param_stddev = config.stddev_mul_thresh;
  ROS_INFO("new StatisticalOutlierRemoval config set: mean_k: %d, stddev: %.2f", param_mean_k, param_stddev);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "filter_node");
  ROS_INFO("Start filter_node");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ros::Subscriber sub_pc_raw = nh.subscribe<sensor_msgs::PointCloud2>("/points_raw", 1, boost::bind(&pointcloudCB, _1));
  pub_pc = nh.advertise<sensor_msgs::PointCloud2>("/points_filtered", 1);
  pnh.param<std::string>("frame_id", param_frame_id, "map");
  pnh.param<int>("mean_k", param_mean_k, 100);
  pnh.param<double>("stddev", param_stddev, 1.0);
  dynamic_reconfigure::Server<map_filter::FilterConfig> cfg_server;
  dynamic_reconfigure::Server<map_filter::FilterConfig>::CallbackType cfg_callback = boost::bind(&cfgCB, _1, _2);
  cfg_server.setCallback(cfg_callback);

  ros::spin();
  return 0;
}