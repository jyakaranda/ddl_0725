/**
 * @brief 
 * 
 * @file ndt_localization.cpp
 * @author jyakaranda
 * @date 2018-09-20
 */

#include "ndt_localization/ndt_localization.h"

NDTLocalization::~NDTLocalization(){
  
}

bool NDTLocalization::init()
{
  ROS_INFO("Start init NDTLocalization");

  pose_init_ = false;
  odom_init_ = false;

  pnh_.param<std::string>("map_frame", param_map_frame_, std::string("/map"));
  pnh_.param<std::string>("odom_frame", param_odom_frame_, std::string("/odom"));
  pnh_.param<std::string>("base_frame", param_base_frame_, std::string("/base_link"));
  pnh_.param<std::string>("laser_frame", param_laser_frame_, std::string("/laser"));
  pnh_.param<double>("tf_timeout", param_tf_timeout_, 0.05);
  pnh_.param<double>("odom_timeout", param_odom_timeout_, 1);
  pnh_.param<bool>("use_odom", param_use_odom_, true);
  pnh_.param<double>("predict_error_thresh", param_predict_error_thresh_, 0.5);
  pnh_.param<double>("ndt_resolution", param_ndt_resolution_, 1.0);
  pnh_.param<int>("ndt_max_iterations", param_ndt_max_iterations_, 25);
  pnh_.param<double>("ndt_step_size", param_ndt_step_size_, 0.1);
  pnh_.param<double>("ndt_epsilon", param_ndt_epsilon_, 0.01);

  sub_initial_pose_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1, boost::bind(&NDTLocalization::initialPoseCB, this, _1));
  sub_map_ = nh_.subscribe<sensor_msgs::PointCloud2>("/map/point_cloud", 1, boost::bind(&NDTLocalization::mapCB, this, _1));
  sub_odom_ = nh_.subscribe<nav_msgs::Odometry>("/odom/imu", 500, boost::bind(&NDTLocalization::odomCB, this, _1));

  tf::StampedTransform transform;
  try
  {
    tf_listener_.waitForTransform(param_base_frame_, param_laser_frame_, ros::Time(0), ros::Duration(param_tf_timeout_), ros::Duration(param_tf_timeout_ / 3));
    tf_listener_.lookupTransform(param_base_frame_, param_laser_frame_, ros::Time(0), transform);
  }
  catch (const tf::TransformException &ex)
  {
    ROS_ERROR("Error waiting for tf in init: %s", ex.what());
    return false;
  }
  Eigen::Translation3f tl_btol(transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ());
  double roll, pitch, yaw;
  tf::Matrix3x3(transform.getRotation()).getEulerYPR(yaw, pitch, roll);
  Eigen::AngleAxisf rot_x_btol(roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf rot_y_btol(pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rot_z_btol(yaw, Eigen::Vector3f::UnitZ());
  tf_btol_ = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();

  ROS_INFO("End init NDTLocalization");
  return true;
}

void NDTLocalization::initialPoseCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
  if (msg->header.frame_id != param_map_frame_)
  {
    ROS_WARN("Please initialize pose under %s frame.", param_map_frame_);
    pose_init_ = false;
    return;
  }
  geometryPose2Pose(msg->pose.pose, initial_pose_);

  pre_pose_ = pre_pose_odom_ = current_pose_odom_ = current_pose_ = initial_pose_;
  pose_init_ = true;

  offset_odom_.reset();
  offset_imu_.reset();

  ROS_INFO("Current pose initialized.");
}

/**
 * @brief 1. caculate pdf(mean, covariance) for each voxel grid in model
 * 
 * @param msg better to be filtered map data.
 */
void NDTLocalization::mapCB(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  if (model_pc_num_ == msg->width)
  {
    // suppose it is same map.
    return;
  }

  model_pc_num_ = msg->width;
  pcl::fromROSMsg(*msg, model_pc_);
  PointCloudT::Ptr map_ptr(new PointCloudT(model_pc_));

  pthread_mutex_lock(&mutex);
  ndt_.setResolution(param_ndt_resolution_);
  ndt_.setInputTarget(map_ptr);
  ndt_.setMaximumIterations(param_ndt_max_iterations_);
  ndt_.setStepSize(param_ndt_step_size_);
  ndt_.setTransformationEpsilon(param_ndt_epsilon_);
  pthread_mutex_unlock(&mutex);
  ROS_INFO("Update model pc with %d points.", model_pc_num_);
}

void NDTLocalization::odomCB(const nav_msgs::Odometry::ConstPtr &msg)
{
  if (!odom_init_)
  {
    odom_init_ = true;
    pre_odom_time_ = msg->header.stamp;
    ROS_INFO("Init odom.");
    return;
  }
  double diff_time = (msg->header.stamp - pre_odom_time_).toSec();
  if (diff_time > param_odom_timeout_)
  {
    ROS_WARN("Long time waiting for odom msg, ignore this msg.");
    pre_odom_time_ = msg->header.stamp;
    return;
  }

  msg_odom_ = msg;
  offset_odom_.roll += msg->twist.twist.angular.x * diff_time;
  offset_odom_.pitch += msg->twist.twist.angular.y * diff_time;
  offset_odom_.yaw += msg->twist.twist.angular.z * diff_time;
  double diff_x = msg->twist.twist.linear.x * diff_time;
  offset_odom_.x += std::cos(-current_pose_.pitch) * std::cos(current_pose_.yaw) * diff_x;
  offset_odom_.y += std::cos(-current_pose_.pitch) * std::sin(current_pose_.yaw) * diff_x;
  offset_odom_.z += std::sin(-current_pose_.pitch) * diff_x;
  current_pose_odom_ = pre_pose_odom_ + offset_odom_;
  predict_pose_odom_ = pre_pose_ + offset_odom_;
  pre_pose_odom_ = current_pose_odom_;
}

/** 
 * 1. get data points
 * 2. match data points to model points(map)
 * 2.1 caculate score function: put the point to corresponding pdf, and sum it up
 * 2.2 optimize transformation matrix(position) using Newton method until score function is converged
 */
void NDTLocalization::pointCloudCB(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  // TODO main function

  if (!map_init_ || !pose_init_)
  {
    ROS_WARN("Cannot localize without given map and initial pose.");
    return;
  }

  PointCloudT scan;
  pcl::fromROSMsg(*msg, scan);
  PointCloudT::Ptr scan_ptr(new PointCloudT(scan));
  PointCloudT::Ptr output_cloud(new PointCloudT());
  Eigen::Matrix4f init_guess;
  Eigen::Matrix4f final_tf;
  Eigen::Matrix4f base_tf;
  pose predict_ndt_pose;
  pose ndt_pose;

  // TODO predict_ndt_pose
  if (param_use_odom_)
  {
    predict_ndt_pose = pre_pose_odom_;
  }

  Eigen::Translation3f init_translation(predict_ndt_pose.x, predict_ndt_pose.y, predict_ndt_pose.z);
  Eigen::AngleAxisf init_rotation_x(predict_ndt_pose.roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf init_rotation_y(predict_ndt_pose.pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf init_rotation_z(predict_ndt_pose.yaw, Eigen::Vector3f::UnitZ());
  init_guess = (init_translation * init_rotation_z * init_rotation_y * init_rotation_x) * tf_btol_;

  std::chrono::time_point<std::chrono::system_clock> align_start, align_end, getFitnessScore_start, getFitnessScore_end;

  pthread_mutex_lock(&mutex);
  ndt_.setInputSource(scan_ptr);

  align_start = std::chrono::system_clock::now();
  ndt_.align(*output_cloud, init_guess);
  align_end = std::chrono::system_clock::now();

  final_tf = ndt_.getFinalTransformation();
  has_converged_ = ndt_.hasConverged();
  iteration_ = ndt_.getFinalNumIteration();
  trans_probability_ = ndt_.getTransformationProbability();

  getFitnessScore_start = std::chrono::system_clock::now();
  fitness_score_ = ndt_.getFitnessScore();
  getFitnessScore_end = std::chrono::system_clock::now();

  ROS_INFO("NDT has converged: %d, iterations: %d, fitness_score: %f, trans_probability: %f", has_converged_, iteration_, fitness_score_, trans_probability_);

  pthread_mutex_unlock(&mutex);

  base_tf = final_tf * tf_btol_.inverse();
  tf::Matrix3x3 mat_b;
  mat_b.setValue(static_cast<double>(base_tf(0, 0)), static_cast<double>(base_tf(0, 1)), static_cast<double>(base_tf(0, 2)),
                 static_cast<double>(base_tf(1, 0)), static_cast<double>(base_tf(1, 1)), static_cast<double>(base_tf(1, 2)),
                 static_cast<double>(base_tf(2, 0)), static_cast<double>(base_tf(2, 1)), static_cast<double>(base_tf(2, 2)));

  ndt_pose.x = base_tf(0, 3);
  ndt_pose.y = base_tf(1, 3);
  ndt_pose.z = base_tf(2, 3);
  mat_b.getEulerYPR(ndt_pose.yaw, ndt_pose.pitch, ndt_pose.roll);

  predict_pose_error_ = std::sqrt((ndt_pose.x - predict_ndt_pose.x) * (ndt_pose.x - predict_ndt_pose.x) +
                                  (ndt_pose.y - predict_ndt_pose.y) * (ndt_pose.y - predict_ndt_pose.y) +
                                  (ndt_pose.z - predict_ndt_pose.z) * (ndt_pose.z - predict_ndt_pose.z));
  bool use_predict_pose;
  if (predict_pose_error_ <= param_predict_error_thresh_)
  {
    use_predict_pose = false;
  }
  else
  {
    use_predict_pose = true;
  }

  // use_predict_pose = false;

  if (use_predict_pose)
  {
    current_pose_ = ndt_pose;
  }
  else
  {
    current_pose_ = predict_ndt_pose;
  }

  pose2GeometryPose(msg_current_pose_.pose, current_pose_);
  msg_current_pose_.header.stamp = msg->header.stamp;
  msg_current_pose_.header.frame_id = param_map_frame_;
  pub_current_pose_.publish(msg_current_pose_);

  // // publish map->odom using map->laser and odom->laser
  // tf::StampedTransform transform1;
  // try
  // {
  //   tf_listener_.waitForTransform(param_odom_frame_, param_laser_frame_, ros::Time(0), ros::Duration(param_tf_timeout_), ros::Duration(param_tf_timeout_ / 3));
  //   tf_listener_.lookupTransform(param_odom_frame_, param_laser_frame_, ros::Time(0), transform1);
  // }
  // catch (const tf::TransformException &ex)
  // {
  //   ROS_ERROR("Error waiting for tf in pointCloudCB: %s", ex.what());
  //   // TODO do some stuff
  //   return;
  // }
  // tf::Transform transform2(tf::Quaternion(1, 2, 3, 4), tf::Vector3(1, 2, 3));
  // tf_broadcaster_.sendTransform(tf::StampedTransform(transform2 * transform1.inverse(), msg->header.stamp, param_map_frame_, param_odom_frame_));
}
