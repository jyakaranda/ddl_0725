#include <ros/ros.h>
#include <ros/duration.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv-3.3.1-dev/opencv2/imgproc/imgproc.hpp>
#include <opencv-3.3.1-dev/opencv2/highgui/highgui.hpp>

#define LEFT_WINDOW "viz_left"
#define RIGHT_WINDOW "viz_right"

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> timeSynchronier;

bool param_viz;

bool msg2cv(const sensor_msgs::ImageConstPtr &msg, cv::Mat &img)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("cv_bridge exception in msg2cv: %s", e.what());
    return false;
  }
  img = cv_ptr->image;
}

bool cv2msg(sensor_msgs::Image &msg, const cv::Mat &img, const std::string &encoding, const std_msgs::Header header)
{
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr->image = img;
  cv_ptr->encoding = encoding;
  cv_ptr->header = header;
  try
  {
    cv_ptr->toImageMsg(msg);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("cv_bridge exception in cv2msg: %s", e.what());
    return false;
  }

  return true;
}

// void leftImgCB(const sensor_msgs::ImageConstPtr &msg)
// {
// }
// void rightImgCB(const sensor_msgs::ImageConstPtr &msg)
// {
// }

void callback(const sensor_msgs::ImageConstPtr &msg_left, const sensor_msgs::ImageConstPtr &msg_right)
{
  cv::Mat mat_left, mat_right;
  if (!msg2cv(msg_left, mat_left) || !msg2cv(msg_right, mat_right))
  {
    // error in translate
    return;
  }

  // TODO detect with mat_left and mag_right
  if (param_viz)
  {
    cv::imshow(LEFT_WINDOW, mat_left);
    cv::imshow(RIGHT_WINDOW, mat_right);
    cv::waitKey(3);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cal_ob_dist");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  // image_transport::ImageTransport it = image_transport::ImageTransport(nh);
  pnh.param<bool>("viz", param_viz, false);

  cv::namedWindow(LEFT_WINDOW);
  cv::namedWindow(RIGHT_WINDOW);
  // image_transport::Subscriber sub_left_img = it.subscribe("/zed/left/image", 10, leftImgCB);
  // image_transport::Subscriber sub_right_img = it.subscribe("/zed/right/image", 10, rightImgCB);
  message_filters::Subscriber<sensor_msgs::Image> sub_left_img(nh, "/zed/left/image", 5);
  message_filters::Subscriber<sensor_msgs::Image> sub_right_img(nh, "/zed/right/image", 5);
  message_filters::Synchronizer<timeSynchronier> sync(timeSynchronier(10), sub_left_img, sub_right_img);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();
  return 0;
}