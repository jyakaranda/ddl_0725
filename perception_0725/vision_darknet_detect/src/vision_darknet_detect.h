
#ifndef DARKNET_YOLO3_H
#define DARKNET_YOLO3_H

#define __APP_NAME__ "vision_darknet_detect"

#include <cstdint>
#include <cstdlib>
#include <string>
#include <vector>

#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>

#include <autoware_msgs/ConfigSsd.h>


#include <DetectedObject.h>
#include <DetectedObjectArray.h>

#include <rect_class_score.h>
#include <opencv2/opencv.hpp>
#include <sl_zed/Camera.hpp>

extern "C"
{
#undef __cplusplus
#include "box.h"
#include "image.h"
#include "network.h"
#include "detection_layer.h"
#include "parser.h"
#include "region_layer.h"
#include "utils.h"
#include "image.h"
#define __cplusplus
}

#define LEFT_WINDOW "viz_left"
#define RIGHT_WINDOW "viz_right"

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> timeSynchronier;


namespace darknet
{
class Yolo3Detector
{
  private:
    double min_confidence_, nms_threshold_;
    network *darknet_network_;
    std::vector<box> darknet_boxes_;
    std::vector<RectClassScore<float>> forward(image &in_darknet_image);

  public:
    Yolo3Detector() {}

    void load(std::string &in_model_file, std::string &in_trained_file, double in_min_confidence,
              double in_nms_threshold);

    ~Yolo3Detector();

    image convert_image(const sensor_msgs::ImageConstPtr &in_image_msg);

    std::vector<RectClassScore<float>> detect(image &in_darknet_image);

    uint32_t get_network_width();

    uint32_t get_network_height();
};
} // namespace darknet

class Yolo3DetectorNode
{
    ros::Subscriber subscriber_image_raw_;
    ros::Subscriber subscriber_yolo_config_;
    ros::Publisher publisher_objects_;
    ros::NodeHandle node_handle_;

    message_filters::Subscriber<sensor_msgs::Image>* sub_left_img;
    message_filters::Subscriber<sensor_msgs::Image>* sub_right_img;
    message_filters::Synchronizer<timeSynchronier>* sync;

    darknet::Yolo3Detector yolo_detector_;

    image darknet_image_ = {};

    float score_threshold_;
    float nms_threshold_;
    double image_ratio_;               //resdize ratio used to fit input image to network input size
    uint32_t image_top_bottom_border_; //black strips added to the input image to maintain aspect ratio while resizing it to fit the network input size
    uint32_t image_left_right_border_;
    std::vector<cv::Scalar> colors_;
    bool param_viz;

    void convert_rect_to_image_obj(std::vector<RectClassScore<float>> &in_objects, autoware_msgs::DetectedObjectArray &out_message);
    void rgbgr_image(image &im);
    //image convert_ipl_to_image(const sensor_msgs::ImageConstPtr& msg);
    //image convert_ipl_to_image(const sensor_msgs::ImageConstPtr &msg);
    //void image_callback(const sensor_msgs::ImageConstPtr& in_image_message);
    //void image_callback(const sensor_msgs::ImageConstPtr &msg_left);
    void config_cb(const autoware_msgs::ConfigSsd::ConstPtr &param);
    //void callback(const sensor_msgs::ImageConstPtr &msg_left, const sensor_msgs::ImageConstPtr &msg_right);

    image convert_ipl_to_image(cv::Mat &mat_image);
    void callback(cv::Mat &mat_left, cv::Mat &mat_right,cv::Mat &Q,cv::Mat &filtered_disp_vis);

  public:
    void Run();
};

#endif // DARKNET_YOLO3_H
