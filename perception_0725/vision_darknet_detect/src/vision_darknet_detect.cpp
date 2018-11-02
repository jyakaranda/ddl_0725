#include "vision_darknet_detect.h"
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
#include <opencv-3.3.1-dev/opencv2/opencv.hpp>

#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include <opencv-3.3.1-dev/opencv2/ximgproc/disparity_filter.hpp>
#include <iostream>
#include <string>
#include <vector>

#include <opencv-3.3.1-dev/opencv2/calib3d/calib3d.hpp>
#include "opencv2/video/tracking.hpp"

#include <cxmisc.h>
#include <highgui.h>
#include <cvaux.h>
#include <iostream>

#include <ctype.h>
#include <stdlib.h>

#include <vector>
#include <string>
#include <algorithm>
#include <ctype.h>
#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include <mutex>


using namespace cv;
using namespace std;
using namespace cv::ximgproc;



const String keys =
"{help h usage ? |                  | print this message                                                }"
"{@left          |../data/aloeL.jpg | left view of the stereopair                                       }"
"{@right         |../data/aloeR.jpg | right view of the stereopair                                      }"
"{GT             |../data/aloeGT.png| optional ground-truth disparity (MPI-Sintel or Middlebury format) }"
"{dst_path       |None              | optional path to save the resulting filtered disparity map        }"
"{dst_raw_path   |None              | optional path to save raw disparity map before filtering          }"
"{algorithm      |bm                | stereo matching method (bm or sgbm)                               }"
"{filter         |wls_conf          | used post-filtering (wls_conf or wls_no_conf)                     }"
"{no-display     |                  | don't display results                                             }"
"{no-downscale   |                  | force stereo matching on full-sized views to improve quality      }"
"{dst_conf_path  |None              | optional path to save the confidence map used in filtering        }"
"{vis_mult       |1.0               | coefficient used to scale disparity map visualizations            }"
"{max_disparity  |160               | parameter of stereo matching                                      }"
"{window_size    |-1                | parameter of stereo matching                                      }"
"{wls_lambda     |8000.0            | parameter of post-filtering                                       }"
"{wls_sigma      |1.6               | parameter of post-filtering                                       }"
;

//mutex 
std::mutex mtx;

#if (CV_MAJOR_VERSION <= 2)

#else
#include "gencolors.cpp"
#endif

int centerX;
int centerY;
float newX;
float newY;

Mat pointcloud;

typedef struct{
    float all_distance;
    float all_direction;
} Result;

namespace darknet
{

// 高度
uint32_t Yolo3Detector::get_network_height()
{
    return darknet_network_->h;
}
//宽度
uint32_t Yolo3Detector::get_network_width()
{
    return darknet_network_->w;
}
//加载
void Yolo3Detector::load(std::string &in_model_file, std::string &in_trained_file, double in_min_confidence, double in_nms_threshold)
{
    min_confidence_ = in_min_confidence;
    nms_threshold_ = in_nms_threshold;
    //模型文件
    darknet_network_ = parse_network_cfg(&in_model_file[0]);
    //训练参数
    load_weights(darknet_network_, &in_trained_file[0]);
    set_batch_network(darknet_network_, 1);

    layer output_layer = darknet_network_->layers[darknet_network_->n - 1];
    darknet_boxes_.resize(output_layer.w * output_layer.h * output_layer.n);
}

Yolo3Detector::~Yolo3Detector()
{
    free_network(darknet_network_);
}

std::vector<RectClassScore<float>> Yolo3Detector::detect(image &in_darknet_image)
{
    return forward(in_darknet_image);
}

image Yolo3Detector::convert_image(const sensor_msgs::ImageConstPtr &msg)
{
    if (msg->encoding != sensor_msgs::image_encodings::BGR8)
    {
        ROS_ERROR("Unsupported encoding");
        exit(-1);
    }

    auto data = msg->data;
    uint32_t height = msg->height, width = msg->width, offset = msg->step - 3 * width;
    uint32_t i = 0, j = 0;
    image im = make_image(width, height, 3);

    for (uint32_t line = height; line; line--)
    {
        for (uint32_t column = width; column; column--)
        {
            for (uint32_t channel = 0; channel < 3; channel++)
                im.data[i + width * height * channel] = data[j++] / 255.;
            i++;
        }
        j += offset;
    }

    if (darknet_network_->w == (int)width && darknet_network_->h == (int)height)
    {
        return im;
    }
    image resized = resize_image(im, darknet_network_->w, darknet_network_->h);
    free_image(im);
    return resized;
}

std::vector<RectClassScore<float>> Yolo3Detector::forward(image &in_darknet_image)
{
    float *in_data = in_darknet_image.data;
    float *prediction = network_predict(darknet_network_, in_data);
    layer output_layer = darknet_network_->layers[darknet_network_->n - 1];

    output_layer.output = prediction;
    int nboxes = 0;
    int num_classes = output_layer.classes;
    detection *darknet_detections = get_network_boxes(darknet_network_, darknet_network_->w, darknet_network_->h, min_confidence_, .5, NULL, 0, &nboxes);

    do_nms_sort(darknet_detections, nboxes, num_classes, nms_threshold_);

    std::vector<RectClassScore<float>> detections;

    for (int i = 0; i < nboxes; i++)
    {
        int class_id = -1;
        float score = 0.f;
        //find the class
        for (int j = 0; j < num_classes; ++j)
        {
            if (darknet_detections[i].prob[j] >= min_confidence_)
            {
                if (class_id < 0)
                {
                    class_id = j;
                    score = darknet_detections[i].prob[j];
                }
            }
        }
        //if class found
        if (class_id >= 0)
        {
            RectClassScore<float> detection;

            detection.x = darknet_detections[i].bbox.x - darknet_detections[i].bbox.w / 2;
            detection.y = darknet_detections[i].bbox.y - darknet_detections[i].bbox.h / 2;
            detection.w = darknet_detections[i].bbox.w;
            detection.h = darknet_detections[i].bbox.h;
            detection.score = score;
            detection.class_type = class_id;
            //std::cout << detection.toString() << std::endl;

            detections.push_back(detection);
        }
    }
    //std::cout << std::endl;
    return detections;
}
} // namespace darknet

////////////////////////////////////////////////////////////////////////////////////////////////////

bool msg2cv(const sensor_msgs::ImageConstPtr &msg,Mat &img_out,Mat &img)
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
    cv::cvtColor(cv_ptr->image, img_out, CV_RGB2GRAY);
    //cv::imshow("img_out",img_out);
    img = cv_ptr->image;
}
////////////////////////////////////////////////////////////
// ############## 计算ROI #############

Rect computeROI(Size2i src_sz, Ptr<StereoMatcher> matcher_instance)
{
	int min_disparity = matcher_instance->getMinDisparity();
	int num_disparities = matcher_instance->getNumDisparities();
	int block_size = matcher_instance->getBlockSize();

	int bs2 = block_size / 2;
	int minD = min_disparity, maxD = min_disparity + num_disparities - 1;

	int xmin = maxD + bs2;
	int xmax = src_sz.width + minD - bs2;
	int ymin = bs2;
	int ymax = src_sz.height - bs2;

	Rect r(xmin, ymin, xmax - xmin, ymax - ymin);
	return r;
}

// ############## 计算距离 #############
float computeDistance(Mat& filtered_disp,Mat& pointcloud,const Mat& Q,int x,int y,float& distance)
{
	if (filtered_disp.empty())
	{
		//std::cout << "empty disparity map!" << std::endl;
        ROS_INFO("empty disparity map!");
		return 0;
	}
	reprojectImageTo3D(filtered_disp, pointcloud, Q, true);
	pointcloud *= 160;

	std::vector<cv::Mat> xyzSet;
	split(pointcloud, xyzSet);
	cv::Mat depth;
	xyzSet[2].copyTo(depth);

	distance = depth.at<float>(x, y);

	 
}


/////////////返回深度结果//////////////////////
Result camera_obj(const Mat &left, const Mat &right,int x,int y,int w,int h,Mat &Q,Mat &filtered_disp_vis)
{
    int new_width = left.cols;
    int new_height = left.rows;
    centerX = (int)(new_width/2);
    centerY = (int)(new_height/2);
   
    
	float distance;//均值
	float distance01;//左
	float distance02;//右
    float angle;    
    computeDistance(filtered_disp_vis,pointcloud,Q,x,y,distance01);
	computeDistance(filtered_disp_vis,pointcloud,Q,x+w,y+h,distance02);
	distance=(distance01 + distance02)/2;
    distance=(int)distance%20;

/*
    if (distance>20)
	{
        distance = 0;
		angle = 180;
    }  
	else
	{
        // 角度
		newX = (x+x+w)/2;
        //newY = (y+y+h)/2;
		angle = atan((newX-centerX)/(distance+0.1))*180/3.1415;              
	}
*/
    // 角度
	newX = (x+x+w)/2;
    //newY = (y+y+h)/2;
    int tmp= newX-centerX;
    //ROS_INFO("newX-centerX: %d",tmp);//100+
    //ROS_INFO("final_result.all_distance: %f", final_result.all_distance);  
	angle = atan((newX-centerX)/(distance*100+0.00001))*180/3.1415;  

    Result result;
    result.all_distance=distance;
	result.all_direction=angle;

	return result;

}

Mat slMat2cvMat(sl::Mat& input) {
	// Mapping between MAT_TYPE and CV_TYPE
	int cv_type = -1;
	switch (input.getDataType()) {
	case sl::MAT_TYPE_32F_C1: cv_type = CV_32FC1; break;
	case sl::MAT_TYPE_32F_C2: cv_type = CV_32FC2; break;
	case sl::MAT_TYPE_32F_C3: cv_type = CV_32FC3; break;
	case sl::MAT_TYPE_32F_C4: cv_type = CV_32FC4; break;
	case sl::MAT_TYPE_8U_C1: cv_type = CV_8UC1; break;
	case sl::MAT_TYPE_8U_C2: cv_type = CV_8UC2; break;
	case sl::MAT_TYPE_8U_C3: cv_type = CV_8UC3; break;
	case sl::MAT_TYPE_8U_C4: cv_type = CV_8UC4; break;
	default: break;
	}

	// Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
	// cv::Mat and sl::Mat will share a single memory structure
	return Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(sl::MEM_CPU));
}
//////////////////////////////////////////////////


void Yolo3DetectorNode::convert_rect_to_image_obj(std::vector<RectClassScore<float>> &in_objects, autoware_msgs::DetectedObjectArray &out_message)
{
    for (unsigned int i = 0; i < in_objects.size(); ++i)
    {
        {
            autoware_msgs::DetectedObject obj;

            obj.x = (in_objects[i].x / image_ratio_) - image_left_right_border_ / image_ratio_;
            obj.y = (in_objects[i].y / image_ratio_) - image_top_bottom_border_ / image_ratio_;
            obj.width = in_objects[i].w / image_ratio_;
            obj.height = in_objects[i].h / image_ratio_;
            if (in_objects[i].x < 0)
                obj.x = 0;
            if (in_objects[i].y < 0)
                obj.y = 0;
            if (in_objects[i].w < 0)
                obj.width = 0;
            if (in_objects[i].h < 0)
                obj.height = 0;

            obj.color.r = colors_[in_objects[i].class_type].val[0];
            obj.color.g = colors_[in_objects[i].class_type].val[1];
            obj.color.b = colors_[in_objects[i].class_type].val[2];
            obj.color.a = 1.0f;

            obj.score = in_objects[i].score;
            obj.label = in_objects[i].GetClassString();
            // DetectedObject[] objects
            out_message.objects.push_back(obj);
        }
    }
}

void Yolo3DetectorNode::rgbgr_image(image &im)
{
    int i;
    for (i = 0; i < im.w * im.h; ++i)
    {
        float swap = im.data[i];
        im.data[i] = im.data[i + im.w * im.h * 2];
        im.data[i + im.w * im.h * 2] = swap;
    }
}



image Yolo3DetectorNode::convert_ipl_to_image(Mat &mat_image)
{
    //image Yolo3DetectorNode::convert_ipl_to_image(const sensor_msgs::ImageConstPtr &msg)
    /*
    //cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(msg, "bgr8");//toCvCopy(image_source, sensor_msgs::image_encodings::BGR8);
    cv::Mat mat_image,mat_image_out;
    if (!msg2cv(msg,mat_image_out, mat_image))
    {
        // error in translate
        ROS_INFO("error in translate");
        //return;
    }
    */
 
    int network_input_width = yolo_detector_.get_network_width();
    int network_input_height = yolo_detector_.get_network_height();

    int image_height = mat_image.rows;
    int image_width = mat_image.cols;
    
  
    IplImage ipl_image;
    cv::Mat final_mat;

   

    if (network_input_width != image_width || network_input_height != image_height)
    {
        //final_mat = cv::Mat(network_input_width, network_input_height, CV_8UC3, cv::Scalar(0,0,0));
        image_ratio_ = (double)network_input_width / (double)mat_image.cols;

        cv::resize(mat_image, final_mat, cv::Size(), image_ratio_, image_ratio_);
        image_top_bottom_border_ = abs(final_mat.rows - network_input_height) / 2;
        image_left_right_border_ = abs(final_mat.cols - network_input_width) / 2;
        cv::copyMakeBorder(final_mat, final_mat,
                           image_top_bottom_border_, image_top_bottom_border_,
                           image_left_right_border_, image_left_right_border_,
                           cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));
    }
    else
        final_mat = mat_image;

    ipl_image = final_mat;

    unsigned char *data = (unsigned char *)ipl_image.imageData;
    int h = ipl_image.height;
    int w = ipl_image.width;
    int c = ipl_image.nChannels;
    int step = ipl_image.widthStep;
    int i, j, k;

    image darknet_image = make_image(w, h, c);

    for (i = 0; i < h; ++i)
    {
        for (k = 0; k < c; ++k)
        {
            for (j = 0; j < w; ++j)
            {
                darknet_image.data[k * w * h + i * w + j] = data[i * step + j * c + k] / 255.;
            }
        }
    }
    rgbgr_image(darknet_image);
    return darknet_image;
}


/*
void Yolo3DetectorNode::image_callback(const sensor_msgs::ImageConstPtr &msg_left)
{
    std::vector<RectClassScore<float>> detections;

    darknet_image_ = convert_ipl_to_image(msg_left);


    detections = yolo_detector_.detect(darknet_image_);

    //Prepare Output message
    autoware_msgs::DetectedObjectArray output_message;

    output_message.header = msg_left->header;

    convert_rect_to_image_obj(detections, output_message);
    // 发布
    publisher_objects_.publish(output_message);

    free(darknet_image_.data);
}
*/


void Yolo3DetectorNode::config_cb(const autoware_msgs::ConfigSsd::ConstPtr &param)
{
    score_threshold_ = param->score_threshold;
}

//////////////////////////////////////////////// 检测 ////////////////////////////////////////////
string itos(int i) // 将int 转换成string
{
    stringstream s;
    s << i;
    return s.str();
}  
//void Yolo3DetectorNode::callback(const sensor_msgs::ImageConstPtr &msg_left, const sensor_msgs::ImageConstPtr &msg_right)
void Yolo3DetectorNode::callback(Mat &mat_left, Mat &mat_right,Mat &Q,Mat &filtered_disp_vis)
{
    /*
    cv::Mat mat_left, mat_left_out,mat_right,mat_right_out;
    if (!msg2cv(msg_left,mat_left_out, mat_left) || !msg2cv(msg_right,mat_right_out, mat_right))
    {
        // error in translate
        return;
    }
   */
    cv::imshow("mat_left",mat_left);
    cv::imshow("mat_right",mat_right);

    int network_input_width = yolo_detector_.get_network_width();
    int network_input_height = yolo_detector_.get_network_height();


    image_ratio_ = (double)network_input_width / (double)mat_left.cols;
    // 左
    cv::Mat final_mat;
    cv::resize(mat_left, final_mat, cv::Size(), image_ratio_, image_ratio_);

    image_top_bottom_border_ = abs(final_mat.rows - network_input_height) / 2;
    image_left_right_border_ = abs(final_mat.cols - network_input_width) / 2;
    cv::copyMakeBorder(final_mat, final_mat,
        image_top_bottom_border_, image_top_bottom_border_,
        image_left_right_border_, image_left_right_border_,
        cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

    // 右
    cv::Mat final_mat_right;
    cv::resize(mat_right, final_mat_right, cv::Size(), image_ratio_, image_ratio_);

    
    cv::copyMakeBorder(final_mat_right, final_mat_right,
        image_top_bottom_border_, image_top_bottom_border_,
        image_left_right_border_, image_left_right_border_,
        cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

    int new_width = final_mat.cols;
    int new_height = final_mat.rows;
   
    centerX = (int)(new_width/2);
    centerY = (int)(new_height/2);

    /*
    if (param_viz)
    {
        cv::imshow(LEFT_WINDOW, mat_left);
        cv::imshow(RIGHT_WINDOW, mat_right);
        cv::waitKey(3);
    }
    */
   
    std::vector<RectClassScore<float>> detections;
   

    //darknet_image_ = convert_ipl_to_image(msg_left);  
    darknet_image_ = convert_ipl_to_image(final_mat);  
    detections = yolo_detector_.detect(darknet_image_);

    RectClassScore<float> detection;  
    // 矩形   
    for(int i=0;i<detections.size();i++){
       
            detection = detections[i];
    
            //ROS_INFO("detection.class_type: %d", detection.class_type);      
            //ROS_INFO("detection.score: %f", detection.score);
       
            String  classname[80] ={
                 "PERSON", "BICYCLE", "CAR", "MOTORBIKE", "AEROPLANE", "BUS", "TRAIN", "TRUCK", "BOAT", "TRAFFIC_LIGHT",//10
                 "FIRE_HYDRANT", "STOP_SIGN", "PARKING_METER", "BENCH", "BIRD", "CAT", "DOG", "HORSE", "SHEEP", "COW",//10
                 "ELEPHANT", "BEAR", "ZEBRA", "GIRAFFE","BACKPACK", "UMBRELLA", "HANDBAG", "TIE", "SUITCASE","FRISBEE",//10
                 "SKIS", "SNOWBOARD", "SPORTS_BALL","KITE", "BASEBALL_BAT", "BASEBALL_GLOVE", "SKATEBOARD", "SURFBOARD","TENNIS_RACKET", "BOTTLE",//10
                 "WINE_GLASS", "CUP","FORK", "KNIFE", "SPOON", "BOWL", "BANANA","APPLE","SANDWICH","ORANGE",//10
                 "BROCCOLI","CARROT", "HOT_DOG","PIZZA", "DONUT","CAKE", "CHAIR","SOFA", "POTTEDPLANT","BED",//10
                
                 "DININGTABLE", "TOILET", "TVMONITOR","LAPTOP","MOUSE","REMOTE","KEYBOARD", "CELL_PHONE", "MICROWAVE","OVEN",//10
                 "TOASTER", "SINK","REFRIGERATOR","BOOK", "CLOCK", "VASE","SCISSORS","TEDDY_BEAR", "HAIR_DRIER", "TOOTHBRUSH"//10
            };
         
             
             String  result_string1 =  "  class:  " ;
             String  result_string2 =  "  score:  " ;
             String  result_string3 =  "  distance:  " ;
             String  result_string4 =  "  direction:  " ;
             //String  result_class_type= itos(detection.class_type);
             String  result_class_type= classname[detection.class_type];
             String  result_score;
            
             stringstream ss;
             ss << detection.score << flush;
             result_score = ss.str();

             
             cv::rectangle(final_mat,Rect((int)detection.x,(int)detection.y,(int)detection.w,(int)detection.h),Scalar(255,0,0),3,1,0); 
            //cv::imshow("final_mat",final_mat); 

         
            Result final_result;        
            final_result =  camera_obj(final_mat, final_mat_right,(int)detection.x,(int)detection.y,(int)detection.w,(int)detection.h,Q,filtered_disp_vis);
                   
            //ROS_INFO("final_result.all_distance: %f", final_result.all_distance);  
            //ROS_INFO("final_result.all_direction: %f", final_result.all_direction); 
             String  result_all_distance;
            
             stringstream ss1;
             ss1 << final_result.all_distance << flush;
             result_all_distance = ss1.str();

              String  result_all_direction;
            
             stringstream ss2;
             ss2 << final_result.all_direction << flush;
             result_all_direction = ss2.str();

            String  result_string = result_string1 + result_class_type +result_string2+ result_score;
            String  result_string_ = result_string3 + result_all_distance +result_string4+ result_all_direction;
            CvFont font;
            cvInitFont(&font, CV_FONT_HERSHEY_COMPLEX, 0.5, 0.5, 1, 2, 8); 

            //IplImage *img=&(IplImage)final_mat;

            IplImage tmp = IplImage(final_mat);
            CvArr* arr = (CvArr*)&tmp;
            cvPutText(arr, result_string.c_str(), cvPoint((int)detection.x, (int)detection.y), &font, CV_RGB(0,255,0));
            cvPutText(arr, result_string_.c_str(), cvPoint((int)detection.x, (int)detection.y-20), &font, CV_RGB(0,255,0));
        
            cvShowImage("img", arr); 
            
            
            
    }
  
    free(darknet_image_.data);
}

////////////////////////////////////////////////  run

void Yolo3DetectorNode::Run()
{
   
    
    ros::NodeHandle private_node_handle("~");

    std::string network_definition_file;
    std::string pretrained_model_file;
    if (private_node_handle.getParam("network_definition_file", network_definition_file))
    {
        ROS_INFO("Network Definition File (Config): %s", network_definition_file.c_str());
    }
    else
    {
        ROS_INFO("No Network Definition File was received. Finishing execution.");
        return;
    }
    if (private_node_handle.getParam("pretrained_model_file", pretrained_model_file))
    {
        ROS_INFO("Pretrained Model File (Weights): %s", pretrained_model_file.c_str());
    }
    else
    {
        ROS_INFO("No Pretrained Model File was received. Finishing execution.");
        return;
    }

    private_node_handle.param<float>("score_threshold", score_threshold_, 0.5);
    ROS_INFO("[%s] score_threshold: %f", __APP_NAME__, score_threshold_);

    private_node_handle.param<float>("nms_threshold", nms_threshold_, 0.45);
    ROS_INFO("[%s] nms_threshold: %f", __APP_NAME__, nms_threshold_);

    ROS_INFO("Initializing Yolo on Darknet...");
    yolo_detector_.load(network_definition_file, pretrained_model_file, score_threshold_, nms_threshold_);
    ROS_INFO("Initialization complete.");

#if (CV_MAJOR_VERSION <= 2)
    cv::generateColors(colors_, 80);
#else
    generateColors(colors_, 80);
#endif

    publisher_objects_ = node_handle_.advertise<autoware_msgs::DetectedObjectArray>("/detection/vision_objects", 1);

    

    std::string config_topic("/config");
    config_topic += "/Yolo3";
    subscriber_yolo_config_ = node_handle_.subscribe(config_topic, 1, &Yolo3DetectorNode::config_cb, this);

    //zed
	sl::Camera zed;
	// Set configuration parameters
	sl::InitParameters init_params;
	init_params.camera_resolution = sl::RESOLUTION_HD1080;
	init_params.depth_mode = sl::DEPTH_MODE_PERFORMANCE;
	init_params.coordinate_units = sl::UNIT_METER;
	//dispatity filter
	cv::Ptr<DisparityWLSFilter> wls_filter;
	// Open the camera
	sl::ERROR_CODE err = zed.open(init_params);
	if (err != sl::SUCCESS) {
		
        ROS_INFO("Camera did not work");
		zed.close();
	
	}
    //ROS_INFO("Camera open");
    // Set runtime parameters after opening the camera
	sl::RuntimeParameters runtime_parameters;
	runtime_parameters.sensing_mode = sl::SENSING_MODE_STANDARD;
	// Prepare new image size to retrieve half-resolution images
	sl::Resolution image_size = zed.getResolution();
	int new_width = image_size.width / 2;
	int new_height = image_size.height / 2;

	// To share data between sl::Mat and cv::Mat, use slMat2cvMat()
	// Only the headers and pointer to the sl::Mat are copied, not the data itself

 	
	sl::Mat image_zed_left(new_width, new_height, sl::MAT_TYPE_8U_C1);
	sl::Mat image_zed_right(new_width, new_height, sl::MAT_TYPE_8U_C1);
   
	Mat left = slMat2cvMat(image_zed_left);
	Mat right = slMat2cvMat(image_zed_right);
	Size imageSize = left.size();
    
    bool no_downscale =false ;
    String algo = "bm";
    String filter = "wls_conf";//wls_conf
    int max_disp =160;
    double lambda = 8000.0 ;
	double sigma = 1.6 ;
	double vis_mult = 1.0;

	//fps view
	char str[10];
	char disstr[20];
	int wsize = 21;

    //set calibration to get Q matrix
	sl::CameraInformation cabibration;
	cabibration = zed.getCameraInformation();
	float temp[3] = {0};
	temp[0] = cabibration.calibration_parameters.T.x;
	temp[1] = cabibration.calibration_parameters.T.y;
	temp[2] = cabibration.calibration_parameters.T.z;

	Mat R1, R2, P1, P2, R,T,Q;
	R = (Mat_<double>(3, 3) << 0, 0, 0, 0, 0, 0, 0, 0, 0);
	T = (Mat_ <double > (3, 1) << 0.12, 0, 0);
	Rect validRoi[2];
	Mat cameraMatrix[2];
	Mat distCoeffs[2];
	cameraMatrix[0] = (Mat_<double>(3, 3) << 1397, 0, 1053.53, 0, 1397, 663.661, 0, 0, 1);
	cameraMatrix[1] = (Mat_<double>(3, 3) << 1403.42, 0, 1082.5, 0, 1403.42, 610.597, 0, 0, 1);
	distCoeffs[0] = (Mat_<double>(1, 5) << -0.170728, 0.0237634, 0, 0, 0);
	distCoeffs[1] = (Mat_<double>(1, 5) << -0.171119, 0.0238599, 0, 0, 0);
	stereoRectify(cameraMatrix[0], distCoeffs[0],
		cameraMatrix[1], distCoeffs[1],
		imageSize, R, T, R1, R2, P1, P2, Q,
		CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);
    //ROS_INFO("Q");
	
    while (1)
	{
				
		double fps;

		if (zed.grab(runtime_parameters) == sl::SUCCESS) {
           
            sl::Mat image_test_left,image_test_right;
            zed.retrieveImage(image_test_left, sl::VIEW_LEFT,sl::MEM_CPU, new_width, new_height); // Get the left image
            zed.retrieveImage(image_test_right, sl::VIEW_RIGHT,sl::MEM_CPU, new_width, new_height); // Get the right image
            Mat test_image_left = slMat2cvMat(image_test_left);
            Mat test_image_right = slMat2cvMat(image_test_right);
/*
            cv::namedWindow("test_image_left", WINDOW_AUTOSIZE);
	        cv::imshow("test_image_left", test_image_left);
            cv::namedWindow("test_image_right", WINDOW_AUTOSIZE);
	        cv::imshow("test_image_right", test_image_right);
*/	        
			// Retrieve the left image, depth image in half-resolution

			zed.retrieveImage(image_zed_left, sl::VIEW_LEFT_GRAY, sl::MEM_CPU, new_width, new_height);
			zed.retrieveImage(image_zed_right, sl::VIEW_RIGHT_GRAY, sl::MEM_CPU, new_width, new_height);
			
			Mat left_disp, right_disp;
			Mat filtered_disp;
			Rect ROI;
			Ptr<DisparityWLSFilter> wls_filter;
            //ROS_INFO("filtering with confidence (significantly better quality than wls_no_conf)");
						
			if (filter == "wls_conf") // filtering with confidence (significantly better quality than wls_no_conf)
			{
			
			    if (algo == "bm")
			    {
			        //! [matching]
			        Ptr<StereoBM> left_matcher = StereoBM::create(max_disp, wsize);
			        wls_filter = createDisparityWLSFilter(left_matcher);
			        Ptr<StereoMatcher> right_matcher = createRightMatcher(left_matcher);

			        //cvtColor(left, left_for_matcher, COLOR_BGR2GRAY);
			        //cvtColor(right, right_for_matcher, COLOR_BGR2GRAY);

		            left_matcher->compute(left, right, left_disp);
			        right_matcher->compute(right, left, right_disp);
			    }
			    //ROS_INFO("bm");
			    wls_filter->setLambda(lambda);
			    wls_filter->setSigmaColor(sigma);
			
			    wls_filter->filter(left_disp, left, filtered_disp, right_disp);
			
		        //ROS_INFO("wls_filter");
			    ROI = wls_filter->getROI();
			    if (!no_downscale)
			    {
			        // upscale raw disparity and ROI back for a proper comparison:
			        resize(left_disp, left_disp, Size(), 2.0, 2.0);
			        left_disp = left_disp*2.0;
			        //////////////////////////////////////////////////////
			        ROI = Rect(ROI.x * 2, ROI.y * 2, ROI.width * 2, ROI.height * 2);
			        //////////////////////////////////////////////////////
			    }
			}
			else
			{
			    //std::cout << "Unsupported filter";
                ROS_INFO("Unsupported filter");
			
			}
			
			//std::cout.precision(5);
            
			Mat raw_disp_vis;
			cv::ximgproc::getDisparityVis(left_disp, raw_disp_vis, vis_mult);
			//ROS_INFO("raw_disp_vis");
			Mat filtered_disp_vis;
			cv::ximgproc::getDisparityVis(filtered_disp, filtered_disp_vis, vis_mult);
            //ROS_INFO("filtered_disp_vis");
		    cv::namedWindow("filtered disparity", WINDOW_AUTOSIZE);
            cv::imshow("filtered disparity", filtered_disp_vis);
			
		
			int waittime = 10;
			cv::waitKey(waittime);
            // 调用
            Yolo3DetectorNode::callback(test_image_left, test_image_right,Q,filtered_disp_vis);
			
			
		}
	}



/*
    private_node_handle.param<bool>("viz", param_viz, false);

    cv::namedWindow(LEFT_WINDOW);
    cv::namedWindow(RIGHT_WINDOW);

    sub_left_img = new message_filters::Subscriber<sensor_msgs::Image>(node_handle_, "/zed/left/image", 5);
    sub_right_img = new message_filters::Subscriber<sensor_msgs::Image>(node_handle_, "/zed/right/image", 5);
    sync = new message_filters::Synchronizer<timeSynchronier>(timeSynchronier(10), *sub_left_img, *sub_right_img);
    sync->registerCallback(boost::bind(&Yolo3DetectorNode::callback, this, _1, _2));
*/
    


    ROS_INFO_STREAM(__APP_NAME__ << "");

    ros::spin();
    ROS_INFO("END Yolo");
}
