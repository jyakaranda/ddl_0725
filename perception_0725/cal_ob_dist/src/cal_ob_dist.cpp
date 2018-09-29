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
////////
#include "../../detector_xy/ZED/src/disparity_ref.cpp"
////////
#include "python/Python.h"
////////
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

/////////////// 
typedef struct sampledata
{
	int* classes;
  
	float* scores;
  int** bboxes;

}SLsampledata;

typedef struct 
{
   float* all_distance;
   float* all_direction;
} Result; 


void callback(const sensor_msgs::ImageConstPtr &msg_left, const sensor_msgs::ImageConstPtr &msg_right)
{
  cv::Mat mat_left, mat_right;
  if (!msg2cv(msg_left, mat_left) || !msg2cv(msg_right, mat_right))
  {
    // error in translate
    return;
  }
  /////////////////////////////////////////
  // TODO detect with mat_left and mag_right
  if (param_viz)
  {
    cv::imshow(LEFT_WINDOW, mat_left);
    cv::imshow(RIGHT_WINDOW, mat_right);
    cv::waitKey(3);

    Py_Initialize();    //初始化
    if ( !Py_IsInitialized() ) {  
        return -1;  
    }  
    string path = "~/SSD/notebooks";
    string chdir_cmd = string("sys.path.append(../../detector_xy"") + path + "/")";

    const char* cstr_cmd = chdir_cmd.c_str();
    PyRun_SimpleString("import sys");
    PyRun_SimpleString(cstr_cmd);
    
    //加载模块

    PyObject* moduleName = PyString_FromString("test"); //模块名
    PyObject* pModule = PyImport_Import(moduleName);
   
    if (!pModule) // 加载模块失败
    {
        cout << "[ERROR] Python get module failed." << endl;
        return 0;
    }
    cout << "[INFO] Python get module succeed." << endl;

    //加载函数
    PyObject* pv = PyObject_GetAttrString(pModule, "process_image");
    if (!pv || !PyCallable_Check(pv))  // 验证是否加载成功
    {
        cout << "[ERROR] Can't find funftion (test_add)" << endl;
        return 0;
    }
    cout << "[INFO] Get function (test_add) succeed." << endl;
    
    //设置参数
    PyObject* args = PyTuple_New(1);   // 1个参数
    PyObject* arg1 = PyInt_FromLong(mat_left);    // 参数一
   
    PyTuple_SetItem(args, 0, arg1);

    //调用函数
     PyObject* pRet = PyObject_CallObject(pv, args);

    // 获取参数
    if (pRet)  // 验证是否调用成功
    {
        PyObject* result = PyArg_ParseTuple(pRet);
       
    }


    int len1 =sizeof(result[0]) / sizeof(result[0][0]);//
    int len2 =sizeof(result[1]) / sizeof(result[1][0]);//
    int len3 =sizeof(result[2]) / sizeof(result[2][0]);//

    
    int* classes = new int[len1];
    for(int i=0;i<len1;i++){
       
         classes[i]=result[0][i];

    }

    float* scores = new float[len2];
    for(int i=0;i<len2;i++){
       
         scores[i]=result[1][i];

    }

    int** bboxes = new int[len3][4];
    for(int i=0;i<len3;i++){
               
         bboxes[i][0] = result[2][i][0];//ymin
         bboxes[i][1] = result[2][i][1];//xmin
         bboxes[i][2] = result[2][i][2];//ymax
         bboxes[i][3] = result[2][i][3];//xmax

    }

    SLsampledata data;

    data.classes= classes;
    data.scores= scores;
    data.bboxes= bboxes;
    
    float* distance_all;
		float* direction_all;
    Result result_all;
    result_all=camera_obj(mat_left, mat_right,bboxes);
    //数组长为len3
    distance_all=result_all.all_distance;//距离  距离返回值为10000的，实际检测有误 > 20
    direction_all= result_all.all_direction;//角度 当距离返回值为10000时，角度设置为180，实际整组数据测距有误

    Py_Finalize();      // 释放资源
    
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