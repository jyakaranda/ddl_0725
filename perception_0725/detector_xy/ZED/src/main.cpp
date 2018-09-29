// ZED includes
#include <sl_zed/Camera.hpp>

// OpenCV includes
#include "opencv2/opencv.hpp"
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/ximgproc/disparity_filter.hpp"
#include "opencv2/imgcodecs.hpp"
//
//#include <cxmisc.h>
//#include <highgui.h>
//#include <cvaux.h>
#include <iostream>
#include <ctype.h>
// Sample includes
#include <SaveDepth.hpp>
#include <stdlib.h>
#include <vector>
#include <string>
#include <algorithm>
#include <ctype.h>
#include <stdarg.h>
#include <string.h>
#include <stdio.h>

using namespace sl;
using namespace std;


cv::Mat slMat2cvMat(Mat& input);

void calDisparity(const cv::Mat left, const cv::Mat  right, cv::Mat & disparity);
void printHelp();

int camera_detect(const cv::Mat left, const cv::Mat  right,int** bboxes) {

    // Create a ZED camera object
    //Camera zed;
    // Set configuration parameters
    //InitParameters init_params;
    // 分辨率
    //init_params.camera_resolution = RESOLUTION_HD1080;
    // 设置深度模式
    //init_params.depth_mode = DEPTH_MODE_PERFORMANCE;
    // 使用coordinate_*条目（坐标系，坐标单位...）来协调框架配置参数
   // init_params.coordinate_units = UNIT_METER;
    // 定义视差图
	cv::Mat disparity;

	//dispatity fliter

    // Open the camera
    /*
    ERROR_CODE err = zed.open(init_params);
    if (err != SUCCESS) {
        printf("%s\n", toString(err).c_str());
        zed.close();
        return 1; // Quit if an error occurred
    }
    */
    // Display help in console
    printHelp();
    // 设置运行参数
    //RuntimeParameters runtime_parameters;
    //runtime_parameters.sensing_mode = SENSING_MODE_STANDARD;

    // 准备新的图像尺寸以检索半分辨率图像
    //Resolution image_size = zed.getResolution();
    
   

    int new_width = left.cols;
    int new_height = left.rows;

    // To share data between sl::Mat and cv::Mat, use slMat2cvMat()
    // Only the headers and pointer to the sl::Mat are copied, not the data itself
    Mat image_zed_left(new_width, new_height, MAT_TYPE_8U_C1);
	Mat image_zed_right(new_width, new_height, MAT_TYPE_8U_C1);
    cv::Mat image_ocv_left = slMat2cvMat(image_zed_left);
	cv::Mat image_ocv_right = slMat2cvMat(image_zed_right);
	

   // Mat depth_image_zed(new_width, new_height, MAT_TYPE_8U_C4);
  //  cv::Mat depth_image_ocv = slMat2cvMat(depth_image_zed);
   // Mat point_cloud;

    // Loop until 'q' is pressed
    char key = ' ';
    while (key != 'q') {
         // 感知成功
        if (zed.grab(runtime_parameters) == SUCCESS) {

            // 以半分辨率检索左图像，深度图像
			zed.retrieveImage(image_zed_left, VIEW_LEFT_GRAY , MEM_CPU, new_width, new_height);
			zed.retrieveImage(image_zed_right, VIEW_RIGHT_GRAY, MEM_CPU, new_width, new_height);

            // zed.retrieveImage(depth_image_zed, VIEW_DEPTH, MEM_CPU, new_width, new_height);

            // Retrieve the RGBA point cloud in half-resolution
            // To learn how to manipulate and display point clouds, see Depth Sensing sample
            // zed.retrieveMeasure(point_cloud, MEASURE_XYZRGBA, MEM_CPU, new_width, new_height);

            // Display image and depth using cv:Mat which share sl:Mat data
            // 计算视差
			calDisparity(image_ocv_left, image_ocv_right, disparity);
            //cv::imshow("Image_left", image_ocv_left);
			//cv::imshow("Image_right", image_ocv_right);
			cv::imshow("Disparity", disparity);
            // Handle key event
            cv::waitKey(10);
            processKeyEvent(zed, key);
        }
    }
    zed.close();
    return 0;
}

/**
* Conversion function between sl::Mat and cv::Mat
**/
cv::Mat slMat2cvMat(Mat& input) {
    // Mapping between MAT_TYPE and CV_TYPE
    int cv_type = -1;
    switch (input.getDataType()) {
        case MAT_TYPE_32F_C1: cv_type = CV_32FC1; break;
        case MAT_TYPE_32F_C2: cv_type = CV_32FC2; break;
        case MAT_TYPE_32F_C3: cv_type = CV_32FC3; break;
        case MAT_TYPE_32F_C4: cv_type = CV_32FC4; break;
        case MAT_TYPE_8U_C1: cv_type = CV_8UC1; break;
        case MAT_TYPE_8U_C2: cv_type = CV_8UC2; break;
        case MAT_TYPE_8U_C3: cv_type = CV_8UC3; break;
        case MAT_TYPE_8U_C4: cv_type = CV_8UC4; break;
        default: break;
    }

    // Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
    // cv::Mat and sl::Mat will share a single memory structure
    return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(MEM_CPU));
}

/**
* This function displays help in console
**/
void printHelp() {
    std::cout << " Press 's' to save Side by side images" << std::endl;
    std::cout << " Press 'p' to save Point Cloud" << std::endl;
    std::cout << " Press 'd' to save Depth image" << std::endl;
    std::cout << " Press 'm' to switch Point Cloud format" << std::endl;
    std::cout << " Press 'n' to switch Depth format" << std::endl;
}
/*
int loadCalibData()
{
	// roi1 roi2 mapx1 mapy1 mapx2 mapy2
	try
	{
		cv::FileStorage fs("calib_paras.xml", cv::FileStorage::READ);
		cout << fs.isOpened() << endl;

		if (!fs.isOpened())
		{
			return (0);
		}

		cv::Size imageSize;
		cv::FileNodeIterator it = fs["imageSize"].begin();

		it >> imageSize.width >> imageSize.height;
		//  if (imageSize.width != m_frameWidth || imageSize.height != m_frameHeight)   {           return (-1);        }

		vector<int> roiVal1;
		vector<int> roiVal2;

		fs["leftValidArea"] >> roiVal1;

		m_Calib_Roi_L.x = roiVal1[0];
		m_Calib_Roi_L.y = roiVal1[1];
		m_Calib_Roi_L.width = roiVal1[2];
		m_Calib_Roi_L.height = roiVal1[3];

		fs["rightValidArea"] >> roiVal2;
		m_Calib_Roi_R.x = roiVal2[0];
		m_Calib_Roi_R.y = roiVal2[1];
		m_Calib_Roi_R.width = roiVal2[2];
		m_Calib_Roi_R.height = roiVal2[3];


		fs["QMatrix"] >> m_Calib_Mat_Q;
		fs["remapX1"] >> m_Calib_Mat_Remap_X_L;
		fs["remapY1"] >> m_Calib_Mat_Remap_Y_L;
		fs["remapX2"] >> m_Calib_Mat_Remap_X_R;
		fs["remapY2"] >> m_Calib_Mat_Remap_Y_R;

		cv::Mat lfCamMat;
		fs["leftCameraMatrix"] >> lfCamMat;
		m_FL = lfCamMat.at<double>(0, 0);

		m_Calib_Mat_Q.at<double>(3, 2) = -m_Calib_Mat_Q.at<double>(3, 2);

		m_Calib_Mat_Mask_Roi = cv::Mat::zeros(m_frameHeight, m_frameWidth, CV_8UC1);
		cv::rectangle(m_Calib_Mat_Mask_Roi, m_Calib_Roi_L, cv::Scalar(255), -1);

		m_BM.state->roi1 = m_Calib_Roi_L;
		m_BM.state->roi2 = m_Calib_Roi_R;

		m_Calib_Data_Loaded = true;

		string method;
		fs["rectifyMethod"] >> method;
		if (method != "BOUGUET")
		{
			return (-2);
		}

	}
	catch (std::exception& e)
	{
		m_Calib_Data_Loaded = false;
		return (-99);
	}

	return 1;


}*/
// 计算视差  左 右 视差    BM算法的状态参数
void calDisparity(const cv::Mat  _left, cv::Mat _right, cv::Mat & disparity)
{

	cv::Rect leftROI, rightROI;//左右视图的有效像素区域
	cv::Ptr<cv::StereoBM> bm = cv::StereoBM::create(16, 9);
    // 预处理滤波器类型  水平方向Sobel算子，默认类型
	bm->setPreFilterType(CV_STEREO_BM_XSOBEL);  //CV_STEREO_BM_NORMALIZED_RESPONSE����CV_STEREO_BM_XSOBEL
	//预处理滤波器窗口大小  参数必须为奇数值
    bm->setPreFilterSize(9);
    // 预处理滤波器的截断值
	bm->setPreFilterCap(31);

	bm->setBlockSize(15);
    // 最小视差，默认值为 0
	bm->setMinDisparity(0);
    // 视差窗口，即最大视差值与最小视差值之差
	bm->setNumDisparities(64);
    // 低纹理区域的判断阈值
	bm->setTextureThreshold(10);
    // 视差唯一性百分比
	bm->setUniquenessRatio(5);
    // 检查视差连通区域变化度的窗口大小
	bm->setSpeckleWindowSize(100);
    //  视差变化阈值
	bm->setSpeckleRange(32);
	bm->setROI1(leftROI);
	bm->setROI2(rightROI);
	copyMakeBorder(_left, _left, 0, 0, 80, 0, IPL_BORDER_REPLICATE);  //防止黑边
	copyMakeBorder(_right, _right, 0, 0, 80, 0, IPL_BORDER_REPLICATE);
	bm->compute(_left, _right, disparity);
	disparity = disparity.colRange(80, _left.cols);
	disparity.convertTo(disparity, CV_32F, 1.0 / 16);
}

