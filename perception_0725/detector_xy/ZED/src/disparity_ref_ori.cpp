
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/ximgproc/disparity_filter.hpp"
#include <iostream>
#include <string>
#include <vector>
#include <mutex>
#include <sl_zed/Camera.hpp>


using namespace cv;
using namespace cv::ximgproc;

//mutex 
std::mutex mtx;
//function declaration
CV_EXPORTS void setMouseCallback(const std::string& winname, MouseCallback onMouse, void* userdata = 0);
Mat slMat2cvMat(sl::Mat& input);
float computeDistance(Mat& filtered_disp,Mat& pointcloud,const Mat& Q,float& distance);
Rect computeROI(Size2i src_sz, Ptr<StereoMatcher> matcher_instance);
//distance paras
Mat image;

bool left_mouse = false;
int pic_info[4];




Mat pointcloud;
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


static void onMouse(int event, int x, int y, int /*flags*/, void* /*param*/){


	Mat mouse_show;
	image.copyTo(mouse_show);

	if (event == CV_EVENT_LBUTTONDOWN)
	{
	
		pic_info[0] = x;
		pic_info[1] = y;
		std::cout << "x:" << pic_info[0] << "y:" << pic_info[1] << std::endl;
	
		left_mouse = true;

	}
	else if (event == CV_EVENT_LBUTTONUP)
	{
		left_mouse = false;
	}
}




int main(int argc, char** argv)
{

	
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
		printf("%s\n", toString(err).c_str());
		zed.close();
		return 1; // Quit if an error occurred
	}

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

	
	//opencv
	CommandLineParser parser(argc, argv, keys);
	parser.about("Disparity Filtering Demo");
	if (parser.has("help"))
	{
		parser.printMessage();
		return 0;
	}

	String dst_path = parser.get<String>("dst_path");
	String dst_raw_path = parser.get<String>("dst_raw_path");
	String dst_conf_path = parser.get<String>("dst_conf_path");
	String algo = parser.get<String>("algorithm");
	String filter = parser.get<String>("filter");

	bool no_display = parser.has("no-display");
	bool no_downscale = parser.has("no-downscale");
	int max_disp = parser.get<int>("max_disparity");
	double lambda = parser.get<double>("wls_lambda");
	double sigma = parser.get<double>("wls_sigma");
	double vis_mult = parser.get<double>("vis_mult");
	//fps view
	char str[10];
	char disstr[20];
	int wsize = 21;

	if (!parser.check())
	{
		parser.printErrors();
		return -1;
	}
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
	std::cout << Q << std::endl;
	//cabibration.calibration_parameters.T 

    ///////////////////
	
	float distance;
	while (1)
	{
		
		double matching_time, filtering_time, time_per_frame;
		double fps;
		time_per_frame = (double)getTickCount();
		if (zed.grab(runtime_parameters) == sl::SUCCESS) {

			// Retrieve the left image, depth image in half-resolution
			zed.retrieveImage(image_zed_left, sl::VIEW_LEFT_GRAY, sl::MEM_CPU, new_width, new_height);
			zed.retrieveImage(image_zed_right, sl::VIEW_RIGHT_GRAY, sl::MEM_CPU, new_width, new_height);
			
			Mat left_disp, right_disp;
			Mat filtered_disp;
			Rect ROI;
			Ptr<DisparityWLSFilter> wls_filter;
			
			if (max_disp <= 0 || max_disp % 16 != 0)
			{
			std::cout << "Incorrect max_disparity value: it should be positive and divisible by 16";
			return -1;
			}
			if (wsize <= 0 || wsize % 2 != 1)
			{
			std::cout << "Incorrect window_size value: it should be positive and odd";
			return -1;
			}
			if (filter == "wls_conf") // filtering with confidence (significantly better quality than wls_no_conf)
			{
			//else
			//{
			//}

			if (algo == "bm")
			{
			//! [matching]
			Ptr<StereoBM> left_matcher = StereoBM::create(max_disp, wsize);
			wls_filter = createDisparityWLSFilter(left_matcher);
			Ptr<StereoMatcher> right_matcher = createRightMatcher(left_matcher);

			//cvtColor(left, left_for_matcher, COLOR_BGR2GRAY);
			//cvtColor(right, right_for_matcher, COLOR_BGR2GRAY);

		    matching_time = (double)getTickCount();
			left_matcher->compute(left, right, left_disp);
			right_matcher->compute(right, left, right_disp);
			matching_time = ((double)getTickCount() - matching_time) / getTickFrequency();
			//! [matching]
			}
			//! [filtering]
			wls_filter->setLambda(lambda);
			wls_filter->setSigmaColor(sigma);
			filtering_time = (double)getTickCount();
			wls_filter->filter(left_disp, left, filtered_disp, right_disp);
			filtering_time = ((double)getTickCount() - filtering_time) / getTickFrequency();
			//! [filtering]
			// Get the ROI that was used in the last filter call:
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
			std::cout << "Unsupported filter";
			return -1;
			}
			
			std::cout.precision(5);
			cv::namedWindow("left", WINDOW_AUTOSIZE);
			cv::imshow("left", left);
			cv::namedWindow("right", WINDOW_AUTOSIZE);
			cv::imshow("right", right);
			//! [visualization]
			Mat raw_disp_vis;
			cv::ximgproc::getDisparityVis(left_disp, raw_disp_vis, vis_mult);
			//cv::namedWindow("raw disparity", WINDOW_AUTOSIZE);
			//cv::imshow("raw disparity", raw_disp_vis);
			Mat filtered_disp_vis;
			cv::ximgproc::getDisparityVis(filtered_disp, filtered_disp_vis, vis_mult);
			//compute distance
			
			//
			//Mat dis8, xyz, Q;
			int numdis = 0;	
			//mtx.lock();

			//计算距离
			//////////////////////////////////
		
			computeDistance(filtered_disp_vis,pointcloud,Q,distance);
			//////////////////////////////////

			std::cout << "distance" << distance << std::endl;
			//mtx.unlock();
			cv::namedWindow("filtered disparity", WINDOW_AUTOSIZE);
			cv::setMouseCallback("filtered disparity", onMouse, 0);
			//compute fps
			time_per_frame = ((double)getTickCount() - time_per_frame) / getTickFrequency();
			fps = 1.0 / time_per_frame;
			sprintf(str, "%.2f", fps);
			std::string fpsString("FPS:");
			fpsString += str;
			cv::putText(filtered_disp_vis, fpsString, cv::Point(5, 20), cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 0, 0));//cv scalar store BGR instead of RGB
			
			//std::string disString("Input the point:");
			//sprintf(disstr);
			std::string disString("Distance to point:");
			sprintf(disstr, "%.2f", distance);
			disString += disstr;
			cv::putText(filtered_disp_vis, disString, cv::Point(5, 40), cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(255, 0, 0));
			
			circle(filtered_disp_vis, cvPoint(pic_info[0], pic_info[1]), 10, CV_RGB(50, 0, 200), 2, 8, 0);
			cv::imshow("filtered disparity", filtered_disp_vis);
			int waittime = 10;
			cv::waitKey(waittime);
			
			
		}
	}
}


// ############## 计算距离 #############
float computeDistance(Mat& filtered_disp,Mat& pointcloud,const Mat& Q,float& distance)
{
	if (filtered_disp.empty())
	{
		std::cout << "empty disparity map!" << std::endl;
		return 0;
	}
	reprojectImageTo3D(filtered_disp, pointcloud, Q, true);
	pointcloud *= 160;
	/*for (int y = 0; y < pointcloud.rows; ++y)
	{
		for (int x = 0; x < pointcloud.cols; ++x)
		{
			cv::Point3f point = pointcloud.at<cv::Point3f>(y, x);
			point.y = -point.y;
			pointcloud.at<cv::Point3f>(y, x) = point;
		}
	}*/
	std::vector<cv::Mat> xyzSet;
	split(pointcloud, xyzSet);
	cv::Mat depth;
	xyzSet[2].copyTo(depth);
  
	
	
	distance = depth.at<float>(pic_info[0], pic_info[1]);


    /*
	std::cout << "x:" << pic_info[0] <<" "<<"y:"<<pic_info[1]<<std::endl;
	float mouseDistance = 0;
	mouseDistance = depth.at<float>(pic_info[0], pic_info[1]);
	std::cout << "mouseDistance:" << mouseDistance << std::endl;
	*/
	
}


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
