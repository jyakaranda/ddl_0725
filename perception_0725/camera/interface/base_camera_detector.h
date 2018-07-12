// @brief: The base class of camera 2D object detection

#ifndef MODULES_PERCEPTION_OBSTACLE_CAMERA_INTERFACE_BASE_CAMERA_DETECTOR_H_
#define MODULES_PERCEPTION_OBSTACLE_CAMERA_INTERFACE_BASE_CAMERA_DETECTOR_H_

#include <memory>
#include <string>
#include <vector>

#include "Eigen/Core"
#include "opencv2/opencv.hpp"

#include "modules/common/macro.h"
#include "modules/perception/lib/base/registerer.h"
#include "ddl_0725/perception_0725/common/camera.h"
#include "ddl_0725/perception_0725/obstacle/visual_object.h"


namespace perception
{

struct CameraDetectorInitOptions
{
  std::shared_ptr<CameraDistortD> intrinsic;
};

struct CameraDetectorOptions
{
  cv::Mat gray_frame;
  cv::Mat range_frame;
  std::shared_ptr<CameraDistortD> intrinsic;
  std::shared_ptr<Eigen::Matrix4d> extrinsic_ground2camera;
  std::shared_ptr<Eigen::Matrix4d> extrinsic_stereo;
};

class BaseCameraDetector
{
public:
  BaseCameraDetector() {}
  virtual ~BaseCameraDetector() {}

  virtual bool Init(const CameraDetectorInitOptions &options =
                        CameraDetectorInitOptions()) = 0;

  // @brief: Object detection on image from camera
  // @param [in]: image frame from camera
  // @param [in/out]: detected objects
  virtual bool Detect(const cv::Mat &frame,
                      const CameraDetectorOptions &options,
                      std::vector<std::shared_ptr<VisualObject>> *objects) = 0;

  virtual bool Multitask(const cv::Mat &frame,
                         const CameraDetectorOptions &options,
                         std::vector<std::shared_ptr<VisualObject>> *objects,
                         cv::Mat *mask)
  {
    return true;
  }

  virtual bool Lanetask(const cv::Mat &frame, cv::Mat *mask) { return true; }
  // @brief: Extract deep learning ROI features for each object
  // @param [in/out]: detected objects, with 2D bbox and its features
  virtual bool Extract(std::vector<std::shared_ptr<VisualObject>> *objects) = 0;

  virtual std::string Name() const = 0;

private:
  DISALLOW_COPY_AND_ASSIGN(BaseCameraDetector);
};

REGISTER_REGISTERER(BaseCameraDetector);
#define REGISTER_CAMERA_DETECTOR(name) REGISTER_CLASS(BaseCameraDetector, name)

} // namespace perception

#endif // MODULES_PERCEPTION_OBSTACLE_CAMERA_INTERFACE_BASE_CAMERA_DETECTOR_H_
