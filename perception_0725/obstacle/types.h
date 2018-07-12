#ifndef MODULES_PERCEPTION_OBSTACLE_BASE_TYPES_H_
#define MODULES_PERCEPTION_OBSTACLE_BASE_TYPES_H_

#include <string>

#include "modules/perception/common/pcl_types.h"

namespace perception {

enum class ObjectType {
  UNKNOWN = 0,
  UNKNOWN_MOVABLE = 1,
  UNKNOWN_UNMOVABLE = 2,
  PEDESTRIAN = 3,
  BICYCLE = 4,
  VEHICLE = 5,
  MAX_OBJECT_TYPE = 6,
};

enum InternalObjectType {
  INT_BACKGROUND = 0,
  INT_SMALLMOT = 1,
  INT_PEDESTRIAN = 2,
  INT_NONMOT = 3,
  INT_BIGMOT = 4,
  INT_UNKNOWN = 5,
  INT_MAX_OBJECT_TYPE = 6,
};

enum class SensorType {
  VELODYNE_64 = 0,
  VELODYNE_16 = 1,
  RADAR = 2,
  CAMERA = 3,
  ULTRASONIC = 4,
  UNKNOWN_SENSOR_TYPE = 10,
};

enum class ScoreType {
  UNKNOWN_SCORE_TYPE = 0,
  SCORE_CNN = 1,
  SCORE_RADAR = 2,
};

typedef pcl_util::PointCloud PolygonType;
typedef pcl_util::PointDCloud PolygonDType;

using SeqId = uint32_t;

std::string GetObjectName(const ObjectType& obj_type);

std::string GetSensorType(SensorType sensor_type);

bool is_lidar(SensorType sensor_type);
bool is_radar(SensorType sensor_type);
bool is_camera(SensorType sensor_type);

}  // namespace perception

#endif  // MODULES_PERCEPTION_OBSTACLE_BASE_TYPES_H_
