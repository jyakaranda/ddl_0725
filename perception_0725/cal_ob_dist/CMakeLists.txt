SET( CMAKE_CXX_FLAGS "-std=c++11 -O3")
cmake_minimum_required(VERSION 2.8.3)
project(cal_ob_dist)


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
)


find_package(OpenCV REQUIRED)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES cal_ob_dist
 CATKIN_DEPENDS cv_bridge image_transport roscpp rospy sensor_msgs std_msgs
 DEPENDS OpenCV
)






include_directories(

  ${catkin_INCLUDE_DIRS}
)




add_executable(cal_ob_dist
  src/cal_ob_dist.cpp
)

add_dependencies(cal_ob_dist
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(cal_ob_dist
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)






