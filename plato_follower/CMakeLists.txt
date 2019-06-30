cmake_minimum_required(VERSION 2.8.3)
project(plato_follower)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_ros
  image_geometry
  image_transport
  cv_bridge
  darknet_ros_msgs
)

find_package(OpenCV REQUIRED)

message(STATUS "OpenCV_INCLUDE_DIRS = ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV_LIBS = ${OpenCV_LIBS}")

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(convert_pointcloud src/convert_pointcloud.cpp)
target_link_libraries(convert_pointcloud
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
