cmake_minimum_required(VERSION 2.8.3)
project(plato_hololens)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_ros
  image_geometry
  image_transport
  cv_bridge
  message_generation
  darknet_ros_msgs
  sensor_msgs
)

find_package(OpenCV REQUIRED)

message(STATUS "OpenCV_INCLUDE_DIRS = ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV_LIBS = ${OpenCV_LIBS}")

add_message_files(
  FILES
  DenseHuman.msg
  DenseHumans.msg
  DenseImagePose.msg
  Human.msg
  Humans.msg
  HumanBodyPart.msg
  MyObject.msg
  MyObjects.msg
  BoolStamped.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

add_executable(merge_pointcloud_and_image src/merge_pointcloud_and_image.cpp)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(merge_pointcloud_and_image
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)