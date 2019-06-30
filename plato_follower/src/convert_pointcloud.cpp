#include <ros/ros.h>
#include <ros/console.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include "pcl_ros/transforms.h"
#include <boost/foreach.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <image_transport/image_transport.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <cmath>

typedef pcl::PointCloud<pcl::PointXYZ> MyPointCloud;
typedef MyPointCloud::ConstPtr MyPointCloudPtr;
typedef darknet_ros_msgs::BoundingBoxes::ConstPtr MyBoundingBoxesPtr;

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

tf::TransformListener* tf_listener_ptr;
tf::StampedTransform transform;
image_geometry::PinholeCameraModel pcam;


void callback(const ImageConstPtr& image_color_msg, const MyPointCloudPtr& pointcloud_msg,
              const CameraInfoConstPtr& info_color_msg, const MyBoundingBoxesPtr& bb_msg,
              const ros::Publisher *pub, const tf2_ros::Buffer *buff){

        MyPointCloud input_cloud_transformed_ros;
        pcl_ros::transformPointCloud("/camera", *pointcloud_msg, input_cloud_transformed_ros, *tf_listener_ptr);
        pcam.fromCameraInfo(*info_color_msg);
        vector<cv::Point3d> points_projection;
        vector<cv::Point3d> points_lidar;
        vector<cv::Point2d> points_on_image;
        geometry_msgs::PoseStamped goal_point;
        geometry_msgs::PoseStamped map_point;
        geometry_msgs::TransformStamped camera_to_map;

        try
        {
                cv::Mat image_color = cv_bridge::toCvCopy(image_color_msg)->image;
                // cv::cvtColor(image_color,image_color, cv::COLOR_RGB2BGR);
                double maxradius=5;
                double maxrange=20;
                double currentrange = 0;
                BOOST_FOREACH (const pcl::PointXYZ& pt, input_cloud_transformed_ros.points){
                        cv::Point2d proj = pcam.project3dToPixel(cv::Point3d(pt.x, pt.y, pt.z));
                        if(proj.x > 0 && pt.z > 0 && proj.x < image_color.cols
                           && proj.y > 0 && proj.y < image_color.rows) {
                                points_projection.push_back(cv::Point3d(proj.x, proj.y, pt.z));
                                points_lidar.push_back(cv::Point3d(pt.x, pt.y, pt.z));
                        }
                }

                camera_to_map = buff->lookupTransform("map", "camera", ros::Time(0),ros::Duration(0.5));

                double minDistance = 1000000;
                cv::Point3d point_min;
                point_min.x = 0;
                point_min.y = 0;
                point_min.z = 0;
                for (auto box : bb_msg->bounding_boxes) {
                        cv::Point pt1, pt2;
                        pt1.x = box.xmin;
                        pt1.y = box.ymin;
                        pt2.x = box.xmax;
                        pt2.y = box.ymax;
                        cv::rectangle(image_color, pt1, pt2, CV_RGB(255,0,0), 2);
                        bool found = false;
                        for (int i=0; i<points_projection.size(); i++) {
                                if(points_projection[i].x > box.xmin &&
                                   points_projection[i].y > box.ymin &&
                                   points_projection[i].x < box.xmax &&
                                   points_projection[i].y < box.ymax) {
                                        currentrange = min(points_projection[i].z, maxrange);
                                        cv::circle(image_color,cv::Point2d(points_projection[i].x, points_projection[i].y),maxradius * (1-currentrange/maxrange),cv::Scalar(0,255*(currentrange/maxrange),255*(1-currentrange/maxrange)),-1,8,0);
                                        if(points_lidar[i].z < minDistance && points_lidar[i].y > 0.2) {
                                                minDistance = points_lidar[i].z;
                                                point_min = points_lidar[i];
                                                found = true;
                                        }
                                }
                        }


                        goal_point.header.frame_id = "/camera";
                        goal_point.header.stamp = ros::Time(0);
                        goal_point.pose.position.x = point_min.x;
                        goal_point.pose.position.y = point_min.y;
                        goal_point.pose.position.z = point_min.z;
                        double angle_to_rotate = atan2(point_min.y,point_min.x) + atan2(0, -1)/2;
                        goal_point.pose.orientation.x = 0;
                        goal_point.pose.orientation.y = 0;
                        goal_point.pose.orientation.z = 1;
                        goal_point.pose.orientation.w = angle_to_rotate;
                        try{
                                tf2::doTransform( goal_point, map_point, camera_to_map);
                                ROS_INFO("\n x coodinate is: %f , \n y coordinate is:%f , \n z coordinate is:%f \n",
                                         map_point.pose.position.x,map_point.pose.position.y, goal_point.pose.position.z);
                                pub->publish(map_point);
                        }
                        catch(tf::TransformException& ex) {
                                ROS_ERROR("Received an exception trying to transform a point : %s", ex.what());
                        }
                        break;

                }
                cv::imshow("view", image_color);
                cv::waitKey(1);
        }
        catch (cv_bridge::Exception& e)
        {
                ROS_ERROR("Could not convert from '%s' to 'bgr8'.", image_color_msg->encoding.c_str());
        }
        points_projection.clear();
        points_lidar.clear();
        points_on_image.clear();
}


int main (int argc, char** argv)
{
        ros::init(argc, argv, "sub_pcl");
        tf_listener_ptr = new tf::TransformListener();
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        ros::NodeHandle nh;
        ros::Publisher goal_pub = nh.advertise<geometry_msgs::PoseStamped>("plato/follower/goal", 1);
        message_filters::Subscriber<Image> image_color_sub(nh,"/darknet_ros/detection_image", 1);
        message_filters::Subscriber<MyPointCloud> pointcloud_sub(nh,"/velodyne_points", 1);
        message_filters::Subscriber<CameraInfo> info_color_sub(nh,"/plato/camera1/camera_info", 1);
        message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> bounding_boxes_sub(nh,"/darknet_ros/bounding_boxes", 1);
        typedef sync_policies::ApproximateTime<Image, MyPointCloud, CameraInfo, darknet_ros_msgs::BoundingBoxes> MySyncPolicy;
        Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_color_sub, pointcloud_sub, info_color_sub, bounding_boxes_sub);
        sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4, &goal_pub, &tfBuffer));
        cv::namedWindow("view",cv::WINDOW_NORMAL);
        cv::startWindowThread();

        ros::spin();

}
