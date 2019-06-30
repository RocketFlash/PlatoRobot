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
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>
#include <cmath>
#include <math.h>

#include <plato_hololens/MyObjects.h>
#include <plato_hololens/MyObject.h>
#include <plato_hololens/BoolStamped.h>
#include <plato_hololens/DenseImagePose.h>

typedef pcl::PointCloud<pcl::PointXYZ> MyPointCloud;
typedef MyPointCloud::ConstPtr MyPointCloudPtr;
typedef darknet_ros_msgs::BoundingBoxes::ConstPtr MyBoundingBoxesPtr;

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

tf::TransformListener* tf_listener_ptr;
tf::StampedTransform transform;
image_geometry::PinholeCameraModel pcam;
vector<plato_hololens::MyObject> detected_ojects;
// cv::Mat image_color(320, 240, CV_8UC3, cv::Scalar(0,0,0));

void callback(const ImageConstPtr& image_color_msg, const MyPointCloudPtr& pointcloud_msg,
              const CameraInfoConstPtr& info_color_msg, const MyBoundingBoxesPtr& bb_msg,
              const plato_hololens::BoolStampedConstPtr& is_moving_msg, const ImageConstPtr& orig_img_msg,
              const ros::Publisher *marker_pub, const ros::Publisher *pose_image_pub,const tf2_ros::Buffer *buff){

        MyPointCloud input_cloud_transformed_ros;
        pcl_ros::transformPointCloud("/camera", *pointcloud_msg, input_cloud_transformed_ros, *tf_listener_ptr);
        pcam.fromCameraInfo(*info_color_msg);
        vector<cv::Point3d> points_projection;
        vector<cv::Point3d> points_lidar;
        vector<cv::Point2d> points_on_image;
        geometry_msgs::TransformStamped camera_to_map;
        double eps_dist = 1;

        

        if(is_moving_msg->bool_val.data==false){
                try
                {
                        cv::Mat image_color = cv_bridge::toCvCopy(image_color_msg)->image;
                        cv::Mat image_color_orig = cv_bridge::toCvCopy(orig_img_msg)->image;
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

                        
                        
                        int count = 0;
                        for (auto box : bb_msg->bounding_boxes) {
                                double minDistance = 1000000;
                                cv::Point3d point_min;
                                point_min.x = 0;
                                point_min.y = 0;
                                point_min.z = 0;
                                
                                geometry_msgs::PointStamped goal_point_stamped;
                                geometry_msgs::PointStamped map_point_stamped;

                                cv::Point pt1, pt2;
                                string class_n = box.Class;
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
                                                
                                                if(sqrt(pow(points_projection[i].x - (box.xmin + ((box.xmax - box.xmin)/2)) ,2)  + pow(points_projection[i].y - (box.ymin + ((box.ymax-box.ymin)/3)),2)) < minDistance){
                                                        minDistance = sqrt(pow(points_projection[i].x - (box.xmin + ((box.xmax - box.xmin)/2)) ,2)  + pow(points_projection[i].y - (box.ymin + ((box.ymax-box.ymin)/3)),2));
                                                        point_min = points_lidar[i];
                                                        found = true;
                                                }
                                        }
                                }

                                // cv::imshow("view", image_color);
                                // cv::waitKey(1);

                                goal_point_stamped.header.frame_id = "/camera";
                                goal_point_stamped.header.stamp = ros::Time::now();

                                goal_point_stamped.point.x = point_min.x;
                                goal_point_stamped.point.y = point_min.y;
                                goal_point_stamped.point.z = point_min.z;
                                
                                try{
                                        tf2::doTransform( goal_point_stamped, map_point_stamped, camera_to_map);
                                        // ROS_INFO("\nobject_no %i class %s \n x coodinate is: %f , \n y coordinate is:%f , \n z coordinate is:%f \n",
                                        //          count, class_n.c_str(), map_point_stamped.point.x, map_point_stamped.point.y, map_point_stamped.point.z);
                                }
                                catch(tf::TransformException& ex) {
                                        ROS_ERROR("Received an exception trying to transform a point : %s", ex.what());
                                }
                                count++;

                                bool same_point = false;
                                if(detected_ojects.size()!=0){

                                        for(int j =0; j<detected_ojects.size(); j++){
                                                double dist_to_obj = sqrt(pow(detected_ojects[j].pose.x - map_point_stamped.point.x, 2) + pow(detected_ojects[j].pose.y - map_point_stamped.point.y, 2) + pow(detected_ojects[j].pose.z - map_point_stamped.point.z, 2));
                                                if(dist_to_obj < eps_dist){
                                                        ROS_INFO("\ndistance %f:", dist_to_obj);
                                                        same_point = true;
                                                        break;
                                                }
                                        }

                                        
                                        
                                }

                                if(!same_point){
                                        plato_hololens::MyObject new_obj;
                                        int id = -1;
                                        // ['person','table','chair','window','door','tv','bottle']
                                        string class_nm(class_n.c_str());
                                        if(class_nm.compare("Person")==0 || class_nm.compare("Man")==0 || class_nm.compare("Woman")==0){
                                                id = 0;
                                                plato_hololens::DenseImagePose new_hum;
                                                new_hum.pose.x = map_point_stamped.point.x;
                                                new_hum.pose.y = map_point_stamped.point.y;
                                                new_hum.pose.z = map_point_stamped.point.z;
                                                cv::Rect region_of_interest = cv::Rect(box.xmin, box.ymin, (box.xmax - box.xmin), (box.ymax - box.ymin));
                                                cv::Mat image_roi = image_color_orig(region_of_interest);
                                                cv_bridge::CvImage img_bridge;
                                                sensor_msgs::Image img_person_msg; 

                                                std_msgs::Header header;
                                                header.stamp = ros::Time::now();
                                                img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, image_roi);
                                                img_bridge.toImageMsg(img_person_msg); 
                                                new_hum.image =  img_person_msg;
                                                pose_image_pub->publish(new_hum);
                                        }else if(class_nm.compare("Table")==0){
                                                id = 1;
                                        }else if(class_nm.compare("Chair")==0){
                                                id = 2;
                                        }else if(class_nm.compare("Window")==0){
                                                id = 3;
                                        }else if(class_nm.compare("Door")==0){
                                                id = 4;
                                        }else if(class_nm.compare("Computer monitor")==0){
                                                id = 5;
                                        }else if(class_nm.compare("Bottle")==0){
                                                id = 6;
                                        }

                                        if(id>=0){
                                                new_obj.class_id = id;
                                                new_obj.pose.x = map_point_stamped.point.x;
                                                new_obj.pose.y = map_point_stamped.point.y;
                                                new_obj.pose.z = map_point_stamped.point.z;  
                                                detected_ojects.push_back(new_obj);
                                        }
                                }

                        }

                        visualization_msgs::MarkerArray marker_array;
                        for(int i=0; i<detected_ojects.size(); i++){
                                visualization_msgs::Marker marker;
                                
                                // Set the frame ID and timestamp.  See the TF tutorials for information on these.
                                marker.header.frame_id = "/map";
                                marker.header.stamp = ros::Time::now();

                                // Set the namespace and id for this marker.  This serves to create a unique ID
                                // Any marker sent with the same namespace and id will overwrite the old one
                                marker.ns = "basic_shapes";
                                marker.id = i;

                                // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
                                marker.type = visualization_msgs::Marker::SPHERE;

                                // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
                                marker.action = visualization_msgs::Marker::ADD;

                                // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
                                marker.pose.position.x = detected_ojects[i].pose.x;
                                marker.pose.position.y = detected_ojects[i].pose.y;
                                marker.pose.position.z = detected_ojects[i].pose.z;
                                marker.pose.orientation.x = 0.0;
                                marker.pose.orientation.y = 0.0;
                                marker.pose.orientation.z = 0.0;
                                marker.pose.orientation.w = 1.0;

                                // Set the scale of the marker -- 1x1x1 here means 1m on a side
                                marker.scale.x = 0.1;
                                marker.scale.y = 0.1;
                                marker.scale.z = 0.1;

                                // Set the color -- be sure to set alpha to something non-zero!
                                if(detected_ojects[i].class_id==0){
                                        marker.color.r = 0.0f;
                                        marker.color.g = 1.0f;
                                        marker.color.b = 0.0f;
                                }else if(detected_ojects[i].class_id==1){
                                        marker.color.r = 0.0f;
                                        marker.color.g = 0.0f;
                                        marker.color.b = 1.0f;
                                }else{
                                        marker.color.r = 1.0f;
                                        marker.color.g = 0.0f;
                                        marker.color.b = 0.0f;
                                }
                                marker.color.a = 1.0;
                                marker_array.markers.push_back(marker);
                        }
                        
                        marker_pub->publish(marker_array);
                }
                catch (cv_bridge::Exception& e)
                {
                        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", image_color_msg->encoding.c_str());
                }
                points_projection.clear();
                points_lidar.clear();
                points_on_image.clear();
        }
}


int main (int argc, char** argv)
{
        ros::init(argc, argv, "sub_pcl");
        tf_listener_ptr = new tf::TransformListener();
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        ros::NodeHandle nh;

        ros::Publisher objects_pub = nh.advertise<plato_hololens::MyObjects>("plato/hololens/detected_objects", 1);
        ros::Publisher marker_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker", 1);
        ros::Publisher pose_image_pub = nh.advertise<plato_hololens::DenseImagePose>("image_and_pose", 1);
        ros::Rate loop_rate(1);
        
        // create message filters to synchronize data from all sensors
        message_filters::Subscriber<Image> image_color_sub(nh,"/darknet_ros/detection_image", 1);
        message_filters::Subscriber<MyPointCloud> pointcloud_sub(nh,"/velodyne_points", 1);
        message_filters::Subscriber<CameraInfo> info_color_sub(nh,"/camera/camera_info", 1);
        message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> bounding_boxes_sub(nh,"/darknet_ros/bounding_boxes", 1);
        message_filters::Subscriber<plato_hololens::BoolStamped> is_moving_sub(nh,"/plato/is_moving", 1);
        message_filters::Subscriber<Image> orig_imag_sub(nh,"/camera/image_raw", 1);
        
        // synchronizer
        typedef sync_policies::ApproximateTime<Image, MyPointCloud, CameraInfo, darknet_ros_msgs::BoundingBoxes,plato_hololens::BoolStamped,Image> MySyncPolicy;
        Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_color_sub, pointcloud_sub, info_color_sub, bounding_boxes_sub,is_moving_sub, orig_imag_sub);
        sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4, _5, _6, &marker_pub, &pose_image_pub, &tfBuffer));
        cv::namedWindow("view",cv::WINDOW_NORMAL);
        cv::startWindowThread();

        ROS_INFO("I am here!");

        while (ros::ok())
        {
            plato_hololens::MyObjects msg;
            msg.objects = detected_ojects;
            msg.header.frame_id = "/camera";
            msg.header.stamp = ros::Time(0);
            objects_pub.publish(msg);

            ros::spinOnce();
            loop_rate.sleep();
        }

        return 0;

}
