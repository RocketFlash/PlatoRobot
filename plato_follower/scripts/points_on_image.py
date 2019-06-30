#!/usr/bin/env python
from __future__ import print_function
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import math
import tf
import pcl_ros

hfov = 1.3962634
width = 1920
f = (width / 2) / math.tan(hfov / 2)


class image_converter:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber(
            "/plato/camera1/image_raw", Image, self.callback)
        self.velodyne_sub = rospy.Subscriber(
            "/velodyne_points", PointCloud2, self.callback_velodyne)
        hfov = 1.3962634
        width = 1920
        self.f = (width / 2) / math.tan(hfov / 2)
        self.points = np.zeros([350000, 3])
        print(self.f)

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        (rows, cols, channels) = cv_image.shape
        x_cor = self.f * np.divide(self.points[:, 0], self.points[:, 2])
        y_cor = self.f * np.divide(self.points[:, 1], self.points[:, 2])
        x_cor = x_cor[~np.isnan(x_cor)]
        y_cor = y_cor[~np.isnan(y_cor)]

        if x_cor.size == y_cor.size:
            for x, y in zip(x_cor, y_cor):
                if x > 0 and x < 1920 and y > 0 and y < 1080:
                    print("I am here")
                    cv2.circle(cv_image, (x, y), 63, (0, 0, 255), -1)
        #cv2.imshow("Image window", cv_image)
        # cv2.waitKey(3)

    def callback_velodyne(self, data):
        points = pc2.read_points(data, skip_nans=True,
                                 field_names=("x", "y", "z"))
        point_array = np.zeros([10000, 3])
        for indx, point in enumerate(points):
            point_array[indx][0] = point[0]
            point_array[indx][1] = point[1]
            point_array[indx][2] = point[2]
            # print(point[0], point[1], point[2])
        self.points = point_array


def main(args):
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)
    listener = tf.TransformListener()
    try:
        time_now = rospy.Time().now()
        listener.waitForTransform(
            '/camera', '/velodyne16', time_now, rospy.Duration(4.0))
        (trans, rot) = listener.lookupTransform(
            '/camera', '/velodyne16', time_now)
        print(trans)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        pass
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
