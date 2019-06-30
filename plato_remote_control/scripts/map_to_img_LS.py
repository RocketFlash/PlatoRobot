from docutils.nodes import section
import rospy
import numpy as np
import cv2
import math
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from PIL import Image as im


def transform(self, data):
    arr = np.zeros((self.h, self.w, 3), dtype=np.uint8)
    for step_num, i in enumerate(data.ranges):
        if i > self.view_radius:
            continue
        x = int(self.w / 2 - i * math.cos(data.angle_increment * step_num) * self.zoom)
        y = int(self.h / 2 - i * math.sin(data.angle_increment * step_num) * self.zoom)
        arr[x, y] = [255, 255, 255]

    # drawing the position of the robot as orange arrow
    my_x = self.w / 2
    my_y = self.h / 2
    pt1 = (my_x - my_x / 5, my_y)
    pt2 = (my_x + my_x / 10, my_y - my_y / 10)
    pt3 = (my_x + my_x / 20, my_y)
    pt4 = (my_x + my_x / 10, my_y + my_y / 10)

    triangle_cnt = np.array([pt1, pt2, pt3, pt4])
    cv2.drawContours(arr, [triangle_cnt], 0, (255, 104, 0), -1)

    return arr


class map2img:
    def callback(self, data):
        arr = transform(self, data)

        # ******************************************************************
        # The section bounded by stars intended just for debugging purposes
        # p_img = im.fromarray(arr, 'RGB')
        # p_img.save('my.png')
        #
        # cv2.imshow("view", arr)
        # cv2.waitKey(1)
        # print(type(arr), arr.shape)
        # *******************************************************************

        msg = Image()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'camera'
        msg.encoding = 'bgr8'
        msg.height = arr.shape[0]
        msg.width = arr.shape[1]
        msg.step = arr.shape[1] * 3
        msg.data = arr.tostring()

        self.pub.publish(msg)

    def __init__(self, view_radius=10, zoom=15):
        """
		:type zoom: float
		:type view_radius: float
		"""
        if view_radius < 0:
            raise ValueError("view area can't be less than zero!")
        self.view_radius = view_radius
        if zoom < 1:
            raise ValueError(
                "zoom can't be less than one!")
        self.zoom = zoom  # originally resolution of scan data is 1 meter/pixel, so you can zoom it
        self.w = 2 * view_radius * zoom
        self.h = 2 * view_radius * zoom
        self.pub = rospy.Publisher('map_as_img', Image, queue_size=10)
        rospy.Subscriber('/scan', LaserScan, self.callback)
