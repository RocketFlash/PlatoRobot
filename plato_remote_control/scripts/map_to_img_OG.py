from docutils.nodes import section
import cv2
import rospy
import numpy as np
import tf
import copy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image
from PIL import Image as im


class map2img:

    def pose_to_pixel(self, pose):
        """
        Gets pose in map frame and returns pixel coordinates in grid
        :param PoseStamped pose: Pose the object(in map frame) intended to transform to cell in grid
        """
        return (int(self.border_p + (pose.pose.position.y - self.origin_map_m.y) / self.resolution_map),
                int(self.border_p + (pose.pose.position.x - self.origin_map_m.x) / self.resolution_map))

    def get_marker(self, center, src_frame, dst_frame, scale = 1):
        """generates a marker which represents robot pose and return it as 2d points list
        :param PoseStamped center: pose of moving object no matter in which frame
        :param string src_frame: frame of moving object (e.q. chassis or moving base link)
        :param string dst_frame: frame in which you want to put the marker of moving object
        :param float scale: optionally you can scale marker if its size not large enough
        :return: marker's contour on grid
        :rtype: list of 2D points
        """
        my_pose = self.listener.transformPose(src_frame, center)
        poses = [copy.deepcopy(my_pose) for _ in range(4)]

        poses[0].pose.position.x += 0.4*scale
        poses[1].pose.position.x -= 0.2*scale
        poses[1].pose.position.y += 0.2*scale
        poses[2].pose.position.x -= 0.1*scale
        poses[3].pose.position.x -= 0.2*scale
        poses[3].pose.position.y -= 0.2*scale

        poses = [self.listener.transformPose(dst_frame, i) for i in poses]

        return [self.pose_to_pixel(i) for i in poses]

    def get_view(self, current_pose):
        """
        Produces image intent to publish to some topic
        :param PoseStamped current_pose: pose of moving object no matter in which frame
        :return: Area in the grid occupied by view_radius with marker of robot in center
        :rtype: ndarray
        """
        cur_y, cur_x = self.pose_to_pixel(current_pose)

        x_lower = cur_x - int(self.view_radius_m / self.resolution_map)
        x_higher = cur_x + int(self.view_radius_m / self.resolution_map)
        y_lower = cur_y - int(self.view_radius_m / self.resolution_map)
        y_higher = cur_y + int(self.view_radius_m / self.resolution_map)

        view = copy.deepcopy(self.grid[x_lower:x_higher, y_lower:y_higher])

        points = []
        for i in self.get_marker(current_pose, "chassis", "map", scale=1.5):
            points.append(
                (i[0] - y_lower, i[1] - x_lower))

        marker_cnt = np.array(points)
        cv2.drawContours(view, [marker_cnt], 0, (255, 104, 0), -1)
        return view

    def callback_make_map(self, grid):
        self.resolution_map = grid.info.resolution
        self.border_p = int(self.view_radius_m / grid.info.resolution)
        # Since it is necessary to return the image within the radius of view_radius_m
        # around the robot, it is need to add a border around the grid so as index not to be
        # out of bounds of array when the one is at the very edge of the map
        self.w_p = grid.info.width + 2 * self.border_p
        self.h_p = grid.info.height + 2 * self.border_p
        self.origin_map_m = grid.info.origin.position

        self.grid = np.zeros((self.w_p, self.h_p, 3), dtype=np.uint8)

        probability = 50
        for i, e in enumerate(grid.data):
            if e > probability:
                x = i % (self.w_p - 2 * self.border_p) + self.border_p
                y = i / (self.w_p - 2 * self.border_p) + self.border_p
                self.grid[x, y] = [255, 255, 255]
        self.grid_is_ready = True


    def callback_odom(self, msg):
        if not self.grid_is_ready:
            return

        current_pose = PoseStamped()        # current pose in odomerty frame
        current_pose.header = msg.header
        current_pose.pose = msg.pose.pose

        view = self.get_view(current_pose)

        #******************************************************************
        # The section bounded by stars intended just for debugging purposes
        # print self.pose_to_pixel(current_pose)
        # p_img = im.fromarray(view, 'RGB')
        # p_img.save('my.png')
        #
        # cv2.imshow("view",view)
        # cv2.waitKey(1)
        #*******************************************************************

        img = Image(height=2 * self.border_p, width=2 * self.border_p, data=view)

        self.pub.publish(img)

    def __init__(self, view_radius=10):
        """
        :param: For clarity, params ended with _m suffix described in meters, with _p suffix in pixels
		:param float view_radius: Size of area intent to be publish described by this parameter (in meters)
		"""
        if view_radius <= 0:
            raise ValueError("view area can't be equal or less than zero!")
        self.grid_is_ready = False
        self.view_radius_m = view_radius
        self.pub = rospy.Publisher('map_as_img', Image, queue_size=10)
        rospy.Subscriber('/map', OccupancyGrid, self.callback_make_map)
        rospy.Subscriber('/odom', Odometry, self.callback_odom)
        self.listener = tf.TransformListener()
