#!/usr/bin/env python
import rospy
import numpy as np
import cv2
from nav_msgs.msg import OccupancyGrid, MapMetaData
from parse_points import parse_points

def start():
    # Initialize publisher and node
    pub = rospy.Publisher('map', OccupancyGrid, queue_size=1)
    rospy.init_node('occupancy_publish')
    # Create empty map

    map_generated = np.zeros((2000, 2000, 3), np.uint8)
    resolution = 0.01
    # Draw lines on the map
    points = parse_points(file_name = 'room_318.txt')
    points_normilized = [(np.float32(x[0]/resolution), np.float32(x[1]/resolution))for x in points]
    font = cv2.FONT_HERSHEY_SIMPLEX

    lines = [[0,1], [1,23], [23,22], [22,21], [21,20],
             [20,19], [19,18], [18,17], [17,11], [11,13],
             [12,13], [12,10], [10,9], [9,0], [4,6],[6,14],[14,16],[16,8],
             [8,2],[2,3],[3,15],[15,5],[5,4]]
    # for i in range(len(points_normilized)-1):
    #     cv2.putText(map_generated,str(i),points_normilized[i], font, 1, (200,255,155), 2, cv2.LINE_AA)
    #     cv2.line(map_generated, points_normilized[i], points_normilized[i+1], (255, 255, 255), thickness=1)

    for i in range(len(lines)):
        # cv2.putText(map_generated,str(i),points_normilized[i], font, 1, (200,255,155), 2, cv2.LINE_AA)
        cv2.line(map_generated, points_normilized[lines[i][0]], points_normilized[lines[i][1]], (255, 255, 255), thickness=1)
    map_generated = cv2.flip(map_generated, 0)
    map_generated = cv2.flip(map_generated, 1)
    map_generated_gray = cv2.cvtColor(map_generated, cv2.COLOR_BGR2GRAY)

    # cv2.namedWindow('image', cv2.WINDOW_NORMAL)
    # cv2.imshow('image', map_generated_gray)
    # cv2.waitKey(delay=0)
    print(map_generated_gray)
    # Convert map in appropriate form
    map_generated_flat = list(map_generated_gray.flatten())
    map_generated_flat[:] = [x / 255.0 for x in map_generated_flat]
    map_generated_flat[:] = [x * 100.0 for x in map_generated_flat]
    map_width, map_height = map_generated_gray.shape

    # Create an Occupancy Grid message
    map_msg = OccupancyGrid(
        info=MapMetaData(
            width=map_width,
            height=map_height,
            resolution=resolution
        ),
        data=map_generated_flat
    )

    # ROS working loop
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub.publish(map_msg)
        rate.sleep()


if __name__ == '__main__':
    start()
