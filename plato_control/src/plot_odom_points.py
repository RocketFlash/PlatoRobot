#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import matplotlib.pyplot as plt
from geometry_msgs.msg import Vector3
import numpy

hl, = plt.plot([], [])
plt.title('Robot odometry')
plt.xlim((-10, 10))
plt.ylim((-10, 10))
plt.grid(True)


def update_line(hl, x, y):
    hl.set_xdata(numpy.append(hl.get_xdata(), x))
    hl.set_ydata(numpy.append(hl.get_ydata(), y))
    plt.draw()


def callback(data):
    global hl
    update_line(hl, data.x, data.y)


def listener():
    rospy.init_node('odom_points_listener', anonymous=True)
    rospy.Subscriber("odom_points", Vector3, callback)
    plt.show(block=True)


if __name__ == '__main__':
    listener()
