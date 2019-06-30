#!/usr/bin/env python

import math
from math import sin, cos, pi
from plato_api.constants import base_link_name
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3

rospy.init_node('odometry_publisher')
x = 0.0
y = 0.0
th = 0.0
vx = 0
vth = 0
odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)
current_time = rospy.Time.now()
last_time = rospy.Time.now()

odom_pub = rospy.Publisher("odom", Odometry, queue_size=1)
odom_broadcaster = tf.TransformBroadcaster()


def publish_tf(event):
    global vx, vth, x, y, th, current_time, last_time, odom_quat
    global odom_pub, odom_broadcaster

    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    tme = rospy.Time.now()
    odom.header.stamp = tme
    odom.header.frame_id = "odom"

    # set the position
    odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))

    # set the velocity
    odom.child_frame_id = base_link_name
    odom.twist.twist = Twist(Vector3(vx, 0, 0), Vector3(0, 0, vth))

    # publish the message
    odom_pub.publish(odom)

    odom_broadcaster.sendTransform(
        (x, y, 0.),
        odom_quat,
        tme,
        base_link_name,
        "odom"
    )


def callback(data):

    global vx, vth, x, y, th, current_time, last_time, odom_quat

    global odom_pub, odom_broadcaster

    vx = data.linear.x
    vth = data.angular.z

    current_time = rospy.Time.now()
    # compute odometry in a typical way given the velocities of the robot
    dt = (current_time - last_time).to_sec()
    delta_x = (vx * cos(th)) * dt
    delta_y = (vx * sin(th)) * dt
    delta_th = vth * dt
    x += delta_x
    y += delta_y
    th += delta_th

    # since all odometry is 6DOF we'll need a quaternion created from yaw
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)
    last_time = current_time


def listener():
    rospy.Subscriber("wheel_speed_twist", Twist, callback)
    rospy.Timer(rospy.Duration(0.05), publish_tf)
    # spin() simply keeps python from exiting until this node is stop
    rospy.spin()


if __name__ == '__main__':
    listener()
