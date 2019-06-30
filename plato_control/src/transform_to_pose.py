#!/usr/bin/env python

import rospy
import math
import tf
from geometry_msgs.msg import Pose
import numpy as np
import random


if __name__ == '__main__':
    rospy.init_node('transform_to_pose')
    listener1 = tf.TransformListener()
    listener2 = tf.TransformListener()

    plato_pose = rospy.Publisher(
        'plato/state/Pose', Pose, queue_size=1)
    map_pose = rospy.Publisher(
        'holo/pose', Pose, queue_size=1)
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans1, rot1) = listener1.lookupTransform(
                '/holo', '/chassis', rospy.Time(0))
            pose = Pose()
            pose.position.x = trans1[0]
            pose.position.y = trans1[1]
            pose.position.z = trans1[2]
            pose.orientation.x = rot1[0]
            pose.orientation.y = rot1[1]
            pose.orientation.z = rot1[2]
            pose.orientation.w = rot1[3]
            plato_pose.publish(pose)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("shit here 1")
            try:
                (trans1, rot1) = listener1.lookupTransform(
                    '/map', '/chassis', rospy.Time(0))
                pose = Pose()
                pose.position.x = trans1[0]
                pose.position.y = trans1[1]
                pose.position.z = trans1[2]
                pose.orientation.x = rot1[0]
                pose.orientation.y = rot1[1]
                pose.orientation.z = rot1[2]
                pose.orientation.w = rot1[3]
                plato_pose.publish(pose)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print("another shit here 0")
                continue
            continue
        try:
            (trans2, rot2) = listener2.lookupTransform(
                '/holo', '/map', rospy.Time(0))
            pose = Pose()
            pose.position.x = trans2[0]
            pose.position.y = trans2[1]
            pose.position.z = trans2[2]
            pose.orientation.x = rot2[0]
            pose.orientation.y = rot2[1]
            pose.orientation.z = rot2[2]
            pose.orientation.w = rot2[3]
            map_pose.publish(pose)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        rate.sleep()
