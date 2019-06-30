#!/usr/bin/env python
import unittest
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState


class TestGazebo(unittest.TestCase):
    def test_1_joint_state(self):
        msg = rospy.wait_for_message("/joint_states", JointState)
        joints = ["right_wheel_hinge", "left_wheel_hinge"]
        self.assertItemsEqual(joints, msg.name,
                             "expected {0} list is not equal to: {1}".format(joints, msg.name))

        self.assertAlmostEqual(0, msg.position[0], 2,
                               "wheel {0} should be in zero, but {1}".format(msg.name[0], msg.position[0]))
        self.assertAlmostEqual(0, msg.position[1], 2,
                               "wheel {0} should be in zero, but {1}".format(msg.name[1], msg.position[1]))

    def test_2_cmd_vel(self):
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=3)
        time.sleep(1)

        twist = Twist()
        twist.linear.x = 10
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 1
        pub.publish(twist)
        time.sleep(1)

        msg = rospy.wait_for_message("/joint_states", JointState)
        self.assertNotAlmostEqual(0, msg.position[0], 0,
                               "wheel {0} should NOT be in zero, but {1}".format(msg.name[0], msg.position[0]))
        self.assertNotAlmostEqual(0, msg.position[1], 0,
                               "wheel {0} should NOT be in zero, but {1}".format(msg.name[1], msg.position[1]))

if __name__ == "__main__":
    import time
    import socket

    while True: #to wait for roscore
        try:
            rospy.init_node('gazebo_test')
            break  # normal termination
        except socket.error:
            time.sleep(2)
            print "reconnecting"

    rospy.wait_for_message("/joint_states", JointState)

    import rostest

    rostest.rosrun('plato_gazebo', 'gazebo_test', TestGazebo)
