#!/usr/bin/env python
import rospy
import actionlib
from geometry_msgs.msg import Twist
from plato_hololens.msg import BoolStamped
from std_msgs.msg import Bool

is_move = False

def callback_move(data):
    global is_move
    is_move = True


def listener():
    global is_move
    rospy.init_node('send_goal_position')
    rospy.Subscriber("cmd_vel", Twist, callback_move)
    pub = rospy.Publisher('plato/is_moving', BoolStamped, queue_size=1)
    rate = rospy.Rate(100)
    count = 0
    while not rospy.is_shutdown():
        message = BoolStamped()
        now = rospy.Time.now()
        message.header.stamp = now
        message.header.frame_id = 'map'
        bd = Bool()
        
        if is_move == False:
            count+=1
        else:
            count = 0

        if count > 100:
            bd.data = False
        else:
            bd.data = True
        is_move = False
        message.bool_val = bd
        pub.publish(message)
        rate.sleep()


if __name__ == '__main__':
    listener()
