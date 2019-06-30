#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from plato_api.constants import max_speed, wheel_distance
from publish_cmd_vel import CommandPublisher


def update_values(data):
    axes = data.axes
    buttons = data.buttons
    command_publisher.gas = (buttons[1] == 1)
    command_publisher.reverse = (buttons[0] == 1)
    command_publisher.left = (axes[4] > 0)
    command_publisher.right = (axes[4] < 0)
    command_publisher.increase_speed = (buttons[7] == 1)
    command_publisher.decrease_speed = (buttons[6] == 1)
    command_publisher.increase_angle_speed = (buttons[5] == 1)
    command_publisher.decrease_angle_speed = (buttons[4] == 1)
    command_publisher.publish_command()


def start():
    global pub, command_publisher
    rospy.init_node('control_joy')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    command_publisher = CommandPublisher(
        pub, wheel_distance, max_speed=max_speed)
    rospy.Subscriber("joy", Joy, update_values)
    rospy.spin()


if __name__ == '__main__':
    start()
