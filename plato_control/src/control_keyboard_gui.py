#!/usr/bin/env python
import pygame
import rospy
from geometry_msgs.msg import Twist
from plato_api.constants import max_speed, wheel_distance
from gui_utils import KeyboardGui
from publish_cmd_vel import CommandPublisher


def start():
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.init_node('teleop_twist_keyboard')
    gui = KeyboardGui()
    command_publisher = CommandPublisher(
        pub, wheel_distance, max_speed=max_speed)
    while not rospy.is_shutdown():
        gui.update()
        buttons_pressed, mouse_pressed, slider_values = gui.get_pressed_keys()
        command_publisher.gas = (buttons_pressed[0] == 1) or (
            mouse_pressed[0] == 1)
        command_publisher.reverse = (buttons_pressed[1] == 1) or (
            mouse_pressed[1] == 1)
        command_publisher.left = (buttons_pressed[2] == 1) or (
            mouse_pressed[2] == 1)
        command_publisher.right = (buttons_pressed[3] == 1) or (
            mouse_pressed[3] == 1)
        command_publisher.set_indicator_linear(slider_values[0])
        command_publisher.set_indicator_angular(slider_values[1])
        command_publisher.publish_command()
    gui.quit_gui()


if __name__ == '__main__':
    start()
