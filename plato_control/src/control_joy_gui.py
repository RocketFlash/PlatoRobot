#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from plato_api.constants import max_speed, wheel_distance
from publish_cmd_vel import CommandPublisher
from gui_utils import JoystickGui


def start():

    rospy.init_node('control_joy')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    gui = JoystickGui()
    command_publisher = CommandPublisher(
        pub, wheel_distance, max_speed=max_speed)

    while not rospy.is_shutdown():
        gui.update()
        buttons_pressed, axes_pressed, hats_pressed = gui.get_pressed_keys()
        command_publisher.gas = (buttons_pressed[1] == 1) or (
            axes_pressed[1] < 0)
        command_publisher.reverse = (buttons_pressed[0] == 1) or (
            axes_pressed[1] > 0)
        command_publisher.left = (
            hats_pressed[0][0] == -1) or (axes_pressed[0] < 0)
        command_publisher.right = (
            hats_pressed[0][0] == 1) or (axes_pressed[0] > 0)
        command_publisher.increase_speed = (buttons_pressed[7] == 1)
        command_publisher.decrease_speed = (buttons_pressed[6] == 1)
        command_publisher.increase_angle_speed = (buttons_pressed[5] == 1)
        command_publisher.decrease_angle_speed = (buttons_pressed[4] == 1)
        command_publisher.publish_command()
    gui.quit_gui()


if __name__ == '__main__':
    start()
