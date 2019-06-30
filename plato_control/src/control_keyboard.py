#!/usr/bin/env python
from pynput import keyboard
import rospy
from geometry_msgs.msg import Twist
from plato_api.constants import max_speed, wheel_distance
from publish_cmd_vel import CommandPublisher
import sys


def check(key):
    global lis
    values = [0, 0, 0, 0, 0, 0, 0, 0]
    try:
        k = key.char  # single-char keys
    except:
        k = key.name  # other keys
    if key == keyboard.Key.esc or k == 'n':
        lis.stop()
        sys.exit()
    print('Key pressed: ' + k)
    if (k == 'w') or (k == 'up'):
        values[0] = 1
    if (k == 's') or (k == 'down'):
        values[1] = 1
    if (k == 'a') or (k == 'left'):
        values[2] = 1
    if (k == 'd') or (k == 'right'):
        values[3] = 1
    if k == 'e':
        values[4] = 1
    if k == 'q':
        values[5] = 1
    if k == 'x':
        values[6] = 1
    if k == 'z':
        values[7] = 1
    return values


def on_release(key):
    global buttons_pressed
    buttons_released = [0, 0, 0, 0, 0, 0, 0, 0]
    if type(key) == list:
        print('It\'s a list released!')
        for k_ in key:
            values = check(k_)
            buttons_released = [x + y for x,
                                y in zip(buttons_released, values)]
    else:
        buttons_released = check(key)

    buttons_pressed = [0 if buttons_released[i] else buttons_pressed[i]
                       for i in range(len(buttons_pressed))]


def on_press(key):
    global buttons_pressed
    buttons_pressed_curr = [0, 0, 0, 0, 0, 0, 0, 0]
    if type(key) == list:
        print('It\'s a list pressed!')
        for k_ in key:
            values = check(k_)
            buttons_pressed_curr = [x + y for x,
                                    y in zip(buttons_pressed_curr, values)]
    else:
        buttons_pressed_curr = check(key)
    buttons_pressed = [1 if buttons_pressed_curr[i] else buttons_pressed[i]
                       for i in range(len(buttons_pressed))]


def start():
    global pub, command_publisher, buttons_pressed, lis
    lis.start()
    while not rospy.is_shutdown():
        command_publisher.gas = buttons_pressed[0]
        command_publisher.reverse = buttons_pressed[1]
        command_publisher.left = buttons_pressed[2]
        command_publisher.right = buttons_pressed[3]
        command_publisher.increase_speed = buttons_pressed[4]
        command_publisher.decrease_speed = buttons_pressed[5]
        command_publisher.increase_angle_speed = buttons_pressed[6]
        command_publisher.decrease_angle_speed = buttons_pressed[7]
        command_publisher.publish_command()
        # lis.join()


pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
buttons_pressed = [0, 0, 0, 0, 0, 0, 0, 0]
rospy.init_node('teleop_twist_keyboard')
command_publisher = CommandPublisher(
    pub, wheel_distance, max_speed=max_speed)
lis = keyboard.Listener(on_press=on_press, on_release=on_release)
start()
