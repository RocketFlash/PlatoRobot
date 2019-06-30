#!/usr/bin/env python
import rospy
import sys
import serial
import math
import time
from plato_api.constants import port_number, baudrate, wheel_distance, wheel_diameter, max_speed
from geometry_msgs.msg import Twist, Vector3
from open_port import open_port

ser = open_port()

bumper_pressed = False
data_previous = Vector3(0, 0, 0)


def send_command_serial(linear, angular):
    global ser
    speedLeft = linear - angular * wheel_distance / 2
    speedRight = linear + angular * wheel_distance / 2

    wheel_speed_left = int((speedLeft) * 1000)
    wheel_speed_right = int((speedRight) * 1000)
    dta = "<{},{}>".format(wheel_speed_left, wheel_speed_right)
    ser.write(dta)


def callback_bumper(data):
    global bumper_pressed, data_previous
    if not bumper_pressed and (data_previous.x != data.x or
                               data_previous.y != data.y or
                               data_previous.z != data.z):
        bumper_left = data.x
        bumper_center = data.y
        bumper_right = data.z
        print(data.x, data.y, data.z)
        if bumper_left or bumper_right or bumper_center:
            bumper_pressed = True
            distance = 0.4
            time_start = time.time()
            time_duration = distance / (max_speed / 2)
            time_end = time_start + time_duration
            print(time_end, time_duration, time_start)
            if bumper_center:
                while time.time() < time_end:
                    send_command_serial(-max_speed / 2, 0)
                    time.sleep(0.1)
            elif bumper_left:
                while time.time() < time_end:
                    send_command_serial(-max_speed / 2,
                                        ((max_speed / 4) / (wheel_distance / 2)))
                    time.sleep(0.1)
            elif bumper_right:
                while time.time() < time_end:
                    send_command_serial(-max_speed / 2, -
                                        ((max_speed / 4) / (wheel_distance / 2)))
                    time.sleep(0.1)
            send_command_serial(0, 0)
            bumper_pressed = False
            data_previous = data
    else:
        data_previous = Vector3(0, 0, 0)


def callback_cmd(data):
    global bumper_pressed
    if not bumper_pressed:
        send_command_serial(data.linear.x, data.angular.z)


def listener():
    rospy.init_node('twist_to_wheels_velocities_transform')
    rospy.Subscriber("cmd_vel", Twist, callback_cmd)
    rospy.Subscriber("data_bumper", Vector3, callback_bumper, queue_size=1)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


listener()
ser.close()
