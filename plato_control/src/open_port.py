#!/usr/bin/env python
import rospy
import serial
from plato_api.constants import port_number, baudrate
import sys


def open_port():
    port_number_working = port_number
    open_port_success = False
    try:
        ser = serial.Serial(port_number_working, baudrate, timeout=100)
	print('Port is opened {}'.format(port_number_working))
        open_port_success = True
	return ser
    except Exception:
        print('Can not open port {}'.format(port_number_working))

    if not open_port_success:
        for i in range(20):
            print('Let\'s try to open /dev/ttyUSB{} port'.format(i))
            try:
                ser = serial.Serial(
                    '/dev/ttyUSB{}'.format(i), baudrate, timeout=100)
                open_port_success = True
                port_number_working = '/dev/ttyUSB{}'.format(i)
                return ser
            except Exception:
                print('Can not open /dev/ttyUSB{}'.format(i))
        if not open_port_success:
            print('Can not open any serial port')
            rospy.loginfo("Cannot open any serial port in publish_data!")
            sys.exit(0)

if __name__ == "__main__":
    open_port()
