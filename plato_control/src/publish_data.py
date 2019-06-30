#!/usr/bin/env python
import rospy
import serial
import sys
from plato_api.constants import port_number, baudrate, wheel_distance
from std_msgs.msg import Int32, Header
from geometry_msgs.msg import Twist, Vector3, Quaternion
from sensor_msgs.msg import Imu
from open_port import open_port


def represents_float(s):
    try:
        float(s)
        return True
    except ValueError:
        return False


def represents_int(s):
    try:
        int(s)
        return True
    except ValueError:
        return False


def parse_data_bumper(received_line):
    values = received_line.split(',')
    data_bumper = Vector3()
    if len(values) == 3:
        bumper_left, bumper_middle, bumper_right = values[0], values[1], values[2]
        if represents_int(bumper_left) and represents_int(bumper_middle) \
                and represents_int(bumper_right):
            data_bumper.x = int(bumper_left)
            data_bumper.y = int(bumper_middle)
            data_bumper.z = int(bumper_right)
        else:
            print("SONAR: Cannot cast data to int")
            print("String: {}".format(received_line))
            return (False,)
    else:
        print("SONAR: Number of parsed values isn't equal to 3")
        print("String: {}".format(received_line))
        return (False,)
    return True, data_bumper


def parse_data_sonar(received_line):
    values = received_line.split(',')
    data_sonar = Vector3()
    if len(values) == 3:
        sonar_left, sonar_middle, sonar_right = values[0], values[1], values[2]
        if represents_int(sonar_left) and represents_int(sonar_middle) \
                and represents_int(sonar_right):
            data_sonar.x = int(sonar_left)
            data_sonar.y = int(sonar_middle)
            data_sonar.z = int(sonar_right)
        else:
            print("SONAR: Cannot cast data to int")
            print("String: {}".format(received_line))
            return (False,)
    else:
        print("SONAR: Number of parsed values isn't equal to 3")
        print("String: {}".format(received_line))
        return (False,)
    return True, data_sonar


def parse_data_imu(received_line):
    values = received_line.split(',')
    data_imu_angular_speed = Vector3()
    data_imu_acceleration = Vector3()
    data_imu_quaternion = Quaternion()
    if len(values) == 10:
        ang_x, ang_y, ang_z = values[0], values[1], values[2]
        acc_x, acc_y, acc_z = values[3], values[4], values[5]
        q_x, q_y, q_z, q_w = values[6], values[7], values[8], values[9]
        if represents_float(ang_x) and represents_float(ang_y) \
                and represents_float(ang_z) and represents_float(acc_x) \
                and represents_float(acc_y) and represents_float(acc_z) \
                and represents_float(q_x) and represents_float(q_y) \
                and represents_float(q_z) and represents_float(q_w):
            data_imu_angular_speed.x = float(ang_x)
            data_imu_angular_speed.y = float(ang_y)
            data_imu_angular_speed.z = float(ang_z)
            data_imu_acceleration.x = float(acc_x)
            data_imu_acceleration.y = float(acc_y)
            data_imu_acceleration.z = float(acc_z)
            data_imu_quaternion.x = float(q_x)
            data_imu_quaternion.y = float(q_y)
            data_imu_quaternion.z = float(q_z)
            data_imu_quaternion.w = float(q_w)
            return True, data_imu_angular_speed, data_imu_acceleration, data_imu_quaternion
        else:
            print("IMU: Cannot cast data to float")
            print("String: {}".format(received_line))
            return (False,)
    else:
        print("IMU: Number of parsed values isn't equal to 10")
        print("String: {}".format(received_line))
        return (False,)


def parse_data_wheels(received_line):
    left_value = 0
    right_value = 0
    twist_msg = Twist()
    values = received_line.split(',')
    if len(values) == 2 and represents_int(values[0]) and represents_int(values[1]):
        left_value = int(values[0])
        right_value = int(values[1])
        # linear and angular velocities calculated using wheel velocities
        # and convert them (mmps to meters per second)
        speed_linear = (float(left_value - right_value) / 2) / 1000
        speed_angular = -(float(right_value + left_value) /
                          (wheel_distance)) / 1000
        twist_msg.linear.x = speed_linear
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = 0.0
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = speed_angular
        return True, twist_msg, left_value, right_value
    else:
        print("Something wrong with wheels data")
        print("String: {}".format(received_line))
        return (False, )


def talker():
    orientation_covariance = [0, 0.0, 0.0,
                              0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0]
    angular_velocity_covariance = [0, 0.0, 0.0,
                                   0.0, 0.0, 0.0,
                                   0.0, 0.0, 0.0]
    linear_acceleration_covariance = [0.0, 0.0, 0.0,
                                      0.0, 0.0, 0.0,
                                      0.0, 0.0, 0.0]
    publisher_left = rospy.Publisher('wheel_speed_left', Int32, queue_size=1)
    publisher_right = rospy.Publisher('wheel_speed_right', Int32, queue_size=1)
    publisher_twist = rospy.Publisher('wheel_speed_twist', Twist, queue_size=1)
    publisher_sonar = rospy.Publisher('data_sonar', Vector3, queue_size=1)
    publisher_bumper = rospy.Publisher('data_bumper', Vector3, queue_size=1)
    publisher_imu = rospy.Publisher('data_imu', Imu, queue_size=1)
    # publisher_imu_angular_velocities = rospy.Publisher(
    #     'data_imu_angular_velocities', Vector3, queue_size=1)
    # publisher_imu_accelerations = rospy.Publisher(
    #     'data_imu_accelerations', Vector3, queue_size=1)
    # publisher_imu_quaternion = rospy.Publisher(
    #     'data_imu_quaternion', Quaternion, queue_size=1)

    rospy.init_node('publish_robot_data')

    ser = open_port()
    print(ser)
    with ser:
        while not rospy.is_shutdown():
            line = ser.readline().rstrip()
            if len(line) > 4:
                if line[0] == 'W' and line[1] == '['\
                        and line[len(line) - 1] == 'W'\
                        and line[len(line) - 2] == ']':
                    parsed_values = parse_data_wheels(line[2:len(line) - 2])
                    if parsed_values[0]:
                        publisher_left.publish(parsed_values[2])
                        publisher_right.publish(parsed_values[3])
                        publisher_twist.publish(parsed_values[1])
                elif line[0] == 'S' and line[1] == '['\
                        and line[len(line) - 1] == 'S'\
                        and line[len(line) - 2] == ']':
                    parsed_values = parse_data_sonar(line[2:len(line) - 2])
                    if parsed_values[0]:
                        publisher_sonar.publish(parsed_values[1])
                elif line[0] == 'B' and line[1] == '['\
                        and line[len(line) - 1] == 'B'\
                        and line[len(line) - 2] == ']':
                    parsed_values = parse_data_bumper(line[2:len(line) - 2])
                    if parsed_values[0]:
                        publisher_bumper.publish(parsed_values[1])
                elif line[0] == 'I' and line[1] == '['\
                        and line[len(line) - 1] == 'I'\
                        and line[len(line) - 2] == ']':
                    parsed_values = parse_data_imu(line[2:len(line) - 2])
                    if parsed_values[0]:
                        imu_angular_velocities = parsed_values[1]
                        imu_accelerations = parsed_values[2]
                        imu_quaternion = parsed_values[3]
                        imu_message = Imu()
                        now = rospy.Time.now()
                        imu_message.header.stamp = now
                        imu_message.header.frame_id = 'imu'
                        imu_message.angular_velocity = imu_angular_velocities
                        imu_message.linear_acceleration = imu_accelerations
                        imu_message.orientation = imu_quaternion
                        imu_message.linear_acceleration_covariance = linear_acceleration_covariance
                        imu_message.angular_velocity_covariance = angular_velocity_covariance
                        imu_message.orientation_covariance = orientation_covariance
                        publisher_imu.publish(imu_message)
                        # publisher_imu_angular_velocities.publish(
                        #     parsed_values[1])
                        # publisher_imu_accelerations.publish(parsed_values[2])
                        # publisher_imu_quaternion.publish(parsed_values[3])
                else:
                    print("Cannot parse data from the string")
            else:
                print("Received string is empty")


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
