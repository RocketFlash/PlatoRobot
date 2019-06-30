import unittest
from plato_control.src.publish_data import *
from geometry_msgs.msg import Vector3


class TestParseDataSonar(unittest.TestCase):
    def test_parse_data_sonar_success(self):
        self.assertEqual((True, Vector3(3, 56, 0)), parse_data_sonar("3,56,0"))
        self.assertEqual((True, Vector3(3, 56, 0)), parse_data_sonar("3,56,0"))
        self.assertEqual((True, Vector3(-3, 56, 0)),
                         parse_data_sonar("-3,56,-0"))
        self.assertEqual((True, Vector3(-3, -56, 0)),
                         parse_data_sonar("-3,-56,-0"))
        self.assertEqual((True, Vector3(3, -56, 0)),
                         parse_data_sonar("3,-56,0"))

    def test_parse_data_sonar_fail(self):
        self.assertEqual((False,), parse_data_sonar("-3.0,56,-0"))
        self.assertEqual((False,), parse_data_sonar("3,560"))
        self.assertEqual((False,), parse_data_sonar("560"))
        self.assertEqual((False,), parse_data_sonar("560,"))
        self.assertEqual((False,), parse_data_sonar("3,,5"))
        self.assertEqual((False,), parse_data_sonar("3, ,5"))
        self.assertEqual((False,), parse_data_sonar("3,5,3,3"))
        self.assertEqual((False,), parse_data_sonar("S-3,56,-0]S"))
        self.assertEqual((False,), parse_data_sonar("S-3,5F6,-0]S"))
        self.assertEqual((False,), parse_data_sonar("3,56,5,6-0"))
        self.assertEqual((False,), parse_data_sonar("3,56,44,-0"))


class TestParseDataBumper(unittest.TestCase):
    def test_parse_data_bumper_success(self):
        self.assertEqual((True, Vector3(3, 56, 0)),
                         parse_data_bumper("3,56,0"))
        self.assertEqual((True, Vector3(3, 56, 0)),
                         parse_data_bumper("3,56,0"))
        self.assertEqual((True, Vector3(-3, 56, 0)),
                         parse_data_bumper("-3,56,-0"))
        self.assertEqual((True, Vector3(-3, -56, 0)),
                         parse_data_bumper("-3,-56,-0"))
        self.assertEqual((True, Vector3(3, -56, 0)),
                         parse_data_bumper("3,-56,0"))

    def test_parse_data_bumper_fail(self):
        self.assertEqual((False,), parse_data_bumper("-3.0,56,-0"))
        self.assertEqual((False,), parse_data_bumper("3,560"))
        self.assertEqual((False,), parse_data_bumper("560"))
        self.assertEqual((False,), parse_data_bumper("560,"))
        self.assertEqual((False,), parse_data_bumper("3,,5"))
        self.assertEqual((False,), parse_data_bumper("3, ,5"))
        self.assertEqual((False,), parse_data_bumper("3,5,3,3"))
        self.assertEqual((False,), parse_data_bumper("S-3,56,-0]S"))
        self.assertEqual((False,), parse_data_bumper("S-3,5F6,-0]S"))
        self.assertEqual((False,), parse_data_bumper("3,56,5,6-0"))
        self.assertEqual((False,), parse_data_bumper("3,56,44,-0"))


class TestParseDataIMU(unittest.TestCase):
    def test_parse_data_imu_success(self):
        self.assertEqual((True, Vector3(1, 2, 3), Vector3(
            4, 5, 6), Quaternion(7.0, 8.0, 9.0, 10.0)), parse_data_imu("1,2,3,4,5,6,7,8,9,10"))
        self.assertEqual((True, Vector3(1, 2, 3), Vector3(
            4, 5, 6), Quaternion(7.0, 8.0, 9.0, 10.0)), parse_data_imu("1.0,2.0,3.0,4.0,5.0,6.0,7.0,8.0,9.0,10.0"))
        self.assertEqual((True, Vector3(-1, -2, -3), Vector3(
            -4, -5, -6), Quaternion(-7.0, -8.0, -9.0, -10.0)), parse_data_imu("-1,-2.0,-3.0,-4,-5,-6,-7,-8,-9,-10"))
        self.assertEqual((True, Vector3(-1, -2, -3), Vector3(
            -4, -5, 6), Quaternion(7.0, 8.0, 9.0, 10.0)), parse_data_imu("-1,-2.0,-3.0,-4,-5,6.0,7,8.0,9,10.0"))
        self.assertEqual((True, Vector3(1, 2, 3), Vector3(
            4, 5, 6), Quaternion(7.0, 8.0, 9.0, 10.0)), parse_data_imu("1.000000,2.000,3.00,4.0000000000000000000,5,6,7,8,9,10"))

    def test_parse_data_imu_fail(self):
        self.assertEqual((False,), parse_data_imu("3,560"))
        self.assertEqual((False,), parse_data_imu("560"))
        self.assertEqual((False,), parse_data_imu("560,"))
        self.assertEqual((False,), parse_data_imu("3,,5"))
        self.assertEqual((False,), parse_data_imu("3, ,5"))
        self.assertEqual((False,), parse_data_imu("3,5,3,3"))
        self.assertEqual((False,), parse_data_imu("S-3,56,-0]S"))
        self.assertEqual((False,), parse_data_imu("S-3,5F6,-0]S"))
        self.assertEqual((False,), parse_data_imu("3,56,5,6-0"))
        self.assertEqual((False,), parse_data_imu("3,56,44,-0"))
        self.assertEqual((False,), parse_data_imu("1,2,3, ,5,6,7,8,9,10"))
        self.assertEqual((False,), parse_data_imu("1,2,3,4,5H,6,7,8,9,10"))
        self.assertEqual((False,), parse_data_imu("1,2, , ,5,6,7,8,9,10"))
        self.assertEqual((False,), parse_data_imu("1,--2,3,--4,5,6,7,8,9,10"))
        self.assertEqual((False,), parse_data_imu("1,2,3,4K5,5,6,7,8,9,10"))
        self.assertEqual((False,), parse_data_imu("1,2,3,4,5,6,7,8,9,"))
        self.assertEqual((False,), parse_data_imu("1,2,3,4,5,6,7,8,9"))
        self.assertEqual((False,), parse_data_imu("1,2,3,4,5,6,7,8,9,10,11"))
        self.assertEqual((False,), parse_data_imu(
            "1,8.000000K,2,3,4,5,6,7,8,9"))


class TestParseDataWheels(unittest.TestCase):
    def test_parse_data_wheels_success(self):
        test_twist = Twist()
        test_twist.linear.x = 0.0
        test_twist.angular.z = 0.0
        self.assertEqual((True, test_twist, 0, 0), parse_data_wheels("0, 0"))
        test_twist.angular.z = -0.74074074074074074074074
        self.assertEqual((True, test_twist, 100, 100),
                         parse_data_wheels("100,100"))
        test_twist.angular.z = 0.74074074074074074074074
        self.assertEqual((True, test_twist, -100, -100),
                         parse_data_wheels("-100,-100"))
        test_twist.linear.x = 0.1
        test_twist.angular.z = 0.0
        self.assertEqual((True, test_twist, 100, -100),
                         parse_data_wheels("100,-100"))

    def test_parse_data_wheels_fail(self):
        self.assertEqual((False,), parse_data_wheels("-3.0,56,0"))
        self.assertEqual((False,), parse_data_wheels("3,5,4,3,2"))
        self.assertEqual((False,), parse_data_wheels("3s,5s,3,3"))
        self.assertEqual((False,), parse_data_sonar("S-3,56,-0]S"))
        self.assertEqual((False,), parse_data_wheels("--56,-0"))


if __name__ == '__main__':
    unittest.main()
