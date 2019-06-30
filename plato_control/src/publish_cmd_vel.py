from geometry_msgs.msg import Twist


class CommandPublisher(object):
    """ Class for robot commands publishing:

    Attributes:
        publisher: ROS publisher
        wheel_distance: distance between wheels centers
        max_speed: maximally achievable linear speed
        indicator_linear: speed index as a percentage of maximum linear speed
                          linear_speed = (indicator_linear / 100) * max_speed
        indicator_angular: speed index as a percentage of maximum angular speed
                          angular_speed = (indicator_angular) / 100 * \
                                (max_speed_linear / (wheel_distance / 2)
    """

    def __init__(self, publisher, wheel_distance=0.27, max_speed=0.8,
                 indicator_linear=50, indicator_angular=50):
        self.publisher = publisher
        self.direction_linear = 0
        self.direction_angular = 0
        self.count = 0
        self.indicator_linear = indicator_linear
        self.indicator_angular = indicator_angular
        self.max_speed_linear = max_speed
        self.gas = False
        self.reverse = False
        self.left = False
        self.right = False
        self.increase_speed = False
        self.decrease_speed = False
        self.increase_angle_speed = False
        self.decrease_angle_speed = False
        self.speed_linear = 0
        self.speed_angular = 0
        self.wheel_distance = wheel_distance
        self.control_speed = 0
        self.control_turn = 0

    def publish_command(self):
        """ Function updates commands states and publish ROS Twist message
        """
        self.update()
        self.publisher.publish(self.get_twist())

    def get_twist(self):
        """ Function retuns ROS Twist message
        """
        twist = Twist()
        twist.linear.x = self.control_speed
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = self.control_turn
        return twist

    def set_indicator_linear(self, value):
        """ Function set new linear indicator value

        :param value: new indicator value
        :type value: int

        """
        self.indicator_linear = value

    def set_indicator_angular(self, value):
        """ Function set new angular indicator value

        :param value: new indicator value
        :type value: int

        """
        self.indicator_angular = value

    def update(self):
        """ Function updates commands states and calculate control linear
        and angular speeds
        """
        if self.gas or self.reverse or self.left or self.right:
            if self.gas:
                self.direction_linear = 1
            elif self.reverse:
                self.direction_linear = -1
            else:
                self.direction_linear = 0

            if self.left:
                self.direction_angular = 1
            elif self.right:
                self.direction_angular = -1
            else:
                self.direction_angular = 0

            self.count = 0
        else:
            self.count += 1
            if self.count >= 2:
                self.direction_linear = 0
                self.direction_angular = 0

        if self.increase_speed and self.indicator_linear < 100:
            self.indicator_linear += 2
        if self.decrease_speed and self.indicator_linear > 0:
            self.indicator_linear -= 2

        if self.increase_angle_speed and self.indicator_angular < 100:
            self.indicator_angular += 2
        if self.decrease_angle_speed and self.indicator_angular > 0:
            self.indicator_angular -= 2

        self.speed_linear = float(
            self.indicator_linear) / 100 * self.max_speed_linear
        self.speed_angular = float(
            self.indicator_angular) / 100 * (self.max_speed_linear / (self.wheel_distance / 2))
        target_speed = self.speed_linear * self.direction_linear
        target_turn = self.speed_angular * self.direction_angular

        if target_speed > self.control_speed:
            self.control_speed = min(target_speed, self.control_speed + 0.05)
        elif target_speed < self.control_speed:
            self.control_speed = max(target_speed, self.control_speed - 0.05)
        else:
            self.control_speed = target_speed

        if target_turn > self.control_turn:
            self.control_turn = min(target_turn, self.control_turn + 0.5)
        elif target_turn < self.control_turn:
            self.control_turn = max(target_turn, self.control_turn - 0.5)
        else:
            self.control_turn = target_turn
