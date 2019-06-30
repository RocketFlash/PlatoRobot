#!/usr/bin/env python
# -*- coding: utf-8 -*-
# NOTE: this example requires PyAudio because it uses the Microphone class
import sys
import rospy
import time
from geometry_msgs.msg import Twist
from plato_api.constants import max_speed, wheel_distance
import speech_recognition as sr
from publish_cmd_vel import CommandPublisher
reload(sys)
sys.setdefaultencoding('utf8')


events_dict = {
    'вперёд': [1, 0, 0, 0, 0, 0, 0, 0],
    'назад': [0, 1, 0, 0, 0, 0, 0, 0],
    'налево': [0, 0, 1, 0, 0, 0, 0, 0],
    'направо': [0, 0, 0, 1, 0, 0, 0, 0],
    'стоп': [0, 0, 0, 0, 0, 0, 0, 0]
}


def start():
    global pub, command_publisher
    rospy.init_node('teleop_twist_voice')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    command_publisher = CommandPublisher(
        pub, wheel_distance, max_speed=max_speed)
    r = sr.Recognizer()

    while not rospy.is_shutdown():
        with sr.Microphone() as source:
            audio = r.listen(source)

        # recognize speech using Google Speech Recognition
        try:
            # for testing purposes, we're just using the default API key
            # to use another API key, use `r.recognize_google(audio, key="GOOGLE_SPEECH_RECOGNITION_API_KEY")`
            # instead of `r.recognize_google(audio)`
            command_string = r.recognize_google(audio, language="ru-RU")
            command_string = str(command_string.lower())
            print(command_string)
            if command_string in events_dict:
                values = events_dict[command_string]
                command_publisher.gas = (values[0] == 1)
                command_publisher.reverse = (values[1] == 1)
                command_publisher.left = (values[2] == 1)
                command_publisher.right = (values[3] == 1)
                command_publisher.increase_speed = (values[4] == 1)
                command_publisher.decrease_speed = (values[5] == 1)
                command_publisher.increase_angle_speed = (values[6] == 1)
                command_publisher.decrease_angle_speed = (values[7] == 1)
                command_publisher.publish_command()
                publish_commands(parsed_values[1])
        except sr.UnknownValueError:
            print("Could not understand your command")
        except sr.RequestError as e:
            print(
                "Could not request results from Google Speech Recognition service; {0}")