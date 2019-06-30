#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import requests
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
from plato_hololens.msg import DenseHuman, DenseHumans, DenseImagePose
# OpenCV2 for saving an image
import cv2
import base64

url = "http://52.168.125.183:5000/upload"
bridge = CvBridge()

pub_hum = rospy.Publisher('humans_poses_publisher', DenseHumans, queue_size=1)
msg_hum = DenseHumans()

def callback(data):
    print('Msg_received')
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(data.image, "bgr8")

    except CvBridgeError, e:
        print(e)
    else:
        # Save your OpenCV2 image as a jpeg 
        cv2.imwrite('people.jpeg', cv2_img)
        files = {'file': open('people.jpeg', 'rb')}
        r = requests.post(url, files=files)
        open('humanDense.png', 'wb').write(r.content)

        with open("humanDense.png", "rb") as image_file: 
            encoded_file = base64.b64encode(image_file.read())

        human = DenseHuman()
        point = Point()
        point.x=data.pose.x
        point.y = data.pose.y
        point.z=data.pose.z
        human.pose = point
        #file
        human.image_base64 = encoded_file
        header = Header()
        header.frame_id = 'holo'
        header.stamp = rospy.Time.now()
        msg_hum.header = header
        msg_hum.people.append(human)
        print('I am here! Publish Dense')

        pub_hum.publish(msg_hum)
    
def listener():
    rospy.init_node('people_uploader', anonymous=True)

    rospy.Subscriber("image_and_pose", DenseImagePose, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()