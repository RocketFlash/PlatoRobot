#!/usr/bin/env python
import rospy
from plato_hololens.msg import Humans, Human, HumanBodyPart, MyObjects, MyObject
from std_msgs.msg import Header
from geometry_msgs.msg import Point


def talker():
    pub_hum = rospy.Publisher('humans_poses_publisher', Humans)
    pub_obj = rospy.Publisher('objects_publisher', Humans)
    rospy.init_node('pose_talker', anonymous=True)
    classes = ['cat','dog','car','person','table','chair','window','door','tv','keyboard','bottle']
    r = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():
        msg_hum = Humans()
        msg_obj = MyObjects()
        objects = []
        humans = []
        # Create objects
        for k in range(10):
            obj = MyObject()
            obj.class_id = k
            point = Point()
            point.x = k*10
            point.y = 0
            point.z = 0
            obj.pose = point
            objects.append(obj)
        header = Header()
        header.frame_id = 'holo'
        header.stamp = rospy.Time.now()
        msg_obj.header = header
        rospy.loginfo(msg_obj)
        pub_obj.publish(msg_obj)

        # Create humans
        for j in range(2):
            parts = []
            human = Human()
            point = Point()
            point.x = j*10
            point.y = 0
            point.z = 0
            for i in range(25):
                part = HumanBodyPart()
                part.x = j+int(i*0.2)
                part.y = j+int(i*0.2)
                part.idx = i
                parts.append(part)
            human.body_parts = parts
            human.pose = point
            humans.append(human)
        msg_hum.people = humans
        header = Header()
        header.frame_id = 'holo'
        header.stamp = rospy.Time.now()
        msg_hum.header = header
        rospy.loginfo(msg_hum)
        pub_hum.publish(msg_hum)
        r.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
