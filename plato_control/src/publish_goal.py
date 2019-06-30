#!/usr/bin/env python
import rospy
import actionlib
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


def callback_goal(data):
    global publisher_goal
    data.header.stamp = rospy.Time().now()
    movebase_client(data)


def movebase_client(data):

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    data.pose.position.z = 0
    data.pose.orientation.x = 0
    data.pose.orientation.y = 0
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = data.pose

    client.send_goal(goal)
    wait = client.wait_for_result()
    if not wait:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()


def listener():
    rospy.init_node('send_goal_position')
    rospy.Subscriber("sended_goal", PoseStamped, callback_goal)
    rospy.spin()


if __name__ == '__main__':
    listener()
