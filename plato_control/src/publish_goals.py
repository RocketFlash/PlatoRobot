#!/usr/bin/env python
import rospy
import actionlib
from geometry_msgs.msg import PoseStamped, PoseArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


def callback_goal(data):
    global publisher_goal
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()
    rospy.loginfo("Goals received!")
    for pose in data.poses:
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time().now()
        pose_stamped.header.frame_id = "map"
        pose_stamped.pose.position = pose.position
        pose_stamped.pose.orientation = pose.orientation
        pose_stamped.pose.position.z = 0
        pose_stamped.pose.orientation.x = 0
        pose_stamped.pose.orientation.y = 0
        goal = movebase_client(pose_stamped)
        client.send_goal(goal)
        wait = client.wait_for_result()
        if wait:
            rospy.loginfo("Point is reached!")
        else:
            rospy.loginfo("Point couldn't be reached!")


def movebase_client(data):

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose = data.pose
    return goal


def listener():
    rospy.loginfo("Initialization!")
    rospy.init_node('send_goals_position')
    rospy.Subscriber("plato/cartezian_path", PoseArray, callback_goal)
    rospy.spin()


if __name__ == '__main__':
    listener()
