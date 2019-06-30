#!/usr/bin/env python
import rospy
import actionlib
import math
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal



class MoveBaseController():

    def __init__(self,success_to_move = 2,epsilon = 1, coord_tolerance = 1):
        self.x_goal_curr = 10000
        self.y_goal_curr = 10000
        self.x_prev = 0
        self.y_prev = 0
        self.num_success = 0
        self.success_to_move = success_to_move
        self.epsilon = epsilon
        self.coord_tolerance = coord_tolerance
        rospy.init_node('send_goal_position')
        rospy.Subscriber("plato/follower/goal", PoseStamped, self.callback_goal)


    def check_if_inside_circle(self,point_to_check, point_circle, radius = None):
        '''
        Function checks if point inside a circle or not
        Input:
        point_to_check: tuple (x, y) of point to check
        point_circle: tuple (x, y) of circle center
        Returns:
        ans: result of checking (if True point is inside a circle,
                                else is not)
        '''
        x_curr, y_curr = point_to_check[0], point_to_check[1]
        x_circle, y_circle = point_circle[0], point_circle[1]
        ans = math.sqrt((x_curr - x_circle)**2 + (y_curr - y_circle)**2)
        if radius is None:
            radius = self.epsilon  
        return ans < radius


    def callback_goal(self,data):
        point_to_check = (data.pose.position.x, data.pose.position.y)
        point_circle = (self.x_goal_curr, self.y_goal_curr)
        point_prev = (self.x_prev, self.y_prev)
        self.x_prev, self.y_prev = data.pose.position.x, data.pose.position.y
        if self.num_success >= self.success_to_move:
            self.x_goal_curr, self.y_goal_curr = data.pose.position.x, data.pose.position.y
            print('Current goal is: x:{} y:{}'.format(self.x_goal_curr, self.y_goal_curr))
            self.movebase_client(data)
            self.num_success = 0
        else:
            if not self.check_if_inside_circle(point_to_check, point_circle):
                if self.check_if_inside_circle(point_to_check, point_prev, self.coord_tolerance):
                    self.num_success+=1;
                else:
                    self.num_success = 0
            else:
                self.num_success = 0


    def movebase_client(self,data):

        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        wait = client.wait_for_server()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        data.pose.position.z = 0
        data.pose.orientation.x = 0
        data.pose.orientation.y = 0
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time(0)
        goal.target_pose.pose = data.pose

        client.send_goal(goal)
        # wait = client.wait_for_result()
        # if not wait:
        #     rospy.logerr("Action server not available!")
        #     rospy.signal_shutdown("Action server not available!")
        # else:
        #     return client.get_result()



if __name__ == '__main__':
    MoveBaseController(success_to_move = 2,epsilon = 1, coord_tolerance = 1)
    rospy.spin()
