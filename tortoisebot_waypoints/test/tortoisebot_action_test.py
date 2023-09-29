#! /usr/bin/env python
import rosunit
import unittest
import rostest
import rospy
import actionlib
import math
import time
from geometry_msgs.msg import Point, Quaternion
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from tortoisebot_waypoints.msg import WaypointActionFeedback, WaypointActionResult, WaypointActionAction, WaypointActionGoal

# JENKINS TEST COMMENT
PKG = 'tortoisebot_waypoints'
NAME = 'tortoisebot_waypoints_action_test'

class TestWaypointAction(unittest.TestCase):
    def setUp(self):
        rospy.init_node('action_test_node')
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.goal_position = Point()
        self.goal_position.x = -0.5
        self.goal_position.y = 0.5 
        self.current_position = Point()
        self.initial_position = Point() 
        self.current_orientation = Quaternion()
        self.first_call = True
        self.action_result = None
        self.action_client()
        print('initialized')
    def odom_callback(self, msg):
        self.current_position = msg.pose.pose.position
        if self.first_call:
            self.initial_position = self.current_position
            self.first_call = False

    def quat_to_euler(self, quat):
        orientation = [quat.x, quat.y, quat.z, quat.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation)
        return yaw
    def get_yaw(self):
        yaw = math.atan2(self.goal_position.y - self.initial_position.y, self.goal_position.x - self.initial_position.x)
        return yaw
    def action_client(self):
        print('client called')
        client = actionlib.SimpleActionClient('tortoisebot_as', WaypointActionAction)
        client.wait_for_server()
        goal = WaypointActionGoal()
        goal.position.x = self.goal_position.x
        goal.position.y = self.goal_position.y
        client.send_goal(goal)
        client.wait_for_result()
        print('result received')
        self.action_result = client.get_result()
    def test_position(self):
        x_error = abs(self.goal_position.x - self.current_position.x)
        y_error = abs(self.goal_position.y - self.current_position.y)
        position_margin = 0.5
        self.assertTrue(self.action_result)
        self.assertTrue(x_error and y_error <= position_margin)
    def test_yaw(self):
        yaw_error = abs(self.get_yaw() - self.quat_to_euler(self.current_orientation))
        yaw_margin = 5.0
        #self.assertTrue(self.action_result)
        self.assertTrue(yaw_error <= yaw_margin)

if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestWaypointAction)