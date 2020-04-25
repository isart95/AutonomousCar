#! /usr/bin/env python

import roslib
roslib.load_manifest('autonomous_car')
import rospy
import actionlib

# Decide if necessery and define
from autonomous_nav.msg import NavigationAction, NavigationGoal

if __name__ == '__main__':
    rospy.init_node('navigation_client')
    client = actionlib.SimpleActionClient('doNavigation', NavigationAction)
    client.wait_for_server()

    goal = NavigationGoal()

    # Fill in the goal here (http://wiki.ros.org/actionlib)
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(5.0))
