#! /usr/bin/env python

import roslib
roslib.load_manifest('my_pkg_name')
import rospy
import actionlib

# Decide if necessery and define (same than actionClient)
from autonomous_nav.msg import NavigationAction

class NavigationServer:
  def __init__(self):
    self.server = actionlib.SimpleActionServer('doNavigation', NavigationAction, self.execute, False)
    self.server.start()

  def execute(self, goal):
    # Do lots of awesome groundbreaking robot stuff here:


    self.server.set_succeeded()


if __name__ == '__main__':
  rospy.init_node('navigation_server')
  server = NavigationServer()
  rospy.spin()
