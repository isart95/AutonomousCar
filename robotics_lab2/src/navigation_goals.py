#!/usr/bin/env python

import rospy

# Brings in the SimpleActionClient
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseArray
from nav_msgs.msg import Path

class MoveBaseClient():
	def __init__(self):
		rospy.init_node('move_base_client')

		self.goal = MoveBaseGoal()
		self.next_goal = MoveBaseGoal()
		self.goal_sent = False
		self.goal_cnt = 0
		self.goals = []
		self.waypoints = PoseArray()
		self.waypoints.header.frame_id = "map"

		# Creates the SimpleActionClient, passing the type of the action (MoveBaseAction) to the constructor.
		self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
		
		rospy.loginfo("Waiting for move_base action server...")
		wait = self.client.wait_for_server()
		
		if not wait:
			rospy.logerr("Action server not available!")
			rospy.signal_shutdown("Action server not available!")
			return
		rospy.loginfo("Connected to move base server")
		rospy.loginfo("Starting sending goals...")

		goal_sub = rospy.Subscriber('/simple_navigation_goal', PoseStamped, self.pose_cb)
		plan_sub = rospy.Subscriber('/move_base/GlobalPlanner/plan', Path, self.plan_cb)

		self.waypoints_pub = rospy.Publisher('/waypoint_array', PoseArray, queue_size=1)
		self.plan_pub = rospy.Publisher('/full_plan', Path, queue_size=1)

		self.full_plan = Path()
		self.full_plan.header.frame_id = "map"

		self.plan_pub.publish(self.full_plan)


	def pose_cb(self, pose):
		#rospy.loginfo("\n" + str(pose))
		if (pose not in self.goals) and (len(self.goals) < 4):
			pose.header.frame_id = "map"
			pose.header.stamp = rospy.Time.now()
			self.goals.append(pose)
			self.waypoints.poses.append(pose.pose)
		
		if (len(self.goals) == 4):
			self.waypoints.header.stamp = rospy.Time.now()
			self.waypoints_pub.publish(self.waypoints)
			self.waypoints.poses[:] = []

			self.send_first_goal()

	def active_cb(self):
		rospy.loginfo("Waypoint " + str(self.goal_cnt+1) + " is now being processed by the Action Server...")

	def feedback_cb(self, feedback):
		rospy.loginfo("Feedback for Waypoint " + str(self.goal_cnt + 1) + " received")

	def plan_cb(self, plan):
		if (self.goal_sent == True):
			self.full_plan.header.stamp = rospy.Time.now()
			self.full_plan.poses.extend(plan.poses)
			self.plan_pub.publish(self.full_plan)
			self.goal_sent = False


	def done_cb(self, status, result):
		self.goal_cnt += 1
	# Reference for terminal status values: http://docs.ros.org/diamondback/api/actionlib_msgs/html/msg/GoalStatus.html
		if status == 2:
			rospy.loginfo("Waypoint " + str(self.goal_cnt) + " received a cancel request after it started executing, completed execution!")

		if status == 3:
			rospy.loginfo("Waypoint " + str(self.goal_cnt) + " reached")
			rospy.sleep(2.0)
			if (self.goal_cnt < len(self.goals)):
				self.next_goal.target_pose = self.goals[self.goal_cnt]
				rospy.loginfo("Sending Waypoint " + str(self.goal_cnt + 1) + " to Action Server")
				#rospy.loginfo("In done_cb:\n"+ str(self.goals[self.goal_cnt]))
				self.client.send_goal(self.next_goal, self.done_cb, self.active_cb, self.feedback_cb)
				rospy.sleep(2)
				self.goal_sent = True
				

			else:
				rospy.loginfo("Final Waypoint reached!")
				rospy.signal_shutdown("Final Waypoint reached!")
				return

		if status == 4:
			rospy.loginfo("Waypoint " + str(self.goal_cnt) + " was aborted by the Action Server")
			rospy.sleep(2.0)
			if (self.goal_cnt < len(self.goals)):
				self.next_goal.target_pose = self.goals[self.goal_cnt]
				rospy.loginfo("Sending Waypoint " + str(self.goal_cnt + 1) + " to Action Server")
				#rospy.loginfo("In done_cb:\n"+ str(self.goals[self.goal_cnt]))
				self.client.send_goal(self.next_goal, self.done_cb, self.active_cb, self.feedback_cb)
				rospy.sleep(2)
				self.goal_sent = True
			#rospy.signal_shutdown("Waypoint " + str(self.goal_cnt) + " aborted, shutting down!")
			#return

		if status == 5:
			rospy.loginfo("Waypoint " + str(self.goal_cnt) + " has been rejected by the Action Server")
			#rospy.signal_shutdown("Waypoint " + str(self.goal_cnt) + " rejected, shutting down!")
			#return

		if status == 8:
			rospy.loginfo("Waypoint " + str(self.goal_cnt) + " received a cancel request before it started executing, successfully cancelled!")

	def send_first_goal(self):
		self.goal.target_pose = self.goals[self.goal_cnt]
		#rospy.loginfo("In send_first_goal:\n"+ str(self.goals[self.goal_cnt]))
		rospy.loginfo("Goal " + str(self.goal_cnt + 1) + " sent!")
		self.client.send_goal(self.goal, self.done_cb, self.active_cb, self.feedback_cb)
		self.goal_sent = True
		
		wait = self.client.wait_for_result()
		
		if not wait:
			rospy.logerr("Action server not available!")
			rospy.signal_shutdown("Action server not available!")
		

if __name__ == '__main__':
	try:
		MoveBaseClient()
		rospy.spin()
	except rospy.ROSInterruptException:
		rospy.loginfo("Navigation finished.")
		#rospy.is_shutdown()