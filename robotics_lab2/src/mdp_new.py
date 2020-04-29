#!/usr/bin/env python

import rospy
import numpy as np
import random
import mdptoolbox

import actionlib

from tf import transformations

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped, PoseArray
from nav_msgs.msg import Path

from nav_msgs.msg import OccupancyGrid

class MoveBaseClient():
	def __init__(self):
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

		#goal_sub = rospy.Subscriber('/simple_navigation_goal', PoseStamped, self.pose_cb)
		plan_sub = rospy.Subscriber('/move_base/GlobalPlanner/plan', Path, self.plan_cb)

		self.goals_pub = rospy.Publisher('/goal_array', PoseArray, queue_size=1)
		self.plan_pub = rospy.Publisher('/full_plan', Path, queue_size=1)
		self.waypoints_pub = rospy.Publisher('/waypoint_array', PoseArray, queue_size=1)

		self.full_plan = Path()
		self.full_plan.header.frame_id = "map"

		self.plan_pub.publish(self.full_plan)

	def give_goal_cell(self, pose, num_poses):
		#rospy.loginfo("\n" + str(pose))
		if (pose not in self.goals) and (len(self.goals) < num_poses):
			self.goals.append(pose)
			self.waypoints.poses.append(pose.pose)
		
		if (len(self.goals) == num_poses):
			self.waypoints.header.stamp = rospy.Time.now()
			self.waypoints_pub.publish(self.waypoints)
			self.waypoints.poses[:] = []

	def active_cb(self):
		rospy.loginfo("Goal " + str(self.goal_cnt+1) + " is now being processed by the Action Server...")

	def feedback_cb(self, feedback):
		rospy.loginfo("Feedback for Goal " + str(self.goal_cnt + 1) + " received")

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
			rospy.loginfo("Goal " + str(self.goal_cnt) + " received a cancel request after it started executing, completed execution!")

		if status == 3:
			rospy.loginfo("Goal " + str(self.goal_cnt) + " reached")
			rospy.sleep(2.0)
			if (self.goal_cnt < len(self.goals)):
				self.next_goal.target_pose = self.goals[self.goal_cnt]
				#rospy.loginfo("Sending Goal " + str(self.goal_cnt + 1) + " to Action Server")
				#rospy.loginfo("In done_cb:\n"+ str(self.goals[self.goal_cnt]))
				self.client.send_goal(self.next_goal, self.done_cb, self.active_cb, self.feedback_cb)
				rospy.sleep(2)
				self.goal_sent = True
				

			else:
				rospy.loginfo("Final Goal reached!")
				rospy.signal_shutdown("Final Goal reached!")
				return

		if status == 4:
			rospy.loginfo("Goal " + str(self.goal_cnt) + " was aborted by the Action Server")
			rospy.sleep(2.0)
			if (self.goal_cnt < len(self.goals)):
				self.next_goal.target_pose = self.goals[self.goal_cnt]
				#rospy.loginfo("Sending Goal " + str(self.goal_cnt + 1) + " to Action Server")
				#rospy.loginfo("In done_cb:\n"+ str(self.goals[self.goal_cnt]))
				self.client.send_goal(self.next_goal, self.done_cb, self.active_cb, self.feedback_cb)
				rospy.sleep(2)
				self.goal_sent = True
			#rospy.signal_shutdown("Goal " + str(self.goal_cnt) + " aborted, shutting down!")
			#return

		if status == 5:
			rospy.loginfo("Goal " + str(self.goal_cnt) + " has been rejected by the Action Server")
			#rospy.signal_shutdown("Goal " + str(self.goal_cnt) + " rejected, shutting down!")
			#return

		if status == 8:
			rospy.loginfo("Goal " + str(self.goal_cnt) + " received a cancel request before it started executing, successfully cancelled!")

	def send_first_goal(self):
		self.goal.target_pose = self.goals[self.goal_cnt]
		#rospy.loginfo("In send_first_goal:\n"+ str(self.goals[self.goal_cnt]))
		#rospy.loginfo("Goal " + str(self.goal_cnt + 1) + " sent!")
		self.client.send_goal(self.goal, self.done_cb, self.active_cb, self.feedback_cb)
		self.goal_sent = True
		
		wait = self.client.wait_for_result()
		
		if not wait:
			rospy.logerr("Action server not available!")
			rospy.signal_shutdown("Action server not available!")

class pre_mdp():
	def __init__(self):
		rospy.init_node('pre_mdp')
		self.map_callback = False
		map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_cb)
		
		self.freeStates = []
		self.occupiedStates = []

	def map_cb(self, disc_map):
		map_height = disc_map.info.height
		map_width = disc_map.info.width
		map_resolution = disc_map.info.resolution
		map_data = disc_map.data
		for i in range(map_height):
			for j in range(map_width):
				state_number = (i * map_width) + j
				index_matrix = [i, j]
				if (map_data[state_number] == 0):
					self.freeStates.append(index_matrix)
					#rospy.loginfo("\nfreeStates\n"+str(self.freeStates))
				if (map_data[state_number] == 100):
					self.occupiedStates.append(index_matrix)
		self.map_height = map_height
		self.map_width = map_width
		self.map_resolution = map_resolution
		self.map_data = map_data
		self.map_callback = True

	def state_to_cell_center_coordinate(self, state):
		grid_size = self.map_resolution
		xsize = self.map_height*grid_size
		ysize = self.map_width*grid_size
		
		state_x = state[0]
		state_y = state[1]

		#rospy.loginfo("state_x: " + str(state_x))
		#rospy.loginfo("state_y: " + str(state_y))

		real_x = -(xsize/2.0) + (grid_size/2.0 + (grid_size * (state_y-1)))
		real_y = -(ysize/2.0) + (grid_size/2.0 + (grid_size * (state_x+1)))
		
		
		#rospy.loginfo("Resolution: " + str(grid_size) + ", RW x size: " + str(xsize) + ", RW y size: " + str(ysize))
		#rospy.loginfo("x: " + str(real_x) + ", y: " + str(real_y))
		return [real_x, real_y]


	def create_tpm(self):
		'''
		
		UP --> (SxS)
		DW --> (SxS)
		LT --> (SxS)
		RT --> (SxS)
		
		P --> (AxSxS)

		Iterate through States(rows, column) and assign a probability value to each (State, State) pair given a certain Action is performed.
		When Action can't be performed, the probability of (State, State) is 1.0;
		When Action can be performed, the probability of (State, State) is 0.2, if Action fails;
		When Action can be performed, the probability of (State, next_State) is 0.8, if Action succeeds.
		'''
		P = []
		S = len(self.freeStates)

		UP = np.array([[0.0] * S for a in range(S)])
		DW = np.array([[0.0] * S for b in range(S)])
		LT = np.array([[0.0] * S for c in range(S)])
		RT = np.array([[0.0] * S for d in range(S)])

		for idx in range(len(self.freeStates)):
			state = self.freeStates[idx]
			state_row = state[0]
			state_column = state[1]
			curr_state = (state_row * self.map_width) + state_column
			state_up = ((state_row+1) * self.map_width) + state_column
			state_down = ((state_row-1) * self.map_width) + state_column
			state_left = (state_row * self.map_width) + (state_column-1)
			state_right = (state_row * self.map_width) + (state_column+1)

			########################### UP TRANSITION #########################
			if (state_row == self.map_height-1) or (self.map_data[state_up] == 100):
				UP[idx][idx] = 1.0
			else:
				if [state_row+1, state_column] in self.freeStates:
					idx_up = self.freeStates.index([state_row+1, state_column])
					UP[idx][idx_up] = 0.8
					UP[idx][idx] = 0.2

			########################### DOWN TRANSITION #########################
			if (state_row == 0) or (self.map_data[state_down] == 100):
				DW[idx][idx] = 1.0
			else:
				if [state_row-1, state_column] in self.freeStates:
					idx_down = self.freeStates.index([state_row-1, state_column])
					DW[idx][idx_down] = 0.8
					DW[idx][idx] = 0.2

			########################### LEFT TRANSITION #########################
			if (state_column == 0) or (self.map_data[state_left] == 100):
				LT[idx][idx] = 1.0
			else:
				if [state_row, state_column-1] in self.freeStates:
					idx_left = self.freeStates.index([state_row, state_column-1])
					LT[idx][idx_left] = 0.8
					LT[idx][idx] = 0.2

			########################### RIGHT TRANSITION #########################
			if (state_column == self.map_width-1) or (self.map_data[state_right] == 100):
				RT[idx][idx] = 1.0
			else:
				if [state_row, state_column+1] in self.freeStates:
					idx_right = self.freeStates.index([state_row, state_column+1])
					RT[idx][idx_right] = 0.8
					RT[idx][idx] = 0.2
		P = np.array([UP, DW, LT, RT])
		#rospy.loginfo("\nP\n"+str(P))
		return P

	def create_rm(self, P, goal, cliff):
		'''
		R --> (AxS)

		Iterate trough States(rows) and Actions(columns) and assign a reward value for each (State, Action) pair, being the reward given when Action is performed in State.

		Since we want to obtain the policy that maximizes the future expected discounted reward:
			- if we try to perform an invalid Action from State the reward will be a low negative value in order to discourage the optimal policy of containing Actions that will not result in progress towards the goal.
			- if we try to perform an Action from a State, that will result in the next State being the cliff the reward will be very high negative value so that the optimal policy will avoid passing through that cell.
			- if we try to perform a valid Action from State, that will result in the next State being the goal, the reward will be a very high positive value so that the optimal policy will attempt to reach that cell.
			- if we try to perform a valid Action from State, that will result in the next State not being the goal, the reward will be a low positive value so that the optimal policy will pass through these cells.
		'''
		A = len(P)
		S = len(self.freeStates)
		R = np.array([[0] * A for k in range(S)])
		
		for i in range(A):
			Action = P[i]
			for j in range(S):
				State = self.freeStates[j]
				state_row = State[0]
				state_column = State[1]

				########################################################################### UP ################################################################################
				if i == 0:	
					#if we try to perform an invalid Action from State the reward will be a low negative value in order to discourage the optimal policy of containing Actions that will not result in progress towards the goal.
					if Action[j][j] == 1.0:
						R[j][i] = -1
					#if we try to perform a valid Action from State:
					elif Action[j][j] == 0.2:
						#that will result in the next State being the goal, the reward will be a very high positive value so that the optimal policy will attempt to reach that cell.
						if [state_row-1, state_column] == goal:
							R[j][i] = 100
						
						# that will result in the next State being the cliff the reward will be very high negative value so that the optimal policy will avoid passing through that cell.
						elif [state_row-1, state_column] == cliff:
							R[j][i] = -100
						
						#that will result in the next State not being the goal or the cliff, the reward will be a low positive value so that the optimal policy will pass through these cells.
						else:
							R[j][i] = 1
				
				########################################################################## DOWN ###############################################################################
				if i == 1:	
					#if we try to perform an invalid Action from State the reward will be a low negative value in order to discourage the optimal policy of containing Actions that will not result in progress towards the goal.
					if Action[j][j] == 1.0:
						R[j][i] = -1
					#if we try to perform a valid Action from State:
					elif Action[j][j] == 0.2:
						#that will result in the next State being the goal, the reward will be a very high positive value so that the optimal policy will attempt to reach that cell.
						if [state_row+1, state_column] == goal:
							R[j][i] = 100
						
						# that will result in the next State being the cliff the reward will be very high negative value so that the optimal policy will avoid passing through that cell.
						elif [state_row+1, state_column] == cliff:
							R[j][i] = -100
						
						#that will result in the next State not being the goal or the cliff, the reward will be a low positive value so that the optimal policy will pass through these cells.
						else:
							R[j][i] = 1

				########################################################################## LEFT ###############################################################################
				if i == 2:	
					#if we try to perform an invalid Action from State the reward will be a low negative value in order to discourage the optimal policy of containing Actions that will not result in progress towards the goal.
					if Action[j][j] == 1.0:
						R[j][i] = -1
					#if we try to perform a valid Action from State:
					elif Action[j][j] == 0.2:
						#that will result in the next State being the goal, the reward will be a very high positive value so that the optimal policy will attempt to reach that cell.
						if [state_row, state_column-1] == goal:
							R[j][i] = 100
						
						# that will result in the next State being the cliff the reward will be very high negative value so that the optimal policy will avoid passing through that cell.
						elif [state_row, state_column-1] == cliff:
							R[j][i] = -100
						
						#that will result in the next State not being the goal or the cliff, the reward will be a low positive value so that the optimal policy will pass through these cells.
						else:
							R[j][i] = 1
				
				########################################################################## RIGHT ###############################################################################
				if i == 3:	
					#if we try to perform an invalid Action from State the reward will be a low negative value in order to discourage the optimal policy of containing Actions that will not result in progress towards the goal.
					if Action[j][j] == 1.0:
						R[j][i] = -1
					#if we try to perform a valid Action from State:
					elif Action[j][j] == 0.2:
						#that will result in the next State being the goal, the reward will be a very high positive value so that the optimal policy will attempt to reach that cell.
						if [state_row, state_column+1] == goal:
							R[j][i] = 100
						
						# that will result in the next State being the cliff the reward will be very high negative value so that the optimal policy will avoid passing through that cell.
						elif [state_row, state_column+1] == cliff:
							R[j][i] = -100
						
						#that will result in the next State not being the goal or the cliff, the reward will be a low positive value so that the optimal policy will pass through these cells.
						else:
							R[j][i] = 1
		return R

class PolicyIter():
	def __init__(self, P, R):
		self.P = P
		self.R = R
		self.discount = 0.96
		self.policy = self.run_mdp()

	def run_mdp(self):
		pi = mdptoolbox.mdp.PolicyIteration(self.P, self.R, self.discount)
		pi.run()
		Pol = pi.policy
		rospy.loginfo(Pol)
		return Pol

class ValueIter():
	def __init__(self, P, R):
		self.P = P
		self.R = R
		self.discount = 0.96
		self.policy = self.run_mdp()

	def run_mdp(self):
		vi = mdptoolbox.mdp.ValueIteration(self.P, self.R, self.discount)
		vi.run()
		Pol = vi.policy
		rospy.loginfo(Pol)
		return Pol

class QLearn():
	def __init__(self, P, R):
		self.P = P
		self.R = R
		self.discount = 0.96
		self.policy = self.run_mdp()

	def run_mdp(self):
		ql = mdptoolbox.mdp.QLearning(self.P, self.R, self.discount)
		ql.run()
		Pol = ql.policy
		rospy.loginfo(Pol)
		return Pol


def get_state_action_sequence(initial_state, goal_state, policy, freeStates):
	Action_sequence = []
	State_sequence = []
	current_state = initial_state
	#rospy.loginfo(goal_state)
	while current_state != goal_state:
		state_row = current_state[0]
		state_column = current_state[1]
		#rospy.loginfo(current_state)
		idx = freeStates.index([state_row, state_column])
		Action = policy[idx]
		#rospy.loginfo(Action)
		#rospy.loginfo(idx)

		if Action == 0:
			current_state = [state_row+1, state_column]
		elif Action == 1:
			current_state = [state_row-1, state_column]
		elif Action == 2:
			current_state = [state_row, state_column-1]
		elif Action == 3:
			current_state = [state_row, state_column+1]

		Action_sequence.append(Action)
		State_sequence.append(current_state)

	return Action_sequence, State_sequence

if __name__ == '__main__':
	my_prep = pre_mdp()
	mbc = MoveBaseClient()
	mdp_type = "value_iteration"
	while not rospy.is_shutdown():

		if my_prep.map_callback == True:
			P = my_prep.create_tpm()
			#goal_1 = random.choice(my_prep.freeStates)
			#goal_2 = random.choice(my_prep.freeStates)
			#cliff = random.choice(my_prep.freeStates)
			#initial_state = my_prep.freeStates[3]
			#rospy.loginfo("\ninitial_state:"+str(initial_state))
			#initial_pose = my_prep.state_to_cell_center_coordinate(initial_state)
			#q = transformations.quaternion_from_euler(-0.000004, 0.003180, 0.000146)
			#rospy.loginfo("\nq:"+str(q))
			#rospy.loginfo("\ninitial_pose:"+str(initial_pose))
			'''
			goal_1 = my_prep.freeStates[6]
			goal_2 = my_prep.freeStates[7]
			cliff = my_prep.freeStates[0]
			'''
			'''
			#test set 1
			initial_state = [1, 9]
			#initial_pose = my_prep.state_to_cell_center_coordinate(initial_state)
			#rospy.loginfo("\ninitial_pose:"+str(initial_pose))
			goal_1 = [8, 16]
			goal_2 = [10, 3]
			cliff = [3, 14]
			'''
			
			#test set 2
			initial_state = [8, 16]
			#initial_pose = my_prep.state_to_cell_center_coordinate(initial_state)
			#rospy.loginfo("\ninitial_pose:"+str(initial_pose))
			goal_1 = [10, 3]
			goal_2 = [6, 7]
			cliff = [3, 14]
			
			
			if goal_1 != goal_2 or goal_1 != cliff or goal_2 != cliff:
				# stop loop of giving always the state [0,0] - for tests
				my_prep.map_callback == False
				
				R_mdp1 = my_prep.create_rm(P, goal_1, cliff)
				R_mdp2 = my_prep.create_rm(P, goal_2, cliff)
				if (mdp_type == "policy_iteration"):
					Pol_1 = PolicyIter(P, R_mdp1).policy
					Pol_2 = PolicyIter(P, R_mdp2).policy
				elif (mdp_type == "value_iteration"):
					Pol_1 = ValueIter(P, R_mdp1).policy
					Pol_2 = ValueIter(P, R_mdp2).policy
				elif (mdp_type == "qlearning"):
					Pol_1 = QLearn(P, R_mdp1).policy
					Pol_2 = QLearn(P, R_mdp2).policy
					
				Actions_seq_1, State_seq_1 = get_state_action_sequence(initial_state, goal_1, Pol_1, my_prep.freeStates)
				#rospy.loginfo("next_goal")
				Actions_seq_2, State_seq_2 = get_state_action_sequence(goal_1, goal_2, Pol_2, my_prep.freeStates)
				Pose_seq_1 = []
				Pose_seq_2 = []

				#rospy.loginfo("\nAct_seq_1:"+str(Actions_seq_1))
				#rospy.loginfo("\nAct_seq_2:"+str(Actions_seq_2))

				for state1 in State_seq_1:
					pose1 = my_prep.state_to_cell_center_coordinate(state1)
					Pose_seq_1.append(pose1)
				#rospy.loginfo("\nPose_seq_1:"+str(Pose_seq_1))
				
				num_poses_1 = len(Pose_seq_1)
				
				for idx in range(num_poses_1):
					pose = Pose_seq_1[idx]
					goal = PoseStamped()
					goal.header.frame_id = "map"
					goal.header.stamp = rospy.Time.now()
					goal.pose.position.x = pose[0]
					goal.pose.position.y = pose[1]
					
					if (idx <= num_poses_1 - 2):
						if Actions_seq_1[idx+1] == 0:
							y = np.pi/2.0
						if Actions_seq_1[idx+1] == 1:
							y = (3.0*np.pi)/4.0
						if Actions_seq_1[idx+1] == 2:
							y = np.pi
						if Actions_seq_1[idx+1] == 3:
							y = 0
					
						q = transformations.quaternion_from_euler(0, 0, y)
						goal.pose.orientation.x = q[0]
						goal.pose.orientation.y = q[1]
						goal.pose.orientation.z = q[2]
						goal.pose.orientation.w = q[3]
					mbc.give_goal_cell(goal, num_poses_1)

				mbc.send_first_goal()

				for state2 in State_seq_2:
					pose2 = my_prep.state_to_cell_center_coordinate(state2)
					Pose_seq_2.append(pose2)
				#rospy.loginfo("\nPose_seq_2:"+str(Pose_seq_2))

				num_poses_2 = len(Pose_seq_2)

				for idx in range(num_poses_2):
					pose = Pose_seq_2[idx]
					goal = PoseStamped()
					goal.header.frame_id = "map"
					goal.header.stamp = rospy.Time.now()
					goal.pose.position.x = pose[0]
					goal.pose.position.y = pose[1]

					if (idx <= num_poses_2 - 2):
						if Actions_seq_2[idx+1] == 0:
							y = np.pi/2.0
						if Actions_seq_2[idx+1] == 1:
							y = (3.0*np.pi)/4.0
						if Actions_seq_2[idx+1] == 2:
							y = np.pi
						if Actions_seq_2[idx+1] == 3:
							y = 0
						
						q = transformations.quaternion_from_euler(0, 0, y)
						goal.pose.orientation.x = q[0]
						goal.pose.orientation.y = q[1]
						goal.pose.orientation.z = q[2]
						goal.pose.orientation.w = q[3]

					mbc.give_goal_cell(goal, num_poses_2)
				
				mbc.send_first_goal()

				#rospy.signal_shutdown("Reached the 2nd goal!\nTerminating Process...")
				
			else:
				continue
		else:
			continue
		# stop loop of giving always the state [0,0] - for tests		
		break

	rospy.spin()
