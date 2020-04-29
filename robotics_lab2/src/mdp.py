#!/usr/bin/env python

import rospy
import numpy as np
import random
import mdptoolbox

from nav_msgs.msg import OccupancyGrid

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
				if (map_data[state_number] == 100):
					self.occupiedStates.append(index_matrix)
		self.map_height = map_height
		self.map_width = map_width
		self.map_resolution = map_resolution
		self.map_data = map_data
		self.map_callback = True

	def state_to_cell_center_coordinate(self, state):
		#allocate a matrix for all the map cells (represented by the x and y coordinates of their center) according to the map frame
		grid_size = self.map_resolution
		xsize = self.map_height*grid_size
		ysize = self.map_width*grid_size
		
		cell_coordinate = []
		
		state_row = state[0]
		state_column = state[1]

		cell_x_coordinates = np.arange(-xsize/2.0, xsize/2.0, grid_size, dtype = float)[:, None].T + grid_size/2.0
		cell_y_coordinates = np.arange(-ysize/2.0, ysize/2.0, grid_size, dtype = float)[:, None] + grid_size/2.0
		grid_cells_x_coordinates = np.tile(cell_x_coordinates, (int(ysize/grid_size), 1)) #each line repeated xsize times
		grid_cells_y_coordinates = np.tile(cell_y_coordinates, (1, int(xsize/grid_size))) # each column repeated ysize times
		grid_cells_center = np.array([grid_cells_x_coordinates, grid_cells_y_coordinates], dtype = float)
		rospy.loginfo(grid_cells_center)
		state_cell_center_x = grid_cells_center[0][state_row][state_column]
		state_cell_center_y = grid_cells_center[1][state_row][state_column]
		state_cell_center[0] = state_cell_center_x
		state_cell_center[1] = state_cell_center_y


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
			state_up = ((state_row-1) * self.map_width) + state_column
			state_down = ((state_row+1) * self.map_width) + state_column
			state_left = (state_row * self.map_width) + (state_column-1)
			state_right = (state_row * self.map_width) + (state_column+1)

			########################### UP TRANSITION #########################
			if (state_row == 0) or (self.map_data[state_up] == 100):
				UP[idx][idx] = 1.0
			else:
				if [state_row-1, state_column] in self.freeStates:
					idx_up = self.freeStates.index([state_row-1, state_column])
					UP[idx][idx_up] = 0.8
					UP[idx][idx] = 0.2

			########################### DOWN TRANSITION #########################
			if (state_row == self.map_height-1) or (self.map_data[state_down] == 100):
				DW[idx][idx] = 1.0
			else:
				if [state_row+1, state_column] in self.freeStates:
					idx_down = self.freeStates.index([state_row+1, state_column])
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
		self.run_mdp()

	def run_mdp(self):
		pi = mdptoolbox.mdp.PolicyIteration(self.P, self.R, self.discount)
		pi.run()
		Pol = pi.policy
		#rospy.loginfo(Pol)
		return Pol

class ValueIter():
	def __init__(self, P, R):
		self.P = P
		self.R = R
		self.discount = 0.96
		self.run_mdp()

	def run_mdp(self):
		pi = mdptoolbox.mdp.ValueIteration(self.P, self.R, self.discount)
		pi.run()
		Pol = pi.policy
		#rospy.loginfo(Pol)
		return Pol

class QLearn():
	def __init__(self, P, R):
		self.P = P
		self.R = R
		self.discount = 0.96
		self.run_mdp()

	def run_mdp(self):
		pi = mdptoolbox.mdp.QLearning(self.P, self.R, self.discount)
		pi.run()
		Pol = pi.policy
		#rospy.loginfo(Pol)
		return Pol


def get_action_sequence(initial_state, goal_state, policy, map_width):
	Action_sequence = []
	current_state = initial_state

	while current_state != goal_state:
		state_row = current_state[0]
		state_column = current_state[1]
		idx = (state_row * self.map_width) + state_column
		Action = policy[idx]

		if Action == 0:
			current_state = [state_row, state_column]
			Action_sequence.append(Action)

		if Action == 1:
			current_state = [state_row, state_column]
			Action_sequence.append(Action)

		if Action == 2:
			current_state = [state_row, state_column]
			Action_sequence.append(Action)
		
		if Action == 3:
			current_state = [state_row, state_column]
			Action_sequence.append(Action)



if __name__ == '__main__':
	my_prep = pre_mdp()
	mdp_type = "policy_iteration"
	while not rospy.is_shutdown():

		if my_prep.map_callback == True:
			P = my_prep.create_tpm()
			#goal_1 = random.choice(my_prep.freeStates)
			#goal_2 = random.choice(my_prep.freeStates)
			#cliff = random.choice(my_prep.freeStates)
			#initial_state = my_prep.freeStates[3]
			#goal_1 = my_prep.freeStates[6]
			#goal_2 = my_prep.freeStates[7]
			#cliff = my_prep.freeStates[0]
		
			########### TEST SECTION JOAO START #########################
			# UNCOMMENT THIS SECTION AND COMMENT THE IF-ELSE SECTION BELOW
			
			state = [0, 0]
			my_prep.state_to_cell_center_coordinate(state)
			
			########### TEST SECTION JOAO END #########################

			######### IF-ELSE SECTION START #################################
			'''
			if goal_1 != goal_2 or goal_1 != cliff or goal_2 != cliff:
				R_mdp1 = my_prep.create_rm(P, goal_1, cliff)
				#rospy.loginfo("\nR_mdp1\n"+str(R_mdp1))
				R_mdp2 = my_prep.create_rm(P, goal_2, cliff)
				#rospy.loginfo("\nR_mdp2\n"+str(R_mdp2))
				if (mdp_type == "policy_iteration"):
					Pol_1 = PolicyIter(P, R_mdp1)

					Pol_2 = PolicyIter(P, R_mdp2)

					#rospy.signal_shutdown("Reached the 2nd goal!\nTerminating Process...")

				elif (mdp_type == "value_iteration"):
					Pol_1 = ValueIter(P, R_mdp1)
					Pol_2 = ValueIter(P, R_mdp2)

					#rospy.signal_shutdown("Reached the 2nd goal!\nTerminating Process...")
				
				elif (mdp_type == "qlearning"):
					Pol_1 = QLearn(P, R_mdp1)
					Pol_2 = QLearn(P, R_mdp2)
					Actions_seq_1 = get_action_sequence(initial_state, goal1, Pol_1)
					Actions_seq_2 = get_action_sequence(goal_1, goal2, Pol_2)
					#rospy.signal_shutdown("Reached the 2nd goal!\nTerminating Process...")
			else:
				continue
			'''
			##################3 IF-ELSE SECTION END #############################

		else:
			continue

	rospy.spin()
