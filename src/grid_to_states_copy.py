#!/usr/bin/env python

import rospy
import numpy as np

from nav_msgs.msg import OccupancyGrid

class grid_cells_to_states():
	def __init__(self):
		rospy.init_node('grid_cells_to_states')
		plan_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_cb)
		self.freeStates = []
		self.occupiedStates = []
		self.States = {}
		self.Transition_matrices = {}

	def map_cb(self, disc_map):
		for i in range(disc_map.info.height):
			for j in range(disc_map.info.width):
				state_number = (i * disc_map.info.width) + j
				index_matrix = [i, j]
				if (disc_map.data[state_number] == 0):
					self.freeStates.append(index_matrix)
					rospy.loginfo(self.freeStates)
				if (disc_map.data[state_number] == 100):
					self.occupiedStates.append(index_matrix)

		for state in self.freeStates:
			State = {}
			state_row = state[0]
			state_column = state[1]
			state_number = (state_row * disc_map.info.width) + state_column

			if (state_row > 0) and (state_row < disc_map.info.height-1):
				state_up = [state_row-1, state_column]
				state_down = [state_row+1, state_column]
			else:
				state_up = None
				state_down = None
			
			if (state_column > 0) and (state_column < disc_map.info.width-1):
				state_left = [state_row, state_column-1]
				state_right = [state_row, state_column+1]
			else:
				state_left = None
				state_right = None

			State['current_state'] = state
			
			if state_up in self.freeStates:
				State['state_up'] = state_up
			else:
				State['state_up'] = None
			
			if state_down in self.freeStates:
				State['state_down'] = state_down
			else:
				State['state_down'] = None
			
			if state_left in self.freeStates:
				State['state_left'] = state_left
			else:
				State['state_left'] = None
			
			if state_right in self.freeStates:
				State['state_right'] = state_right
			else:
				State['state_right'] = None

			key = 'state_{}'.format(state_number)
			self.States[key] = State
			num_states = len(self.States)
		
		U = np.array([[0.0] * num_states for k in range(num_states)])
		D = np.array([[0.0] * num_states for k in range(num_states)])
		L = np.array([[0.0] * num_states for k in range(num_states)])
		R = np.array([[0.0] * num_states for k in range(num_states)])
		
		for state in self.States:
			curr_state = self.States[state].get('current_state')
			rospy.loginfo(curr_state)
			state_up = self.States[state].get('state_up')
			#rospy.loginfo(state_up)
			state_down = self.States[state].get('state_down')
			#rospy.loginfo(state_down)
			state_left = self.States[state].get('state_left')
			#rospy.loginfo(state_left)
			state_right =  self.States[state].get('state_right')
			#rospy.loginfo(state_right)
			
			if state_up == None:
				row = curr_state[0]
				column = curr_state[1]
				state_i = (row * disc_map.info.width) + column
				U[state_i][state_i] = 1.0
			else:
				row = curr_state[0]
				column = curr_state[1]
				state_i = (row * disc_map.info.width) + column
				U[state_i][state_i] = 0.2
				
				row = state_up[0]
				column = state_up[1]
				state_i = (row * disc_map.info.width) + column
				U[state_i][state_i] = 0.8

			if state_down == None:
				row = curr_state[0]
				column = curr_state[1]
				state_i = (row * disc_map.info.width) + column
				D[state_i][state_i] = 1.0
			else:
				row = curr_state[0]
				column = curr_state[1]
				state_i = (row * disc_map.info.width) + column
				D[state_i][state_i] = 0.2
				
				row = state_down[0]
				column = state_down[1]
				state_i = (row * disc_map.info.width) + column
				#rospy.loginfo(state_i)
				D[state_i][state_i] = 0.8

			if state_left == None:
				row = curr_state[0]
				column = curr_state[1]
				state_i = (row * disc_map.info.width) + column
				L[state_i][state_i] = 1.0
			else:
				row = curr_state[0]
				column = curr_state[1]
				state_i = (row * disc_map.info.width) + column
				L[state_i][state_i] = 0.2
				
				row = state_left[0]
				column = state_left[1]
				state_i = (row * disc_map.info.width) + column
				L[state_i][state_i] = 0.8

			if state_right == None:
				row = curr_state[0]
				column = curr_state[1]
				state_i = (row * disc_map.info.width) + column
				R[state_i][state_i] = 1.0
			else:
				row = curr_state[0]
				column = curr_state[1]
				state_i = (row * disc_map.info.width) + column
				R[state_i][state_i] = 0.2
				
				row = state_right[0]
				column = state_right[1]
				state_i = (row * disc_map.info.width) + column
				R[state_i][state_i] = 0.8

		self.Transition_matrices['action_up'] = U
		self.Transition_matrices['action_down'] = D
		self.Transition_matrices['action_left'] = L
		self.Transition_matrices['action_right'] = R
		rospy.loginfo(self.Transition_matrices)

if __name__ == '__main__':
	grid_cells_to_states()
		
