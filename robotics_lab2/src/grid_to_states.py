#!/usr/bin/env python

import rospy
import numpy as np

from nav_msgs.msg import OccupancyGrid

class mdp_prep():
	def __init__(self, goal1, goal2, cliff):
		rospy.init_node('grid_cells_to_states')
		plan_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_cb)
		self.freeStates = []
		self.occupiedStates = []
		self.P = []
		self.R = []
		self.goal1 = goal1
		self.goal2 = goal2
		self.cliff = cliff

	def map_cb(self, disc_map):
		for i in range(disc_map.info.height):
			for j in range(disc_map.info.width):
				state_number = (i * disc_map.info.width) + j
				index_matrix = [i, j]
				if (disc_map.data[state_number] == 0):
					self.freeStates.append(index_matrix)
				if (disc_map.data[state_number] == 100):
					self.occupiedStates.append(index_matrix)
	
	def create_tpm(self):
		self.S = len(self.freeStates)

		U = np.array([[0.0] * self.S for k in range(self.S)])
		D = np.array([[0.0] * self.S for k in range(self.S)])
		L = np.array([[0.0] * self.S for k in range(self.S)])
		R = np.array([[0.0] * self.S for k in range(self.S)])

		for idx in range(self.freeStates):
			state = self.freeStates[idx]
			state_row = state[0]
			state_column = state[1]
			curr_state = (state_row * disc_map.info.width) + state_column
			state_up = ((state_row-1) * disc_map.info.width) + state_column
			state_down = ((state_row+1) * disc_map.info.width) + state_column
			state_left = (state_row * disc_map.info.width) + (state_column-1)
			state_right = (state_row * disc_map.info.width) + (state_column+1)

			########################### UP TRANSITION #########################
			if (state_row == 0) or (disc_map.data[state_up] == 100):
				U[idx][idx] = 1.0
			else:
				if [state_row-1, state_column] in self.freeStates:
					idx_up = self.freeStates.index([state_row-1, state_column])
					U[idx][idx_up] = 0.8
					U[idx][idx] = 0.2

			########################### DOWN TRANSITION #########################
			if (state_row == disc_map.info.height-1) or (disc_map.data[state_down] == 100):
				D[idx][idx] = 1.0
			else:
				if [state_row+1, state_column] in self.freeStates:
					idx_down = self.freeStates.index([state_row+1, state_column])
					D[idx][idx_down] = 0.8
					D[idx][idx] = 0.2

			########################### LEFT TRANSITION #########################
			if (state_column == 0) or (disc_map.data[state_left] == 100):
				L[idx][idx] = 1.0
			else:
				if [state_row, state_column-1] in self.freeStates:
					idx_left = self.freeStates.index([state_row, state_column-1])
					L[idx][idx_left] = 0.8
					L[idx][idx] = 0.2

			########################### RIGHT TRANSITION #########################
			if (state_column == disc_map.info.width-1) or (disc_map.data[state_right] == 100):
				R[idx][idx] = 1.0
			else:
				if [state_row, state_column+1] in self.freeStates:
					idx_right = self.freeStates.index([state_row, state_column+1])
					R[idx][idx_right] = 0.8
					R[idx][idx] = 0.2
		self.transitions = [U, D, L, R]
		self.A = len(self.transitions)

	def create_reward(self,):
		R = np.array([[0] * self.S for k in range(self.A)])

if __name__ == '__main__':
	mdp_prep()