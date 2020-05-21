#!/usr/bin/env python

import rospy

from math import floor, ceil

from nav_msgs.msg import OccupancyGrid
import numpy as np

def round_up(n, decimals = 0):
	multiplier = 10 ** decimals
	return ceil(n * multiplier) / multiplier

def round_down(n, decimals = 0):
	multiplier = 10 ** decimals
	return floor(n * multiplier) / multiplier


class map_discretizer():
	def __init__(self, robot_length):
		self.robot_length = robot_length
		resize_factor = 0.0
		new_grid_height = 0
		new_grid_width = 0
		self.sub_map = rospy.Subscriber('/map', OccupancyGrid, self.map_cb)
		self.pub_map = rospy.Publisher('/disc_map', OccupancyGrid, queue_size=1)
		

	def map_cb(self, msg):
		self.g_map = msg
		self.create_resized_grid(self.robot_length)

	'''Calulates the resize factor of the grid according to the argument robot length and the map's resolution.'''
	def get_resize_factor(self, robot_length):
		resize_factor = (robot_length/self.g_map.info.resolution)
		return resize_factor

	'''Calculates the length sized cells grid dimension sizes, the height and the width, according to the argument resize factor. '''
	def resize_map_width_height(self, resize_factor):
		new_grid_height = int(round_up(self.g_map.info.height / resize_factor))
		new_grid_width = int(round_up(self.g_map.info.width / resize_factor))
		return new_grid_height, new_grid_width

	'''Gets the index of the occupancy grid map of where the robot is in the map'''
	def robot_pose_map_to_gridmap(self, robot_pose_x, robot_pose_y):
		self.grid_pose_x = int((robot_pose_x - self.g_map.info.origin.position.x) / self.g_map.info.resolution)
		self.grid_pose_y = int((robot_pose_y - self.g_map.info.origin.position.y) / self.g_map.info.resolution)

	'''Creates a matrix that represents the occupancy grid map'''
	def create_occupancy_grid(self):
		self.grid = [[0] * self.g_map.info.width for i in range(self.g_map.info.height)]
		for i in range(self.g_map.info.height):
			for j in range(self.g_map.info.width):
				self.grid[i][j] = self.g_map.data[i * self.g_map.info.width + j]
		

	'''Creates a new, obstacle free, grid with robot length sized cellsS'''
	def create_grid_w_length_sized_cells(self, new_grid_height, new_grid_width):
		self.new_grid = [[0] * new_grid_width for i in range(new_grid_height)]

	'''Fill the obstacle free grid with obstacles according to the original grid and the resize factor.'''
	def fill_obstacle_free_grid_with_obstacles(self, new_grid_height, new_grid_width, resize_factor):
		for row in range(self.g_map.info.height):
			for column in range(self.g_map.info.width):
				if self.grid[row][column] != 0:
					length_grid_row = int(round_down(row / resize_factor))
					length_grid_column = int(round_down(column / resize_factor))
					if self.new_grid[length_grid_row][length_grid_column] == 0:
						self.new_grid[length_grid_row][length_grid_column] = 100
		
	'''Create a resized grid, that is resized according to the resize factor and length'''
	def create_resized_grid(self, robot_length):
		resize_factor = self.get_resize_factor(robot_length)
		grid_height, grid_width = self.resize_map_width_height(resize_factor)
		self.create_grid_w_length_sized_cells(grid_height, grid_width)
		self.create_occupancy_grid()
		self.fill_obstacle_free_grid_with_obstacles(grid_height, grid_width, resize_factor)
		
		disc_map = OccupancyGrid()
		disc_map.header.stamp = rospy.Time.now()
		disc_map.header.frame_id = 'disc_map'
		disc_map.info.resolution = robot_length
		disc_map.info.height = grid_height
		disc_map.info.width = grid_width
		disc_map.info.origin = self.g_map.info.origin
		
		self.new_grid_data = [item for sublist in self.new_grid for item in sublist]
		
		disc_map.data = self.new_grid_data
		
		while True:
			self.pub_map.publish(disc_map)
			rospy.sleep(2)

if __name__ == '__main__':
	rospy.init_node('map_discretizer')

	while not rospy.is_shutdown():
		turtlebot3_length = 1.508
		disc = map_discretizer(turtlebot3_length)
		rospy.spin()