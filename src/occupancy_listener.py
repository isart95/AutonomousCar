#!/usr/bin/env python

import rospy
import numpy as np
from math import floor
from nav_msgs.msg import OccupancyGrid

class map_to_W_coord():
	def __init__ (self):
		rospy.init_node('map_to_W_coord')
		sub_map = rospy.Subscriber('/disc_map', OccupancyGrid, self.map_callback)

	def map_callback(self, disc_map):
		self.cell_size = disc_map.info.resolution
		self.map_originX = disc_map.info.origin.position.x
		self.map_originY = disc_map.info.origin.position.y
		self.map_height = disc_map.info.height
		self.map_width = disc_map.info.width

		
		i = 0
		possibleStates = []
		occupiedStates = []
		for cell_value in disc_map.data:
			if cell_value == 0:
				possibleStates.append("state " + str(i))
			elif cell_value == 100:
				occupiedStates.append("state " + str(i))
			i+=1
		#print('Occupied States:' + str(occupiedStates))
		#print('Possible States:' + str(possibleStates))

		testState = self.pos_to_state(2.7, 0)
		print(("state " + str(int(testState))))
		print(("state " + str(int(testState)) in possibleStates))

		print("################")
		newX, newY = self.cell_to_pos(testState)
		print("Reversed X is {} and Y is {}".format(newX, newY))

	def pos_to_state(self, x, y):
		x -= self.map_originX
		y -= self.map_originY

		xPos = int(floor(x / self.cell_size))
		yPos = int(floor(y / self.cell_size))
		return int(floor(xPos) + (floor(yPos)*self.map_height))


	def cell_to_pos(self, cellN):
		#cellN = int(cellN.replace('state', ''))
		rest = cellN % self.map_height
		xPos = (rest * self.cell_size) + self.map_originX + (self.cell_size/2)
		yPos = ((cellN / self.map_height)* self.cell_size) + self.map_originY + (self.cell_size/2)
		return xPos, yPos


if __name__ == '__main__':
	while not rospy.is_shutdown():
		my_converter = map_to_W_coord()
	rospy.spin()