#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import numpy as np

def move_square():
	# Starts a new node
	rospy.init_node('turtle_localizer')
	velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
	vel_msg = Twist()

	#Receiveing the user's input
	#print("Let's move your robot")
	speed = 0.2
	rotation_speed = 0.5

	distance = 0.5	
	angle = np.pi/2.0
	
	#number of straight lines
	moved_straight = 0

	isForward = True
	isClockwise = False

	#Since we are moving just in x-axis
	vel_msg.linear.y = 0.0
	vel_msg.linear.z = 0.0
	vel_msg.angular.x = 0.0
	vel_msg.angular.y = 0.0
	vel_msg.angular.z = 0.0

	while not rospy.is_shutdown():

		#current_distance = 0.0
		#current_angle = 0.0

		#Loop to move the turtle in an specified distance
		while (moved_straight < 4):
			
			current_distance = 0.0
			current_angle = 0.0
			
			if (isForward):
				vel_msg.linear.x = abs(speed)
				vel_msg.angular.z = 0.0
				#print("Moving forward...")

				#Setting the current time for distance calculus
				t0 = rospy.Time.now().to_sec()

				while(current_distance < distance):
					#print("Moving forward...")
					#print(distance)
					#print(current_distance)
					#Publish the velocity
					velocity_publisher.publish(vel_msg)
					#Takes actual time to velocity calculus
					t1 = rospy.Time.now().to_sec()
					#Calculates distance
					current_distance = speed*(t1-t0)
					
					#print("distance:" + str(distance))
					#print("current_distance:" + str(current_distance))
			
			else:
				vel_msg.linear.x = -abs(speed)
				vel_msg.angular.z = 0.0

				#Setting the current time for distance calculus
				t0 = rospy.Time.now().to_sec()

				while(current_distance < distance):
					#print(distance)
					#print(current_distance)
					#Publish the velocity
					velocity_publisher.publish(vel_msg)
					#Takes actual time to velocity calculus
					t1 = rospy.Time.now().to_sec()
					#Calculates distance
					current_distance = speed*(t1-t0)

				#print("distance:" + str(distance))
				#print("current_distance:" + str(current_distance))

			moved_straight += 1

			#print("GOT HERE1!!!")

			if (isClockwise):
				vel_msg.linear.x = 0.0
				vel_msg.angular.z = -abs(rotation_speed)

				#print("Rotating...")

				#Setting the current time for rotation calculus
				t0 = rospy.Time.now().to_sec()

				while(current_angle < angle):
					#print("Rotating...")
					#print(angle)
					#print(current_angle)
					#Publish the velocity
					velocity_publisher.publish(vel_msg)
					#Takes actual time to velocity calculus
					t1 = rospy.Time.now().to_sec()
					#Calculates rotation
					current_angle = rotation_speed*(t1-t0)

				#print("angle:" + str(angle))
				#print("current_angle:" + str(current_angle))

			else:
				vel_msg.linear.x = 0.0
				vel_msg.angular.z = abs(rotation_speed)

				#print("Rotating...")

				#Setting the current time for rotation calculus
				t0 = rospy.Time.now().to_sec()

				while(current_angle < angle):
					#print(angle)
					#print(current_angle)
					#Publish the velocity
					velocity_publisher.publish(vel_msg)
					#Takes actual time to velocity calculus
					t1 = rospy.Time.now().to_sec()
					#Calculates rotation
					current_angle = rotation_speed*(t1-t0)

				#print("angle:" + str(angle))
				#print("current_angle:" + str(current_angle))

			#print("GOT HERE2!!!")
		if (moved_straight == 4):
			#After the loop, stops the robot
			vel_msg.linear.x = 0
			vel_msg.angular.z = 0
			#Force the robot to stop
			velocity_publisher.publish(vel_msg)

			moved_straight += 1


if __name__ == '__main__':
	try:
		rospy.sleep(10)
		#Testing our function
		move_square()
		rospy.spin()
	except rospy.ROSInterruptException: pass