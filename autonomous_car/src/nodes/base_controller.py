#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
import math
from math import sin, cos, pi, atan, acos
import tf
from tf import TransformListener
from tf.transformations import euler_from_quaternion
from copy import deepcopy

class Robot():

    def __init__(self):

        rospy.init_node('baseController', anonymous=True)
        self.nodename = rospy.get_name()
        rospy.loginfo("-I- %s started" % self.nodename)

        # Define publishers of velocity and angle
        self.bWheels_pub = rospy.Publisher('bWheels', Float64, queue_size=50)
        self.sWheel_vel_pub = rospy.Publisher('sVelWheel', Float64, queue_size=50)
        self.sWheel_angle_pub = rospy.Publisher('sWheel', Float64, queue_size=50)

        # define transform listener for orientation (theta)
        self.tf_listener= TransformListener()

        # orientation robot
        self.th = 0 # angle of the robot X axis in respect to map x axis
        self.phi = 0 # angle of the steering wheel in respect of the x axis of the robot

        # geometry robot
        self.L = 2.2 #From body_link to steering wheel

        # veocity
        self.xVel = 0
        self.yVel = 0
        self.thVel = 0
        self.phiVel = 0

        # motor velocities
        self.bwheels = 0
        self.swheel = 0

        self.motorCommand()

    def motorCommand(self):

        r = rospy.Rate(10)

        while not rospy.is_shutdown():

            # subscribe to twist message
            rospy.Subscriber("cmd_vel", Twist, self.twistCallback)

            #get orientation robot
            self.getTheta()

            # calculate the commands and publish them
            self.calculate_publish()

            r.sleep()

    def twistCallback(self, data):

        twistdata = deepcopy(data)

        self.xVel = twistdata.linear.x
        self.yVel = twistdata.linear.y
        self.thVel = twistdata.angular.z

        # self.xVel = cos(th)*self.bwheels
        # self.yVel = sin(th)*self.bwheels
        # self.thVel = (tan(phi)/L)*self.bwheels
        # self.phiVel = (identity)

    def getTheta(self):

        transformation = False

        while not transformation:
            try:
                t = self.tf_listener.getLatestCommonTime("/body_link", "/map")
                (trans, quaternion)= self.tf_listener.lookupTransform("/body_link", "/map", t)
                self.th = euler_from_quaternion(quaternion)[2]
                transformation = True
            except (tf.LookupException, tf.ConnectivityException):
                pass# rospy.loginfo("waiting for tf /body_link and /map to exist...")

    def calculate_publish(self):

        self.bwheels = self.xVel/cos(self.th)
        self.phi = atan((self.thVel/self.bwheels)*self.L)

        # The phi velocity, or steering angle vel, doesnt change the behaviour
        # of the car, just how fast goes to the desired angle this wheel
        # So I choose it (I guess it has to be enough quick to permit the normal movement of the car)
        # Maybe we have to check previous time step with this one, and integrate position to get velocity??

        self.phiVel = 0.5

        # Test info:
        rospy.loginfo("\n \n --------------------------------------------------------")
        rospy.loginfo("The x,y velocities are: %s and %s", self.xVel, self.yVel)
        rospy.loginfo("The angular velocity is: %s", self.thVel)
        rospy.loginfo("The velocity of the motors is: %s and the steering angle: %s", self.bwheels, self.phi)

        # Publish motor commands
        self.bWheels_pub.publish(self.bwheels)
        self.sWheel_vel_pub.publish(self.phiVel)
        self.sWheel_angle_pub.publish(self.phi)


if __name__ == '__main__':
    try:
        robot1 = Robot()

    except rospy.ROSInterruptException as e:
        rospy.loginfo('Something went terribly wrong %s', e.message)
