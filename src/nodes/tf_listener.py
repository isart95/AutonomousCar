#!/usr/bin/env python

import rospy
import tf
import numpy as np
import roslib
import math
from  copy import deepcopy

#Messages
from sensor_msgs.msg import JointState
#from nav_msgs.msg import Odometry

#Maximum difference of angular velocity between the 2 back wheels
vel_dif = 0.2


def callback(data, vel_dif):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    for i in range len(data.name):
        print(data.name[i]," is: ", data.position[i])

    steering_angle = deepcopy(data.position[0])
    if (data.velocity[2]-data.velocity) < vel_dif:
        wheels_angVel = deepcopy(data.velocity[3])
    else:
        print("-----------> The 2 back wheels have different velocity")
        wheels_angVel = data.velocity[2]+data.velocity[3])/2


def listener():
    rospy.init_node('joint_listener', anonymous=True)
    rospy.Subscriber("joint_states", JointState, callback )
    rospy.spin()

if __name__ =="__main__":
    listener()
