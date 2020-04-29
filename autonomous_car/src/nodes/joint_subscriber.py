#!/usr/bin/env python

import rospy
import tf
import numpy as np
import math
from  copy import deepcopy

#Messages
from sensor_msgs.msg import JointState
#from nav_msgs.msg import Odometry

#Maximum difference of angular velocity between the 2 back wheels
vel_dif = 0.2
wheelR = 0.25


def callback(data):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    for i in range (len(data.name)):
        print(data.name[i]," is: ", data.position[i])
    #Steering angle
    #phi = deepcopy(data.position[0])
    phi = data.position[0]
    print(phi, '/n')

    #Angular velocity back wheels
    if (data.velocity[2]-data.velocity[3]) < 0.25:
        omega = deepcopy(data.velocity[3])
    else:
        print("-----------> The 2 back wheels have different velocity")
        omega = (data.velocity[2]+data.velocity[3])/2


    #Linear velocity frame origin
    linVel = omega*wheelR*3.6 #km/h
    print (linVel, '/n')



def listener():
    rospy.init_node('joint_listener', anonymous=True)
    rospy.Subscriber("joint_states", JointState, callback )
    rospy.spin()

if __name__ =="__main__":
    listener()
