#!/usr/bin/env python

import rospy
import tf
import geometry_msgs.msg
from copy import deepcopy



def listener(pose):
    rospy.init_node('robot_tf_listener')

    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            #---> We need to create the world frame
            #Converts from the body link to the world frame
            (trans,rot) = listener.lookupTransform('/world', '/body_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        #position and quaterion of the robot from the world frame
        #Do we need a rospy.spin???????????
        p = deepcopy(trans[0])
        q = deepcopy(trans[1])

        rate.sleep()

if __name__ =="__main__":
    listener()
