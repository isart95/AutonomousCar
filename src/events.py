#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
from std_srvs.srv import Empty, EmptyRequest
import random

if __name__ == '__main__':
    try:
        random.seed(0)
        rospy.init_node('event_handler', anonymous=True)
        max_vel_pub = rospy.Publisher('max_vel', Float64, queue_size=1)
        high_vel = 1
        low_vel = 0.25
        cur_vel = high_vel
        
        while not rospy.is_shutdown():
            time_till_next_event = random.uniform(10, 20)
            rospy.sleep(time_till_next_event)

            event = random.randint(1, 4)
            
            stop_srv = rospy.ServiceProxy("stop", Empty)
            begin_srv = rospy.ServiceProxy("begin", Empty)

            if event == 1:
                stop_srv(EmptyRequest())
                rospy.sleep(5)
                begin_srv(EmptyRequest())
            elif event == 2:
                cur_vel = low_vel if cur_vel == high_vel else high_vel
                max_vel_pub.publish(Float64(cur_vel))
            elif event == 3:
                stop_srv(EmptyRequest())
                rospy.sleep(5)
                begin_srv(EmptyRequest())
            elif event == 4:
                stop_srv(EmptyRequest())
                rospy.sleep(5)
                begin_srv(EmptyRequest())

    except rospy.ROSInterruptException as e:
        rospy.loginfo('Something went terribly wrong %s', e.message)