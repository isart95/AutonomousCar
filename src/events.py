#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty, EmptyRequest
import random

if __name__ == '__main__':
    try:
        random.seed(0)
        rospy.init_node('event_handler', anonymous=True)
        
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
                stop_srv(EmptyRequest())
                rospy.sleep(5)
                begin_srv(EmptyRequest())
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