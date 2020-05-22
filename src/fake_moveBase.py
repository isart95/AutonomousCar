#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Vector3

if __name__ == '__main__':
    try:
        # Starts a new node
        rospy.init_node('fakeMoveBase', anonymous=True)
        moveBase_pub = rospy.Publisher('cmd_vel', Twist, queue_size=50)
        twist = Twist(Vector3(1, 0, 0), Vector3(0, 0, 1))
        r = rospy.Rate(60)
        while not rospy.is_shutdown():
            moveBase_pub.publish(twist)
            r.sleep()

    except rospy.ROSInterruptException as e:
        rospy.loginfo('Something went terribly wrong %s', e.message)
