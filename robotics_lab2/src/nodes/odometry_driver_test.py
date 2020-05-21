#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Vector3

if __name__ == '__main__':
    try:
        # Starts a new node
        rospy.init_node('odometry_test', anonymous=True)
        steering_pub = rospy.Publisher('control_cmd_vel', Twist, queue_size=50)
        
        twist = Twist(Vector3(1, 0, 0), Vector3(0, 0, 0.5))

        r = rospy.Rate(60)
        while not rospy.is_shutdown():
            steering_pub.publish(twist)
            r.sleep()

    except rospy.ROSInterruptException as e:
        rospy.loginfo('Something went terribly wrong %s', e.message)