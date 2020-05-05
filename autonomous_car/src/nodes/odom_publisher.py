#!/usr/bin/env python
import math
from math import sin, cos, pi
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D, Point, Pose, Quaternion, Twist, Vector3


class OdometryNode:
    
    def __init__(self):
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=50)
        self.odom_broadcaster = tf.TransformBroadcaster()
        
        rospy.Subscriber('steering_cmd_vel', Twist, self.integrate)

        self.pose = Pose2D(0, 0, 0)
        self.twist = Twist()
        self.steering_angle = 0
        self.steering_twist = Twist()
        self.prev_time = rospy.Time.now()
    

    def integrate(self, msg):
        self.steering_twist = msg
        self.twist = msg
        
        current_time = rospy.Time.now()
        dt = (current_time - self.prev_time).to_sec()
        self.prev_time = current_time


        # Compute odometry in a typical way given the velocities of the robot
        dx = self.twist.linear.x * cos(self.pose.theta) * dt
        dy = self.twist.linear.x * sin(self.pose.theta) * dt
        dtheta = self.twist.angular.z * dt

        self.pose.x += dx
        self.pose.y += dy
        self.pose.theta += dtheta

        # Create quaternion from theta
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.pose.theta)

        # first, we'll publish the transform over tf (from base link frame to odom (map-fix)frame)
        self.odom_broadcaster.sendTransform(
            (self.pose.x, self.pose.y, 0.),
            odom_quat,
            current_time,
            "body_link",
            "odom"
        )

        # next, we'll publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        # set the position
        odom.pose.pose = Pose(Point(self.pose.x, self.pose.y, 0.), Quaternion(*odom_quat))

        # set the velocity
        odom.child_frame_id = "body_link"
        odom.twist.twist = self.twist

        # publish the message
        self.odom_pub.publish(odom)


if __name__ == '__main__':
    try:
        # Starts a new node
        rospy.init_node('odometry_publisher', anonymous=True)
        odom = OdometryNode()
        while not rospy.is_shutdown():
            rospy.spin()
    except rospy.ROSInterruptException as e:
        rospy.loginfo('Something went terribly wrong %s', e.message)