#!/usr/bin/env python
import math
from math import sin, cos, tan, pi
import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D, Point, Pose, PoseWithCovarianceStamped, Quaternion, Twist, Vector3
from std_msgs.msg import String, Header
from sensor_msgs.msg import JointState
import numpy as np
from copy import deepcopy


def clamp(value, low, high):
    return max(min(value, high), low)

class OdometryNode:
    
    def __init__(self):
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=1)
        self.steering_pub = rospy.Publisher('steering_state', JointState, queue_size=1)
        self.odom_broadcaster = tf.TransformBroadcaster()

        rospy.Subscriber('control_cmd_vel', Twist, self.update_twists)
        rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.update_init_pose)
        
        self.quat = [0.0, 0.0, 0.0, 0.0]
        self.pose = Pose2D()
        self.twist = Twist()
        self.steering_angle = 0
        self.control_vel = Twist()
        self.prev_time = rospy.Time.now()
        # geometry robot
        self.L = 2.2 #From body_link to steering wheel
    
    def update_init_pose(self, msg):
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y
        
        self.quat[0] = msg.pose.pose.orientation.x
        self.quat[1] = msg.pose.pose.orientation.y
        self.quat[2] = msg.pose.pose.orientation.z
        self.quat[3] = msg.pose.pose.orientation.w
        
        theta = tf.transformations.euler_from_quaternion([self.quat[0], self.quat[1], self.quat[2], self.quat[3]])
        self.pose.theta = theta[2]

    def update_twists(self, msg):
        self.control_vel = deepcopy(msg)
        self.twist = deepcopy(msg)
        self.twist.angular.z = self.control_vel.linear.x * tan(self.steering_angle) / self.L 


    def integrate(self):
        current_time = rospy.Time.now()
        dt = (current_time - self.prev_time).to_sec()
        self.prev_time = current_time

        # Compute odometry in a typical way given the velocities of the robot
        dx = self.twist.linear.x * cos(self.pose.theta) * dt
        dy = self.twist.linear.x * sin(self.pose.theta) * dt
        dtheta = self.twist.angular.z * dt
        dphi = self.control_vel.angular.z * dt

        self.pose.x += dx
        self.pose.y += dy
        self.pose.theta += dtheta
        self.steering_angle = clamp(self.steering_angle + dphi, -pi/4, pi/4)
        self.twist.angular.z = self.control_vel.linear.x * tan(self.steering_angle) / self.L

        # Create quaternion from theta
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.pose.theta)

        # first, we'll publish the transform over tf (from base link frame to odom (map-fix)frame)
        self.odom_broadcaster.sendTransform(
            (self.pose.x, self.pose.y, 0),
            odom_quat,
            current_time,
            "base_link",
            "odom"
        )

        # add noise to tf for localisation
        noise_x = self.pose.x + np.random.normal(0.0, 0.2)
        noise_y = self.pose.y + np.random.normal(0.0, 0.2)
        noise_theta = self.pose.theta + np.random.normal(0.0, 0.2)
        noise_phi = self.steering_angle + np.random.normal(0.0, 0.2)
        odom_noise_quat = tf.transformations.quaternion_from_euler(0, 0, noise_theta)
        self.odom_broadcaster.sendTransform(
            (noise_x, noise_y, 0),
            odom_noise_quat,
            current_time,
            "base_footprint",
            "odom"
        )

        # map-odom transform
        self.odom_broadcaster.sendTransform(
            (0, 0, 0),
            (0, 0, 0, 1),
            current_time,
            "odom",
            "map"
        )

        # publish steering_state
        steering_state = JointState()
        steering_state.header = Header()
        steering_state.name = ['base_to_steering_point']
        steering_state.position = [noise_phi]
        steering_state.velocity = []
        steering_state.effort = []
        self.steering_pub.publish(steering_state)

        # next, we'll publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        # set the position
        odom.pose.pose = Pose(Point(noise_x, noise_y, 0.), Quaternion(*odom_noise_quat))

        # set the velocity
        odom.child_frame_id = "base_footprint"
        odom.twist.twist = self.twist

        # publish the message
        self.odom_pub.publish(odom)


if __name__ == '__main__':
    try:
        # Starts a new node
        rospy.init_node('odometry_publisher', anonymous=True)
        odom = OdometryNode()
        rate = rospy.Rate(60)
        while not rospy.is_shutdown():
            odom.integrate()
            rate.sleep()

    except rospy.ROSInterruptException as e:
        rospy.loginfo('Something went terribly wrong %s', e.message)