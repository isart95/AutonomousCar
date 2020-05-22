#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from copy import deepcopy
from geometry_msgs.msg import Float64

class EnergyNode:

    def __init__(self):
        self.energy_pub = rospy.Publisher('energy', Float64, queue_size=50 )
        self.prev_time = 0
        self.prev_vel = 0
        self.acceleration = 0
        self.new_time = 0
        self.new_vel = 0

        self.initial_energy = 1000
        self.total_energy = initial_energy

         # constants:
        self.mass = 855
        self.g = 9.81
        self.air_dens = 1.225
        self.A = 1.92  # cross-section car, ignoring wheels
        self.Cd = 0.6 # aerodynamic coefficient (from a Hummer H2)

        rospy.Subscriber('odom'. Odometry, self.callback)

    def calculate(self):
        self.odom = deepcopy(msg)
        self.time_step = self.new_time - elf.prev_time
        self.new_vel = sqrt(odom.velocity.x**2 + odom.velocity.y**2)
        self.acceleration = (self.new_vel-self.prev_vel)/(self.time_step)

        self.energy()

        # Redefining variables:
        self.prev_time = deepcopy(self.new_time)
        self.prev_vel = deepcopy(self.new_vel)


    def energy(self):
        # integrating drag_energy formula = integ(0.5 * Cd * airdens * A * v³ * t
        v1 = self.prev_vel
        t1 = self.prev_time
        v2 = self.new_vel
        t2 = self.new_time
        a = self.acceleration

        drag_energy = 0.5 * self.Cd * self.air_dens * self.A * \
                    ((v1**3 * (t2**2-t1**2)/2) + (3*v1**2*a*(t2**3-t1**3)/3) \
                    (3*v1*a**2*(t2**4-t1**4)/4) + (a**3*(t2**5-t1**5)/5))

        acc_energy = self.mass * self.acceleration * ((v1*(t2**2-t1**2)/2) + \
                    (a * (t2**3-t1**3)/3))

        self.total_energy -= (drag_energy + acc_energy)

        self.energy_pub.publish(self.total_energy)
        percent = self.initial_energy*100/self.total_energy

        print ('At time %s the energy is at the %s percent (%s out of %s Joules)',
                self.new_time, percent, self.total_energy, self.initial_energy)

    def callback(self, msg):
        self.odom = deepcopy(msg)
        self.new_time = rospy.Time.now()


if __name__ == '__main__':
    try:
        # Starts a new node
        rospy.init_node('energy_management', anonymous=True)
        enMan = EnergyNode()
        rate = rospy.Rate(60)
        while not rospy.is_shutdown():
            enMan.calculate()
            rate.sleep()

    except rospy.ROSInterruptException as e:
        rospy.loginfo('Something went terribly wrong %s', e.message)
