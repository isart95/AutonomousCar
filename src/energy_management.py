#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from copy import deepcopy
from std_msgs.msg import Float64
from math import sqrt

class EnergyNode:

    def __init__(self):
        self.energy_pub = rospy.Publisher('energy', Float64, queue_size=50 )
        self.prev_time = rospy.Time.now()
        self.prev_vel = 0
        self.acceleration = 0
        self.new_time = rospy.Time.now()
        self.new_vel = 0

        # LiFePO4 batteries have 400000 j/kg. Lets suppose 10 kg of battery
        self.initial_energy = 400000 * 10
        self.total_energy = deepcopy(self.initial_energy)

         # constants:
        self.mass = 855
        self.g = 9.81
        self.air_dens = 1.225 # kg/m**3
        self.cross_section = 1.92  # cross-section car, ignoring wheels
        self.cd = 0.6 # aerodynamic coefficient (from a Hummer H2)
        self.odom = Odometry()

        rospy.Subscriber("odom", Odometry, self.callback)

    def calculate(self):
        self.time_step = self.new_time.secs - self.prev_time.secs + \
                        10**(-9) * (self.new_time.nsecs - self.prev_time.nsecs)

        self.new_vel = sqrt(self.odom.twist.twist.linear.x**2 + self.odom.twist.twist.linear.y**2)

        if self.time_step > 0:
            self.acceleration = (self.new_vel-self.prev_vel)/(self.time_step)
        else:
            self.acceleration = 0

        self.energy()

        # Redefining variables:
        self.prev_time = deepcopy(self.new_time)
        self.prev_vel = deepcopy(self.new_vel)


    def energy(self):
        # integrating drag_energy formula = integ(0.5 * Cd * airdens * A * v**3 * t
        v1 = self.prev_vel
        t1 = self.prev_time.secs + self.prev_time.nsecs*10**(-9)
        v2 = self.new_vel
        t2 = self.new_time.secs + self.prev_time.nsecs*10**(-9)
        a = self.acceleration
        v_avg = (v2-v1)/2

        drag_energy = 0.5 * self.cd * self.air_dens * self.cross_section * \
                    v_avg**3 * self.time_step

        acc_energy = 0.5 * self.mass *(v2**2-v1**2)

        if v_avg > 0:
            floor_friction = 300
        else:
            floor_friction = 0

        motor_lost = 10

        print '====== v1: ', v1
        print '====== v2: ', v2
        print '====== t1: ', t1
        print '====== t2: ', t2
        print '====== drg: ', drag_energy
        print '====== acc_energy: ', acc_energy

        self.total_energy -= (drag_energy + acc_energy)

        self.energy_pub.publish(self.total_energy)
        percent = self.initial_energy*100/self.total_energy

        print('At time %d the energy is at the %d percent (%d out of %d kJ)' \
                % (self.new_time.secs, percent, self.total_energy/1000, self.initial_energy/1000))

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
