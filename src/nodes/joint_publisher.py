import rospy
from sensor_msgs.msg import JointState

def talker():
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)
    rospy.init_node('publisher_isart', anonymous=True)
    r = rospy.Rate(10)

    mesg = JointState()

    mesg.velocity = [0.1, 0.5, 0.5, 0.5]
    flag = True
    if flag:
        mesg.position = [0, 0, 0, 0]
        flag = False
        print('-------------')


    while not rospy.is_shutdown():
        pub.publish(mesg)
        r.sleep()

if __name__ == '__main__':
    try:
        talker()

    except rospy.ROSInterruptException:
        pass
