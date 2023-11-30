#!/usr/bin/env python3
from math import cos, sin
import rospy
from std_msgs.msg import Float64

jnt1 = 0.0
jnt2 = 0.0
    

def stop():
    print("stopping")
    
def motionTest():
    #Initialise and setup node
    rospy.init_node('motionTest', anonymous=True)
    loop_rate = rospy.Rate(100)
    rospy.on_shutdown(stop)

    # Setup publishers and subscribers
    pubJ1= rospy.Publisher("/haptic_Jnt1_controller/command", Float64, queue_size=10)
    pubJ2= rospy.Publisher("/haptic_Jnt2_controller/command", Float64, queue_size=10)

    while not rospy.is_shutdown():
        pubJ1.publish(10.0)
        rospy.sleep(5.)
        # pubJ2.publish(1.0)
        rospy.sleep(5.)
        pubJ1.publish(-10.0)
        rospy.sleep(5.)
        # pubJ2.publish(0.0)
        rospy.sleep(5.)
        loop_rate.sleep()

if __name__ == '__main__':
    motionTest()