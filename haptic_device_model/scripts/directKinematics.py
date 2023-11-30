#!/usr/bin/env python3
from math import cos, sin
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, Twist

L0 = rospy.get_param("/L0", 0.040)
L1 = rospy.get_param("/L1", 0.105)
L2 = rospy.get_param("/L2", 0.190)

endEff = Point()
endEff.x = 0.0
endEff.y = 0.0
endEff.z = 0.0

velEE = Twist()
velEE.linear.x = 0.0
velEE.linear.y = 0.0
velEE.linear.z = 0.0

jnt1 = 0.0
jnt2 = 0.0

def endpoint_position(l0, l1, l2, q1):
    global endEff
    endEff.y = l1*cos(q1) + l2*cos(q1)
    endEff.z = l0 + l1*sin(q1) + l2*sin(q1)


def callbackState(state):
    global jnt1, jnt2
    # rospy.loginfo(state.position)
    jnt1 = state.position[0]
    jnt2 = state.position[1]

def stop():
    print("stopping")
    
def directKinematics():
    #Initialise and setup node
    rospy.init_node('directKinematics', anonymous=True)
    loop_rate = rospy.Rate(100)
    rospy.on_shutdown(stop)

    # Setup publishers and subscribers
    pubEE = rospy.Publisher("/object_position", Point, queue_size=10)
    rospy.Subscriber("/haptic/joint_states", JointState, callbackState)

    while not rospy.is_shutdown():
        endpoint_position(L0, L1, L2, jnt1)
        pubEE.publish(endEff)
        loop_rate.sleep()

if __name__ == '__main__':
    directKinematics()