#!/usr/bin/env python3
from math import cos, sin
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, Twist

L0 = rospy.get_param("/L0", 0.040)
L1 = rospy.get_param("/L1", 0.105)
L2 = rospy.get_param("/L2", 0.190)

totalEffort = Point()
totalEffort.x = 0.0
totalEffort.y = 0.0
totalEffort.z = 0.0

jnt1 = 0.0
jnt2 = 0.0

tQ1 = 0.0
tQ2 = 0.0

fX = 0.0
fY = 0.0
fZ = 0.0

def endpoint_position(l0, l1, l2, q1):
    global endEff
    endEff.y = l1*cos(q1) + l2*cos(q1)
    endEff.z = l0 + l1*sin(q1) + l2*sin(q1)

def jntTorque(l1, l2, q1, fy, fz):
    global tQ1
    totalEffort.z = fz(l1*cos(q1)+l2*cos(q1)) - fy(l1*sin(q1)+l2*sin(q1))
    totalEffort.y = 0.0


def callbackState(state):
    global jnt1, jnt2
    # rospy.loginfo(state.position)
    jnt1 = state.position[0]
    jnt2 = state.position[1]

def callbackForce(state):
    global fX, fY, fZ
    fX = state.effort[0]
    fY = state.effort[1]
    fZ = state.effort[2]

def stop():
    print("stopping")
    
def torqueCalc():
    #Initialise and setup node
    rospy.init_node('torqueCalc', anonymous=True)
    loop_rate = rospy.Rate(100)
    rospy.on_shutdown(stop)

    # Setup publishers and subscribers
    pubHE = rospy.Publisher("/haptic_effort", Point, queue_size=10)
    rospy.Subscriber("/ee_force", JointState, callbackForce)
    rospy.Subscriber("/haptic/joint_states", JointState, callbackState)

    while not rospy.is_shutdown():
        jntTorque(L1, L2, jnt1, fY, fZ)
        pubHE.publish(totalEffort)
        loop_rate.sleep()

if __name__ == '__main__':
    torqueCalc()