#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

base_pub = None
shoulder_pub = None
elbow_pub = None

def transforme(msg):
    global base_pub
    global shoulder_pub
    global elbow_pub

    base_pub.publish(msg.position[0])
    shoulder_pub.publish(msg.position[1])
    elbow_pub.publish(msg.position[2])

if __name__ == '__main__':
    try:
        base_pub = rospy.Publisher('/AJBot/base_rotation_controller/command', Float64, queue_size=10)
        shoulder_pub = rospy.Publisher('/AJBot/shoulder_rotation_controller/command', Float64, queue_size=10)
        elbow_pub = rospy.Publisher('/AJBot/elbow_rotation_controller/command', Float64, queue_size=10)

        
        rospy.init_node('RvizTrasformer', anonymous=True)


        rospy.Subscriber('/joint_states', JointState, transforme)
        

        rospy.spin()
    except rospy.ROSInterruptException:
        pass