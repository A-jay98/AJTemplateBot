#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension

base_pub = None
shoulder_pub = None
elbow_pub = None
griper_pub = None

def transforme(msg):
    global base_pub
    global shoulder_pub
    global elbow_pub
    global griper_pub

    base_pub.publish(msg.position[0])
    shoulder_pub.publish(msg.position[1])
    elbow_pub.publish(msg.position[2])

    griper_data = Float64MultiArray()
    griper_data.layout.dim.append(MultiArrayDimension())
    griper_data.layout.dim[0].label = ''
    griper_data.layout.dim[0].size = 2
    griper_data.layout.dim[0].stride = 1
    griper_data.layout.data_offset = 0
    griper_data.data = [msg.position[3], msg.position[4]]

    griper_pub.publish(griper_data)


if __name__ == '__main__':
    try:
        base_pub = rospy.Publisher('/AJBot/base_rotation_controller/command', Float64, queue_size=10)
        shoulder_pub = rospy.Publisher('/AJBot/shoulder_rotation_controller/command', Float64, queue_size=10)
        elbow_pub = rospy.Publisher('/AJBot/elbow_rotation_controller/command', Float64, queue_size=10)
        griper_pub = rospy.Publisher('/AJBot/gripper_position_controller/command', Float64MultiArray, queue_size=10)
        
        rospy.init_node('RvizTrasformer', anonymous=True)


        rospy.Subscriber('/joint_states', JointState, transforme)
        

        rospy.spin()
    except rospy.ROSInterruptException:
        pass