#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
import numpy as np

T01 = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 0.5],
    [0, 0, 0, 1],
])

T12 = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1],
])

T23 = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1],
])

T34 = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1],
])


def print_state():
    pos = T01.dot(T12).dot(T23).dot(T34).dot(np.array([0, 0, 0, 1]))

    rospy.loginfo("[%f\t,%f\t,%f]", pos[0], pos[1], pos[2])

def base_listener(data):
    global T12

    T12 = np.array([
        [-np.cos(data), np.sin(data), 0, 0],
        [-np.sin(data), -np.cos(data), 0, 0],
        [0, 0, 1, 0.4],
        [0, 0, 0, 1],
    ]).dot(np.array([
        [1, 0, 0, 0],
        [0, 0, -1, 0],
        [0, 1, 0, 0],
        [0, 0, 0, 1],
    ]))


def shoulder_listener(data):
    global T23

    T23 = np.array([
        [-np.sin(data), -np.cos(data), 0, 0],
        [np.cos(data), -np.sin(data), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
    ]).dot(np.array([
        [1, 0, 0, 0.8],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
    ]))

def elbow_listener(data):
    global T34

    T34 = np.array([
        [np.cos(data), -np.sin(data), 0, 0],
        [np.sin(data), np.cos(data), 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
    ]).dot(np.array([
        [1, 0, 0, 0.8],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
    ]))


def listener(msg):
    base_listener(msg.position[0])
    shoulder_listener(msg.position[1])
    elbow_listener(msg.position[2])
    print_state()


def init():
    base_listener(0.0)
    shoulder_listener(0.0)
    elbow_listener(0.0)

    rospy.init_node('Forward', anonymous=True)

    rospy.Subscriber('/joint_states', JointState, listener)
    
    rospy.spin()

if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass