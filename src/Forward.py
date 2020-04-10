#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
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
    pos = T01.dot(T12).dot(T23).dot(T34).dot(np.array([0, 0, 0, 1]))[:3]

    rospy.loginfo("[%f\t,%f\t,%f]", pos[0], pos[1], pos[2])

def base_listener(data):
    global T12

    data = data.data

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

    print_state()


def shoulder_listener(data):
    global T23

    data = data.data

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

    print_state()

def elbow_listener(data):
    global T34

    data = data.data

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

    print_state()


def init():
    base_listener(Float64(0.0))
    shoulder_listener(Float64(0.0))
    elbow_listener(Float64(0.0))

    rospy.init_node('Forward', anonymous=True)

    rospy.Subscriber('/AJBot/base_rotation_controller/command', Float64, base_listener)
    rospy.Subscriber('/AJBot/shoulder_rotation_controller/command', Float64, shoulder_listener)
    rospy.Subscriber('/AJBot/elbow_rotation_controller/command', Float64, elbow_listener)
    
    rospy.spin()

if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass