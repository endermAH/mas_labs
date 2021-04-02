#!/usr/bin/python3

import rospy
import sys
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

rospy.init_node("listener")

I1 = [1,1,1]
I2 = [1,1,1]
I3 = [1,1,1]

R1 = [1,1,1]
R2 = [1,1,1]
R3 = [1,1,1]

def detection0_callback(data):
    data_i = int(data.data)
    update_i(0, data_i)
    # print("ID 0 data:" + str(data))

def detection1_callback(data):
    data_i = int(data.data)
    update_i(1, data_i)
    # print("ID 1 data:" + str(data))

def detection2_callback(data):
    data_i = int(data.data)
    update_i(2, data_i)
    # print("ID 2 data:" + str(data))

def update_i(callback_from, data_i):
    I1[callback_from] = 1 if data_i == 1 else 0
    I2[callback_from] = 1 if data_i == 0 else 0 # impostor
    I3[callback_from] = 1 if data_i == 1 else 0
    update_r(callback_from)

def update_r(callback_from):
    R1[callback_from] = round((R1[callback_from] + I1[callback_from])/2, 2)
    R2[callback_from] = round((R2[callback_from] + I2[callback_from])/2, 2)
    R3[callback_from] = round((R3[callback_from] + I3[callback_from])/2, 2)

rospy.Subscriber(f"/lidar_check_0", String, detection0_callback)
rospy.Subscriber(f"/lidar_check_1", String, detection1_callback)
rospy.Subscriber(f"/lidar_check_2", String, detection2_callback)

while True:
    stub = input()
    print("R1: " + str(R1))
    print("R2: " + str(R2))
    print("R3: " + str(R3))
