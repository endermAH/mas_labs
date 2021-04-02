#!/usr/bin/python3

import rospy
from geometry_msgs.msg import Twist


pub = rospy.Publisher('/part2_cmr/cmd_vel', Twist, queue_size=10)
rospy.init_node('command_node', anonymous=True)
rate = rospy.Rate(10)  # 10hz
cmd = Twist()

while(True):
    key = input()
    if (key=='w'):
        cmd.linear.x = 0.5

    if (key=='d'):
        cmd.angular.z = -0.5

    if (key=='s'):
        cmd.linear.x = -0.5

    if (key=='a'):
        cmd.angular.z = 0.5

    if (key=='x'):
        cmd.angular.z = 0
        cmd.linear.x = 0
        pub.publish(cmd)
        break

    pub.publish(cmd)
