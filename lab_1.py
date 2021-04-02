#!/usr/bin/python3

import rospy


from geometry_msgs.msg import Twist

pub = rospy.Publisher('/part2_cmr/cmd_vel', Twist, queue_size=10)
rospy.init_node('command_node', anonymous=True)
rate = rospy.Rate(10)  # 10hz

def move(x, z):
  cmd = Twist()
  cmd.linear.x = 0.5*x
  cmd.angular.z = 0.5*z
  pub.publish(cmd)

while(True):
  direction = input()
  if (direction == "w"):
    move(1,0)
  if (direction == "a"):
    move(0,1)
  if (direction == "s"):
    move(-1,0)
  if (direction == "d"):
    move(0,-1)
