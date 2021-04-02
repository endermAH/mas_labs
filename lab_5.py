#!/usr/bin/python3

import rospy
import time

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class RobotController:
    """ Robot controller class """

    def __init__(self):
        """ Class initialization """
        self.pub = rospy.Publisher('/part2_cmr/cmd_vel', Twist, queue_size=10)
        rospy.init_node('command_node', anonymous=True)
        self.rate = rospy.Rate(10)
        self.laser = rospy.Subscriber("/bot_0/laser/scan", LaserScan, self.laser_callback)
        self.is_rotating = False

    def laser_callback(self, data):
        """ Laser callback method """
        print("Is rotating: " + str(self.is_rotating))
        if self.is_rotating == False:
            min_range = 1.6
            for range in data.ranges[300:402]:
                if range < min_range:
                    min_range = range

            if min_range < 1.5:
                print("Obstacle found - rotating")
                self.move(0,0.5)
            else:
                self.move(1,0)

    def rotate_random(self):
        self.is_rotating = True
        self.move(0,0.5)
        time.sleep(1)
        self.move(0,0)
        self.is_rotating = False

    def move(self, x, z):
        """ Robot movement """
        cmd = Twist()
        cmd.linear.x = 0.5*x
        cmd.angular.z = 0.5*z
        self.pub.publish(cmd)

if __name__ == "__main__":
    controller = RobotController()
    a = input()
    controller.move(0,0)
