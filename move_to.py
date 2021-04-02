#!/usr/bin/python3

import rospy
import time
import math

from geometry_msgs.msg import Twist
from gazebo_msgs.srv import GetModelState
from tf.transformations import euler_from_quaternion


class RobotController():
    """ Robot controller """
    robot_name = ""
    robot_publisher = ""
    robot_rate = ""

    def __init__(self, name):
        """ Initialize robot controller """
        print("Initialization started, please wait...")
        self.robot_name = name
        self.robot_publisher = rospy.Publisher('/part2_cmr/cmd_vel', Twist, queue_size=10)
        rospy.init_node('command_node', anonymous=True)
        self.robot_rate = rospy.Rate(10)
        time.sleep(5)
        print("Initialization finished")

    def _get_yaw_angle(self):
        """ Get robot angle"""
        model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        resp_coordinates = model_coordinates(self.robot_name, 'link')
        orientation_list = [
            resp_coordinates.pose.orientation.x,
            resp_coordinates.pose.orientation.y,
            resp_coordinates.pose.orientation.z,
            resp_coordinates.pose.orientation.w
        ]
        _, _, yaw = euler_from_quaternion(orientation_list)
        return yaw

    def _get_coords(self):
        """ Get coordinates """
        model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        resp_coordinates = model_coordinates(self.robot_name, 'link')
        return (resp_coordinates.pose.position.x, resp_coordinates.pose.position.y)

    def _move_direction(self, x, z):
        """ Move robot forward or backward or rotate it """
        cmd = Twist()
        cmd.linear.x = 0.5*x
        cmd.angular.z = 0.5*z
        self.robot_publisher.publish(cmd)

    def _rotate_toward(self, target_x, target_y):
        """ Rotate robot toward position """
        self_x, self_y = self._get_coords()
        target_angle = math.atan2(target_x-self_x, target_y-self_y)
        self_angle = self._get_yaw_angle()
        delta_angle = target_angle - self_angle

        print("Rotate to:" + str(delta_angle))

        cmd = Twist()
        cmd.angular.z = delta_angle/10
        self.robot_publisher.publish(cmd)

        for i in range(100):
            self.robot_rate.sleep()

        cmd = Twist()
        cmd.angular.z = 0
        self.robot_publisher.publish(cmd)

        time.sleep(5)

    def move_to(self, target_x, target_y):
        """ Move to position """
        self_x, self_y = self._get_coords()
        distant = math.sqrt(pow(target_x - self_x, 2)+ pow(target_y - self_y, 2))
        self._rotate_toward(target_x, target_y)

        print("Move forward: " + str(distant))

        cmd = Twist()
        cmd.linear.x = distant / 10
        self.robot_publisher.publish(cmd)

        for i in range(100):
            self.robot_rate.sleep()

        cmd = Twist()
        cmd.linear.x = 0
        self.robot_publisher.publish(cmd)


if __name__ == "__main__":
    controller = RobotController("rosbots")
    controller.move_to(10,10)
