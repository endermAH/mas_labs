#!/usr/bin/env python3
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from matplotlib import pyplot as plt
import rospy
import time
import math

def generate_template(template_path):
    template = cv2.imread(template_path, 0)
    return template

def recognise_picture(img, template):
    cv2.imwrite("test.png", img)
    img2 = img.copy()

    img = img2.copy()
    method = cv2.TM_CCOEFF
    res = cv2.matchTemplate(img, template, method)
    print(res)


class ImageConverter():

    def __init__(self):
        """ Initialize image recognizer """
        print("Initialized")
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera1/image_raw", Image, self.recognizer)
        rospy.init_node('command_node', anonymous=True)

    def recognizer(self,data):
        """ Recognize image """
        print("Started recognizing")
        cv_image = self.bridge.imgmsg_to_cv2(data, "mono8")
        template_path = "../templates/template.png"
        template_obj = generate_template(template_path)
        dst = recognise_picture(cv_image, template_obj)

if __name__ == ("__main__"):
    test = ImageConverter()
    test.image_sub = rospy.Subscriber("/camera1/image_raw", Image, test.recognizer)
