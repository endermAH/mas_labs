#!/usr/bin/python3

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import rospy
import argparse
import cv2 as cv
import numpy as np
import time

CLASSES = [
    "background", "aeroplane", "bicycle", "bird", "boat", "bottle",
    "bus", "car", "cat", "chair", "cow", "diningtable", "dog", "horse",
    "motorbike", "person", "pottedplant", "sheep", "sofa", "train", "tvmonitor"
]

CAFFE_MODEL = "MobileNet/caffemodel"
CAFFE_PROTO = "MobileNet/prototxt"

COLORS = np.random.uniform(0, 255, size=(len(CLASSES), 3))
net = cv.dnn.readNetFromCaffe(CAFFE_PROTO, CAFFE_MODEL)


class RobotController:
    """ Robot controller class """

    def __init__(self):
        """ Initialize image recognizer """
        print("Initializing...")
        rospy.init_node('command_node', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera1/image_raw", Image, self.test_callback)

    def test_callback(self, data):
        """ Image detection callback """
        print("Start detecting")
        frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        cv.imwrite("iseecat.png", frame)
        (h, w) = frame.shape[:2]
        blob = cv.dnn.blobFromImage(cv.resize(frame, (400, 400)), 0.007843, (400, 400), 127)
        net.setInput(blob)
        detections = net.forward()
        for i in np.arange(0, detections.shape[2]):
            confidence = detections[0, 0, i, 2]
            if confidence > 0.30:
                idx = int(detections[0, 0, i, 1])
                box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                (startX, startY, endX, endY) = box.astype("int")
                label = "{}: {:.2f}%".format(CLASSES[idx], confidence*100)
                cv.rectangle(frame, (startX, startY), (endX, endY), COLORS[idx], 2)
                y = startY - 15 if startY-15 > 15 else startY + 15
                print(label)


if __name__ == "__main__":
    controller = RobotController()
    time.sleep(5)
    # controller.test_callback()
    # controller.image_sub = rospy.Subscriber("/camera1/image_raw", Image, controller.test_callback)
