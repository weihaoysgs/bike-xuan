#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
import cv2
from sensor_msgs.msg import CompressedImage
import numpy as np


def sub_compressed_img_callback(msg):
    np_arr = np.fromstring(msg.data, np.uint8)
    img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    cv2.imshow("detect_result", img)
    cv2.waitKey(1)


if __name__ == "__main__":
    rospy.init_node("sub_detecet_result_comprssed_img")
    sub_compressed_img = rospy.Subscriber(
        "detect_result_img", CompressedImage, sub_compressed_img_callback)
    rospy.spin()
