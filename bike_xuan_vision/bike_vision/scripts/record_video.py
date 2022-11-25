#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
import cv2
from sensor_msgs.msg import CompressedImage
import numpy as np
import time

fourcc = cv2.VideoWriter_fourcc(*'XVID')
name = time.time()
out = cv2.VideoWriter("/home/hll/code_space/videos/"+ str(name) + "_output.avi",fourcc, 25.0, (640,480))
 
def sub_compressed_img_callback(msg):
    np_arr = np.fromstring(msg.data, np.uint8)
    img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    out.write(img)
    cv2.waitKey(1)    
    
    
    
if __name__ == "__main__":
    rospy.init_node("node_sub")
    sub_compressed_img = rospy.Subscriber(
        "detect_result_img", CompressedImage, sub_compressed_img_callback)
    rospy.spin()
    out.release()


    