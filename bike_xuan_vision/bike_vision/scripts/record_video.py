#!/usr/bin/python3
# -*- coding: utf-8 -*-
import rospy
import cv2
from sensor_msgs.msg import CompressedImage, Image
import numpy as np
import time
import sys

fourcc = cv2.VideoWriter_fourcc(*'XVID')
name = time.time()
compressed_out = cv2.VideoWriter("/home/hll/code_space/videos/"+ str(name) + "_compressed_output.avi",fourcc, 25.0, (640,480))
raw_img_out = cv2.VideoWriter("/home/hll/code_space/videos/"+ str(name) + "_raw_output.avi",fourcc, 25.0, (640,480))

def img_msg_to_cv2(img_msg):
    if img_msg.encoding != "rgb8":
        rospy.logerr(
            "This Coral detect node has been hardcoded to the 'bgr8' encoding.  Come change the code if you're "
            "actually trying to implement a new camera")
    dtype = np.dtype("uint8")  # Hardcode to 8 bits...
    dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
    image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3),
                                # and three channels of data. Since OpenCV works with bgr natively, we don't need to
                                # reorder the channels.
                                dtype=dtype, buffer=img_msg.data)
    # If the byt order is different between the message and the system.
    if img_msg.is_bigendian == (sys.byteorder == 'little'):
        image_opencv = image_opencv.byteswap().newbyteorder()
    return image_opencv

def sub_compressed_img_callback(msg):
    np_arr = np.fromstring(msg.data, np.uint8)
    img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    compressed_out.write(img)
    cv2.waitKey(1)    

i=0
def sub_astra_raw_img_callback(msg):
    cv_img = cv2.cvtColor(img_msg_to_cv2(
            msg), cv2.COLOR_RGB2BGR)
    cv2.imwrite("/home/hll/code_space/videos/images/" + str(i) + ".jpg", cv_img)
    raw_img_out.write(cv_img)
    cv2.waitKey(1) 
    
if __name__ == "__main__":
    rospy.init_node("node_sub")
    sub_compressed_img = rospy.Subscriber(
        "detect_result_img", CompressedImage, sub_compressed_img_callback)
    sub_astra_raw_img = rospy.Subscriber(
        "/camera/rgb/image_raw", Image, sub_astra_raw_img_callback)
    rospy.spin()
    compressed_out.release()
    raw_img_out.release()


    