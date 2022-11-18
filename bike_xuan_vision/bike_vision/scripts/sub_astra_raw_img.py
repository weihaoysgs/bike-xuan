#!/usr/bin/python3
# -*- coding: utf-8 -*-
import cv2
import numpy as np
import rospy
import sys
import message_filters
from sensor_msgs.msg import Image
from bike_vision.msg import road_obstacle_msg
from vino_infer import YoloXNanoOpenVINO
from params import BikeVisionParams


class BikeXuanVision:
    def __init__(self) -> None:
        self.cv_depth_img = np.zeros([256, 256, 3], dtype=np.uint8)
        self.depth_camera_k = [570.3422047415297, 0.0, 319.5,
                               0.0, 570.3422047415297, 239.5, 0.0, 0.0, 1.0]

        self.yolox_vino = YoloXNanoOpenVINO(BikeVisionParams.open_vino_model_path,
                                            BikeVisionParams.image_mean,
                                            BikeVisionParams.image_std,
                                            BikeVisionParams.net_input_img_size,
                                            BikeVisionParams.num_classes,
                                            BikeVisionParams.confidence_threshold,
                                            BikeVisionParams.nms_threshold,
                                            BikeVisionParams.class_names)
        self.init_subscriber()
        self.init_publisher()
        print("Bike XUAN Vision Init Complete!")

    def img_msg_to_cv2(self, img_msg):
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

    def depth_img_msg_to_cv2(self, depth_img_msg):
        if depth_img_msg.encoding != "16UC1":
            rospy.logerr("This depth image is not encoding with U16C1")
        dtype = np.dtype("uint16")
        dtype = dtype.newbyteorder('>' if depth_img_msg.is_bigendian else '<')
        depth_cv_img = np.ndarray(shape=(depth_img_msg.height, depth_img_msg.width, 1),
                                  # and three channels of data. Since OpenCV works with bgr natively, we don't need to
                                  # reorder the channels.
                                  dtype=dtype, buffer=depth_img_msg.data)
        if depth_img_msg.is_bigendian == (sys.byteorder == 'little'):
            depth_cv_img = depth_cv_img.byteswap().newbyteorder()
        return depth_cv_img

    def sub_astra_color_depth_img_callback(self, color_image_msg):
        cv_img = cv2.cvtColor(self.img_msg_to_cv2(
            color_image_msg), cv2.COLOR_RGB2BGR)
        top_label, top_conf, top_boxes = self.yolox_vino.infer_vino_yolo_model(
            cv_img)
        result_img = self.yolox_vino.draw_yolo_decoder_result(
            top_label, top_conf, top_boxes, cv_img)
        self.publish_detect_result_to_ros(
            result_img, top_label, top_conf, top_boxes)
        cv2.imshow("result", result_img)
        cv2.imshow("depth", self.cv_depth_img)
        cv2.waitKey(1)

    def publish_detect_result_to_ros(self, cv_img, top_label, top_conf, top_boxes):
        road_obstacle_info = road_obstacle_msg()
        road_obstacle_info.header = rospy.Header()
        road_obstacle_info.num_objects = 0

        if top_label is None or top_conf is None or top_boxes is None:
            self.pub_obstacle_info.publish(road_obstacle_info)

        for i, c in list(enumerate(top_label)):
            box = top_boxes[i]
            top, left, bottom, right = box
            top = max(0, np.floor(top).astype('int32'))
            left = max(0, np.floor(left).astype('int32'))
            bottom = min(cv_img.shape[0], np.floor(bottom).astype('int32'))
            right = min(cv_img.shape[1], np.floor(right).astype('int32'))
            middle_y = int((top + bottom) / 2)
            middle_x = int((right + left) / 2)
            real_z = self.cv_depth_img[middle_y, middle_x] * 0.001
            # prepare ros message
            road_obstacle_info.center_x[i] = middle_x
            road_obstacle_info.center_y[i] = middle_y
            road_obstacle_info.distance[i] = abs(real_z)
            road_obstacle_info.num_objects += 1
        self.pub_obstacle_info.publish(road_obstacle_info)

    def cv2_to_img_msg(self, cv_image):
        img_msg = Image()
        img_msg.height = cv_image.shape[0]
        img_msg.width = cv_image.shape[1]
        img_msg.encoding = "bgr8"
        img_msg.is_bigendian = 0
        img_msg.data = cv_image.tostring()
        img_msg.step = len(
            img_msg.data) // img_msg.height
        return img_msg

    def sub_depth_image_callback(self, depth_image_msg):
        self.cv_depth_img = self.depth_img_msg_to_cv2(depth_image_msg)

    def init_subscriber(self):
        # color_raw_img = message_filters.Subscriber(
        #     "/camera/rgb/image_raw", Image)
        # depth_img = message_filters.Subscriber(
        #     "/camera/depth/image_raw", Image)
        # img_message_fileter = message_filters.ApproximateTimeSynchronizer(
        #     [color_raw_img, depth_img], 1, 1, allow_headerless=True)
        # img_message_fileter.registerCallback(
        #     self.sub_astra_color_depth_img_callback)
        sub_color_image = rospy.Subscriber(
            "/camera/rgb/image_raw", Image, self.sub_astra_color_depth_img_callback)
        sub_depth_image = rospy.Subscriber(
            "/camera/depth/image_raw", Image, self.sub_depth_image_callback
        )

    def init_publisher(self):
        self.pub_obstacle_info = rospy.Publisher(
            "bike_obstacle_info", road_obstacle_msg, queue_size=10)


if __name__ == "__main__":
    rospy.init_node("sub_astra_raw_img_node")
    bike_xuan_vision = BikeXuanVision()
    rospy.spin()
