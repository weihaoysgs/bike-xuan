#!/usr/bin/python3
# -*- coding: utf-8 -*-
import time

from openvino.runtime import Core
import torch
import cv2
import numpy as np
from params import BikeVisionParams
from bike_utils.bike_utils import decode_outputs, non_max_suppression


class YoloXNanoOpenVINO(object):
    def __init__(self, open_vino_model_path, image_mean,
                 image_std, input_img_size, num_classes,
                 confidence_threshold, nms_threshold,
                 class_names):
        super(YoloXNanoOpenVINO, self).__init__()
        self.openvino_model_path = open_vino_model_path
        self.image_mean = image_mean
        self.class_names = class_names
        self.confidence_threshold = confidence_threshold
        self.nms_threshold = nms_threshold
        self.num_classes = num_classes
        self.net_int_img_size = input_img_size
        self.image_std = image_std
        self.yolo_vino_model = self.generate_openvino_model()
        self.output_layer_ir1 = self.yolo_vino_model.output("output1")
        self.output_layer_ir2 = self.yolo_vino_model.output("output2")
        self.output_layer_ir3 = self.yolo_vino_model.output("output3")

        print("Yolo X Nano OpenVINO Init Success")

    def preprocess_cv2_image(self, cv_image):
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        resized_image = cv2.resize(rgb_image, self.net_int_img_size)
        normal_image = ((resized_image / 255) - self.image_mean) / self.image_std
        input_image = np.expand_dims(normal_image.transpose(2, 0, 1), 0)
        return input_image

    def generate_openvino_model(self):
        ie = Core()
        model = ie.read_model(self.openvino_model_path)
        compiled_model = ie.compile_model(model=model, device_name="CPU")
        return compiled_model

    def draw_yolo_decoder_result(self, top_label, top_conf, top_boxes, cv_image):
        if top_label is not None and top_conf is not None and top_boxes is not None:
            for i, c in enumerate(top_label):
                predicted_class = self.class_names[int(c)]
                box = top_boxes[i]
                score = top_conf[i]
                top, left, bottom, right = box
                top = max(0, np.floor(top).astype('int32'))
                left = max(0, np.floor(left).astype('int32'))
                bottom = min(cv_image.shape[0], np.floor(bottom).astype('int32'))
                right = min(cv_image.shape[1], np.floor(right).astype('int32'))
                middle_x = int((right + left) / 2)
                middle_y = int((top + bottom) / 2)
                label = '{} {:.2f}'.format(predicted_class, score)
                cv2.rectangle(cv_image, (left, top), (right, bottom), color=(255, 255, 0), thickness=3)
                cv2.putText(cv_image, str(label), org=(left, top), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=1,
                            color=(0, 0, 255), thickness=3)
                cv2.circle(cv_image, (middle_x, middle_y), 6, color=(0, 255, 255), thickness=-1)
            return cv_image

    def infer_vino_yolo_model(self, cv_image):
        results = self.yolo_vino_model([self.preprocess_cv2_image(cv_image=cv_image)])
        result1 = results[self.output_layer_ir1]
        result2 = results[self.output_layer_ir2]
        result3 = results[self.output_layer_ir3]
        outputs = [torch.from_numpy(result1), torch.from_numpy(result2), torch.from_numpy(result3)]
        decoder_outputs = decode_outputs(outputs, self.net_int_img_size)
        yolo_result = non_max_suppression(decoder_outputs, self.num_classes, self.net_int_img_size,
                                          (cv_image.shape[0], cv_image.shape[1]), True,
                                          conf_thres=self.confidence_threshold,
                                          nms_thres=self.nms_threshold)
        if yolo_result[0] is None:
            return None, None, None
        else:
            top_label = np.array(yolo_result[0][:, 6], dtype='int32')
            top_conf = yolo_result[0][:, 4] * yolo_result[0][:, 5]
            top_boxes = yolo_result[0][:, :4]
            return top_label, top_conf, top_boxes


if __name__ == '__main__':
    yolox_nano_openvino = YoloXNanoOpenVINO(BikeVisionParams.open_vino_model_path,
                                            BikeVisionParams.image_mean,
                                            BikeVisionParams.image_std,
                                            BikeVisionParams.net_input_img_size,
                                            BikeVisionParams.num_classes,
                                            BikeVisionParams.confidence_threshold,
                                            BikeVisionParams.nms_threshold,
                                            BikeVisionParams.class_names)
    camera = cv2.VideoCapture(0)
    if camera.isOpened():
        print("camera open success")
    while True:
        start_time = time.time()
        _, cv_img = camera.read()

        top_label, top_conf, top_boxes = yolox_nano_openvino.infer_vino_yolo_model(cv_img, True)
        result_img = yolox_nano_openvino.draw_yolo_decoder_result(top_label, top_conf, top_boxes, cv_img)
        end_time = time.time()
        print("total_time: ", end_time - start_time)
        cv2.imshow("img", result_img)
        cv2.waitKey(1)
