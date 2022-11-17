import numpy as np


class BikeVisionParams(object):
    open_vino_model_path = "/home/hll/code_space/bike_ws/src/bike_xuan_vision/bike_vision/scripts/vino_export/yolox_nano.xml"
    net_input_img_size = (416, 416)
    num_classes = 2
    confidence_threshold = 0.5
    nms_threshold = 0.3
    image_std = np.array([0.229, 0.224, 0.225])
    image_mean = np.array([0.485, 0.456, 0.406])
    class_names = ["obstacle", "obstacle"]
