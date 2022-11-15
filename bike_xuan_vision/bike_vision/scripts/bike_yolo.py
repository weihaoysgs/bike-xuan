import colorsys
import os
import time
import numpy as np
import torch
import torch.nn as nn
from PIL import ImageDraw, ImageFont
from model.yolo import YoloBody
from utils.utils import (cvtColor, get_anchors, get_classes, preprocess_input,
                         resize_image, show_config)
from utils.utils_bbox import DecodeBox
import PIL


class YOLO(object):

    _defaults = {
        "model_path": "/home/hll/code_space/bike_ws/src/bike_xuan_vision/bike_vision/scripts/pth/best_epoch_weights.pth",
        "classes_path": "/home/hll/code_space/bike_ws/src/bike_xuan_vision/bike_vision/scripts/config_file/bike_xuan_obstacle_class.txt",
        "anchors_path": "/home/hll/code_space/bike_ws/src/bike_xuan_vision/bike_vision/scripts/config_file/yolo_anchors.txt",
        "anchors_mask": [[3, 4, 5], [1, 2, 3]],
        "phi": 0,
        "input_shape": [416, 416],
        "confidence": 0.9,
        "nms_iou": 0.3,
        "letterbox_image": False,
        "cuda": False,
    }

    @classmethod
    def get_defaults(cls, n):
        if n in cls._defaults:
            return cls._defaults[n]
        else:
            return "Unrecognized attribute name '" + n + "'"

    def __init__(self, **kwargs):
        self.__dict__.update(self._defaults)
        for name, value in kwargs.items():
            setattr(self, name, value)
            self._defaults[name] = value

        # 获得种类和先验框的数量
        self.class_names, self.num_classes = get_classes(self.classes_path)
        self.anchors, self.num_anchors = get_anchors(self.anchors_path)
        self.bbox_util = DecodeBox(self.anchors, self.num_classes, (self.input_shape[0], self.input_shape[1]),
                                   self.anchors_mask)
        hsv_tuples = [(x / self.num_classes, 1., 1.)
                      for x in range(self.num_classes)]
        self.colors = list(map(lambda x: colorsys.hsv_to_rgb(*x), hsv_tuples))
        self.colors = list(
            map(lambda x: (int(x[0] * 255), int(x[1] * 255), int(x[2] * 255)), self.colors))
        self.generate()
        show_config(**self._defaults)

    # 生成模型
    def generate(self, onnx=False):
        # 建立yolo模型，载入yolo模型的权重
        self.net = YoloBody(self.anchors_mask, self.num_classes, self.phi)
        device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.net.load_state_dict(torch.load(
            self.model_path, map_location=device))
        self.net = self.net.eval()
        print('{} model, anchors, and classes loaded.'.format(self.model_path))
        if not onnx:
            if self.cuda:
                self.net = nn.DataParallel(self.net)
                self.net = self.net.cuda()

    #   检测图片
    def detect_image(self, image, crop=False, count=False):
        image_shape = np.array(np.shape(image)[0:2])  # get image (h,w)
        # 在这里将图像转换成RGB图像，防止灰度图在预测时报错。
        # 代码仅仅支持RGB图像的预测，所有其它类型的图像都会转化成RGB
        image = cvtColor(image)
        # 给图像增加灰条，实现不失真的resize
        # 可以直接resize进行识别
        image_data = resize_image(
            image, (self.input_shape[1], self.input_shape[0]), self.letterbox_image)
        # 添加上batch_size维度
        image_data = np.expand_dims(np.transpose(preprocess_input(
            np.array(image_data, dtype='float32')), (2, 0, 1)), 0)

        with torch.no_grad():
            images = torch.from_numpy(image_data)
            if self.cuda:
                images = images.cuda()
            # 将图像输入网络当中进行预测！
            outputs = self.net(images)
            outputs = self.bbox_util.decode_box(outputs)
            # 将预测框进行堆叠，然后进行非极大抑制
            results = self.bbox_util.non_max_suppression(torch.cat(outputs, 1), self.num_classes, self.input_shape,
                                                         image_shape, self.letterbox_image, conf_thres=self.confidence,
                                                         nms_thres=self.nms_iou)

            if results[0] is None:
                return None, None, None

            top_label = np.array(results[0][:, 6], dtype='int32')
            top_conf = results[0][:, 4] * results[0][:, 5]
            top_boxes = results[0][:, :4]
            return top_label, top_conf, top_boxes

    def convert_to_onnx(self, simplify, model_path):
        import onnx
        self.generate(onnx=True)

        # image size(1, 3, 512, 512) BCHW
        im = torch.zeros(1, 3, *self.input_shape).to('cpu')
        input_layer_names = ["images"]
        output_layer_names = ["output"]

        # Export the model
        print(f'Starting export with onnx {onnx.__version__}.')
        torch.onnx.export(self.net,
                          im,
                          f=model_path,
                          verbose=False,
                          opset_version=12,
                          training=torch.onnx.TrainingMode.EVAL,
                          do_constant_folding=True,
                          input_names=input_layer_names,
                          output_names=output_layer_names,
                          dynamic_axes=None)

        # Checks
        model_onnx = onnx.load(model_path)  # load onnx model
        onnx.checker.check_model(model_onnx)  # check onnx model

        # Simplify onnx
        if simplify:
            import onnxsim
            print(f'Simplifying with onnx-simplifier {onnxsim.__version__}.')
            model_onnx, check = onnxsim.simplify(
                model_onnx,
                dynamic_input_shape=False,
                input_shapes=None)
            assert check, 'assert check failed'
            onnx.save(model_onnx, model_path)

        print('Onnx model save as {}'.format(model_path))


if __name__ == '__main__':
    yolo = YOLO()
    # img = "/home/wangzihanggg/code_space/bike_vision_ws/src/bike_vision/scripts/img/2.jpg"
    # image = PIL.Image.open(img)
    # r_image = yolo.detect_image(image, crop=False, count=False)
    # # r_image.show()
