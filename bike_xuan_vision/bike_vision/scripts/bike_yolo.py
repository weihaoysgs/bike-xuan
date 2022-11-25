from cgi import print_directory
import colorsys
from tracemalloc import start
import numpy as np
import torch
import torch.nn as nn
from PIL import ImageDraw, ImageFont
import PIL
import cv2
import time
from nets.yolo import YoloBody
from bike_utils.bike_utils import (cvtColor, get_classes, preprocess_input, resize_image,
                                   show_config)
from bike_utils.bike_utils import decode_outputs, non_max_suppression


class YOLO(object):
    _defaults = {
        "model_path": '/home/hll/code_space/bike_ws/src/bike_xuan_vision/bike_vision/pth/best_epoch_weights.pth',
        "classes_path": '/home/hll/code_space/bike_ws/src/bike_xuan_vision/bike_vision/pth/bike_obstacle_classes.txt',
        "input_shape": [192, 192],
        "phi": 'nano',
        "confidence": 0.5,
        "nms_iou": 0.3,
        "letterbox_image": True,
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

        self.class_names, self.num_classes = get_classes(self.classes_path)

        hsv_tuples = [(x / self.num_classes, 1., 1.)
                      for x in range(self.num_classes)]
        self.colors = list(map(lambda x: colorsys.hsv_to_rgb(*x), hsv_tuples))
        self.colors = list(
            map(lambda x: (int(x[0] * 255), int(x[1] * 255), int(x[2] * 255)), self.colors))
        self.generate()

        show_config(**self._defaults)

    def generate(self, onnx=False):
        self.net = YoloBody(self.num_classes, self.phi)
        device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.net.load_state_dict(torch.load(
            self.model_path, map_location=device))
        self.net = self.net.eval()
        print('{} model, and classes loaded.'.format(self.model_path))
        if not onnx:
            if self.cuda:
                self.net = nn.DataParallel(self.net)
                self.net = self.net.cuda()

    def detect_image(self, image, crop=False, count=False):
        image = PIL.Image.fromarray(np.uint8(cv2.cvtColor(image, cv2.COLOR_BGR2RGB)))
        image_shape = np.array(np.shape(image)[0:2])
        image = cvtColor(image)
        image_data = resize_image(
            image, (self.input_shape[1], self.input_shape[0]), self.letterbox_image)
        image_data = np.expand_dims(np.transpose(preprocess_input(
            np.array(image_data, dtype='float32')), (2, 0, 1)), 0)

        with torch.no_grad():
            images = torch.from_numpy(image_data)
            if self.cuda:
                images = images.cuda()
            outputs = self.net(images)
            outputs = decode_outputs(outputs, self.input_shape)
            results = non_max_suppression(outputs, self.num_classes, self.input_shape,
                                          image_shape, self.letterbox_image, conf_thres=self.confidence, nms_thres=self.nms_iou)

            if results[0] is None:
                return None, None, None
            else:
                top_label = np.array(results[0][:, 6], dtype='int32')
                top_conf = results[0][:, 4] * results[0][:, 5]
                top_boxes = results[0][:, :4]
                return top_label, top_conf, top_boxes

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
                cv2.rectangle(cv_image, (left, top), (right, bottom), color=(255, 255, 0), thickness=2)
                cv2.putText(cv_image, str(label), org=(left, top), fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.8,
                            color=(0, 0, 255), thickness=2)
                cv2.circle(cv_image, (middle_x, middle_y), 6, color=(0, 255, 255), thickness=-1)
            return cv_image

    def detect_heatmap(self, image, heatmap_save_path):
        import cv2
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        def sigmoid(x):
            y = 1.0 / (1.0 + np.exp(-x))
            return y
        # ---------------------------------------------------#
        #   获得输入图片的高和宽
        # ---------------------------------------------------#
        image_shape = np.array(np.shape(image)[0:2])
        # ---------------------------------------------------------#
        #   在这里将图像转换成RGB图像，防止灰度图在预测时报错。
        #   代码仅仅支持RGB图像的预测，所有其它类型的图像都会转化成RGB
        # ---------------------------------------------------------#
        image = cvtColor(image)
        # ---------------------------------------------------------#
        #   给图像增加灰条，实现不失真的resize
        #   也可以直接resize进行识别
        # ---------------------------------------------------------#
        image_data = resize_image(
            image, (self.input_shape[1], self.input_shape[0]), self.letterbox_image)
        # ---------------------------------------------------------#
        #   添加上batch_size维度
        # ---------------------------------------------------------#
        image_data = np.expand_dims(np.transpose(preprocess_input(
            np.array(image_data, dtype='float32')), (2, 0, 1)), 0)

        with torch.no_grad():
            images = torch.from_numpy(image_data)
            if self.cuda:
                images = images.cuda()
            # ---------------------------------------------------------#
            #   将图像输入网络当中进行预测！
            # ---------------------------------------------------------#
            outputs = self.net(images)

        outputs = [output.cpu().numpy() for output in outputs]
        plt.imshow(image, alpha=1)
        plt.axis('off')
        mask = np.zeros((image.size[1], image.size[0]))
        for sub_output in outputs:
            b, c, h, w = np.shape(sub_output)
            sub_output = np.transpose(sub_output, [0, 2, 3, 1])[0]
            score = np.max(
                sigmoid(sub_output[..., 5:]), -1) * sigmoid(sub_output[..., 4])
            score = cv2.resize(score, (image.size[0], image.size[1]))
            normed_score = (score * 255).astype('uint8')
            mask = np.maximum(mask, normed_score)

        plt.imshow(mask, alpha=0.5, interpolation='nearest', cmap="jet")

        plt.axis('off')
        plt.subplots_adjust(top=1, bottom=0, right=1,
                            left=0, hspace=0, wspace=0)
        plt.margins(0, 0)
        plt.savefig(heatmap_save_path, dpi=200)
        print("Save to the " + heatmap_save_path)
        plt.cla()

    def convert_to_onnx(self, simplify, model_path):
        import onnx
        self.generate(onnx=True)

        # image size(1, 3, 512, 512) BCHW
        im = torch.zeros(1, 3, *self.input_shape).to('cpu')
        input_layer_names = ["images"]
        output_layer_names = ["output1", "output2", "output3"]

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


if __name__ == "__main__":
    bike_yolo = YOLO()
    img = cv2.imread(
        "/home/hll/code_space/bike_ws/src/bike_xuan_vision/bike_vision/imgs/0.jpg")
    start_time = time.time()
    top_label, top_conf, top_boxes = bike_yolo.detect_image(img)
    result_img = bike_yolo.draw_yolo_decoder_result(top_label, top_conf, top_boxes, img)
    print("total time: ", time.time() - start_time)
    # frame = cv2.cvtColor(result_frmae, cv2.COLOR_RGB2BGR)
    cv2.imshow("frame", result_img)
    cv2.waitKey(0)
