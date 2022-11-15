import PIL.Image
from openvino.runtime import Core
import cv2
import matplotlib.pyplot as plt
import numpy as np
import sys
import torch
from nets.yolo import YoloBody
from utils.utils_bbox import decode_outputs, non_max_suppression
from PIL import ImageDraw, ImageFont
import time
class_names = ['person', 'bicycle', 'car', 'motorbike', 'aeroplane', 'bus', 'train',
               'truck', 'boat', 'traffic light', 'fire hydrant', 'stop sign', 'parking meter',
               'bench', 'bird', 'cat', 'dog', 'horse', 'sheep', 'cow', 'elephant', 'bear', 'zebra', 'giraffe',
               'backpack', 'umbrella', 'handbag', 'tie', 'suitcase', 'frisbee', 'skis', 'snowboard', 'sports ball',
               'kite', 'baseball bat', 'baseball glove', 'skateboard', 'surfboard', 'tennis racket', 'bottle',
               'wine glass', 'cup', 'fork', 'knife', 'spoon', 'bowl', 'banana', 'apple', 'sandwich', 'orange',
               'broccoli', 'carrot', 'hot dog', 'pizza', 'donut', 'cake', 'chair', 'sofa', 'pottedplant', 'bed',
               'diningtable', 'toilet', 'tvmonitor', 'laptop', 'mouse', 'remote', 'keyboard', 'cell phone', 'microwave',
               'oven', 'toaster', 'sink', 'refrigerator', 'book', 'clock', 'vase', 'scissors', 'teddy bear',
               'hair drier', 'toothbrush']

colors = [(255, 0, 0), (255, 19, 0), (255, 38, 0), (255, 57, 0), (255, 76, 0), (255, 95, 0), (255, 114, 0),
          (255, 133, 0), (255, 153, 0), (255, 172, 0), (255, 191, 0), (255, 210, 0), (255, 229, 0), (255, 248, 0),
          (242, 255, 0), (223, 255, 0), (203, 255, 0), (184, 255, 0), (165, 255, 0), (146, 255, 0), (127, 255, 0),
          (108, 255, 0), (89, 255, 0), (70, 255, 0), (51, 255, 0), (31, 255, 0), (12, 255, 0), (0, 255, 6),
          (0, 255, 25), (0, 255, 44), (0, 255, 63), (0, 255, 82), (0, 255, 102), (0, 255, 121), (0, 255, 140),
          (0, 255, 159), (0, 255, 178), (0, 255, 197), (0, 255, 216), (0, 255, 235), (0, 255, 255), (0, 235, 255),
          (0, 216, 255), (0, 197, 255), (0, 178, 255), (0, 159, 255), (0, 140, 255), (0, 121, 255), (0, 102, 255),
          (0, 82, 255), (0, 63, 255), (0, 44, 255), (0, 25, 255), (0, 6, 255), (12, 0, 255), (31, 0, 255), (50, 0, 255),
          (70, 0, 255), (89, 0, 255), (108, 0, 255), (127, 0, 255), (146, 0, 255), (165, 0, 255), (184, 0, 255),
          (204, 0, 255), (223, 0, 255), (242, 0, 255), (255, 0, 248), (255, 0, 229), (255, 0, 210), (255, 0, 191),
          (255, 0, 172), (255, 0, 152), (255, 0, 133), (255, 0, 114), (255, 0, 95), (255, 0, 76), (255, 0, 57),
          (255, 0, 38), (255, 0, 19)]


def generate_yolo_nano_model(model_path, num_classes, phi):
    net = YoloBody(num_classes, phi)
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    net.load_state_dict(torch.load(model_path, map_location=device))
    net = net.eval()
    return net


if __name__ == '__main__':
    yolo_nano_model = generate_yolo_nano_model("./model_data/yolox_nano.pth", 80, "nano")
    ie = Core()
    model = ie.read_model(model="vino_export/yolox_nano.xml")
    compiled_model = ie.compile_model(model=model, device_name="CPU")
    input_layer_ir = compiled_model.input(0)
    output_layer_ir1 = compiled_model.output("output1")
    output_layer_ir2 = compiled_model.output("output2")
    output_layer_ir3 = compiled_model.output("output3")
    image = cv2.imread("./img/street.jpg")
    drawed_image = PIL.Image.open("./img/street.jpg")
    drawed_image = drawed_image.convert('RGB')
    rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image_h, image_w, _ = rgb_image.shape
    N, C, H, W = input_layer_ir.shape
    resized_image = cv2.resize(image, (W, H))
    resized_image = ((resized_image / 255) - np.array([0.485, 0.456, 0.406])) / np.array([0.229, 0.224, 0.225])
    input_image = np.expand_dims(
        resized_image.transpose(2, 0, 1), 0
    )
    start_time = time.time()
    results = compiled_model([input_image])
    end_time = time.time()
    print(("total time: ", end_time - start_time))
    result1 = results[output_layer_ir1]
    result2 = results[output_layer_ir2]
    result3 = results[output_layer_ir3]
    outputs = [torch.from_numpy(result1), torch.from_numpy(result2), torch.from_numpy(result3)]
    outputs = decode_outputs(outputs, (416, 416))
    results = non_max_suppression(outputs, 80, (416, 416),
                                  (1330, 1330), True, conf_thres=0.5, nms_thres=0.3)

    if results[0] is None:
        print("no result")
    top_label = np.array(results[0][:, 6], dtype='int32')
    top_conf = results[0][:, 4] * results[0][:, 5]
    top_boxes = results[0][:, :4]
    font = ImageFont.truetype(font='model_data/simhei.ttf', size=np.floor(3e-2 * drawed_image.size[1] + 0.5).astype('int32'))
    thickness = int(max((drawed_image.size[0] + drawed_image.size[1]) // np.mean((416, 416)), 1))
    # ---------------------------------------------------------#
    #   图像绘制
    # ---------------------------------------------------------#
    for i, c in list(enumerate(top_label)):
        predicted_class = class_names[int(c)]
        box = top_boxes[i]
        score = top_conf[i]

        top, left, bottom, right = box

        top = max(0, np.floor(top).astype('int32'))
        left = max(0, np.floor(left).astype('int32'))
        bottom = min(drawed_image.size[1], np.floor(bottom).astype('int32'))
        right = min(drawed_image.size[0], np.floor(right).astype('int32'))

        label = '{} {:.2f}'.format(predicted_class, score)
        draw = ImageDraw.Draw(drawed_image)
        label_size = draw.textsize(label, font)
        label = label.encode('utf-8')
        print(label, top, left, bottom, right)

        if top - label_size[1] >= 0:
            text_origin = np.array([left, top - label_size[1]])
        else:
            text_origin = np.array([left, top + 1])

        for i in range(thickness):
            draw.rectangle([left + i, top + i, right - i, bottom - i], outline=colors[c])
        draw.rectangle([tuple(text_origin), tuple(text_origin + label_size)], fill=colors[c])
        draw.text(text_origin, str(label, 'UTF-8'), fill=(0, 0, 0), font=font)
        del draw
    drawed_image.show()
