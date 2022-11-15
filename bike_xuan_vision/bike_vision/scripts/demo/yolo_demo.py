# -*- coding: utf-8 -*-
import sys
sys.path.append("..")
from model.yolo import YoloBody
from utils.utils import get_classes
from model.yolo_training import weights_init
import torch
import numpy as np

classes_path = "../config_file/yolo_anchors.txt"
anchors_mask = [[3, 4, 5], [1, 2, 3]]
class_names, num_classes = get_classes(classes_path)
model_path = "../pth/best_epoch_weights.pth"

if __name__ == '__main__':
    model = YoloBody(anchors_mask=anchors_mask, num_classes=num_classes)
    weights_init(model)
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    local_rank = 0
    if model_path != '':

        # ------------------------------------------------------#
        #   根据预训练权重的Key和模型的Key进行加载
        # ------------------------------------------------------#
        device = torch.device('cpu')
        model_dict = model.state_dict()

        pretrained_dict = torch.load(model_path, map_location=device)
        load_key, no_load_key, temp_dict = [], [], {}
        for k, v in pretrained_dict.items():
            if k in model_dict.keys() and np.shape(model_dict[k]) == np.shape(v):
                temp_dict[k] = v
                load_key.append(k)
            else:
                no_load_key.append(k)
        model_dict.update(temp_dict)
        model.load_state_dict(model_dict)
        # ------------------------------------------------------#
        #   show the keys, which is not matched
        # ------------------------------------------------------#
        if local_rank == 0:
            print("\nSuccessful Load Key:", str(load_key)[
                                            :500], "……\nSuccessful Load Key Num:", len(load_key))
            print("\nFail To Load Key:", str(no_load_key)[
                                         :500], "……\nFail To Load Key num:", len(no_load_key))
            print(
                "\n\033[1;33;44m温馨提示，head部分没有载入是正常现象，Backbone部分没有载入是错误的。\033[0m")
