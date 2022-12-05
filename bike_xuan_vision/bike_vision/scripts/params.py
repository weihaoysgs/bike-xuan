import numpy as np


class BikeVisionParams(object):
    show_detect_result_img = False
    show_astra_depth_img = False
    use_torch_pth_model = False
    open_vino_model_path = "/home/hll/code_space/bike_ws/src/bike_xuan_vision/bike_vision/scripts/vino_export/yolox_nano.xml"
    net_input_img_size = (416, 416)
    num_classes = 2
    confidence_threshold = 0.5
    nms_threshold = 0.3
    image_std = np.array([0.229, 0.224, 0.225])
    image_mean = np.array([0.485, 0.456, 0.406])
    class_names = ["obstacle", "obstacle"]
    obstacle_points_3d = np.loadtxt("/home/hll/code_space/bike_ws/src/bike_xuan_vision/bike_vision/params/obstacle_rect.txt")
    astra_camera_k = np.loadtxt("/home/hll/code_space/bike_ws/src/bike_xuan_vision/bike_vision/params/camera_k.txt")
    astra_camera_d = np.loadtxt("/home/hll/code_space/bike_ws/src/bike_xuan_vision/bike_vision/params/dist_matrix.txt")


if __name__ == "__main__":
    print(BikeVisionParams.astra_camera_k)
    print(BikeVisionParams.astra_camera_d)
    print(BikeVisionParams.obstacle_points_3d)