import cv2
import numpy as np

def solve_pnp_distance(rect_obstacle, point3d, K, distortion):
    top, left, bottom, right = rect_obstacle[0], rect_obstacle[1], rect_obstacle[2], rect_obstacle[3]
    points_2d = np.empty([4, 2], dtype=np.float32)
    points_2d[0] = [left, top]
    points_2d[1] = [right, top]
    points_2d[2] = [left, bottom]
    points_2d[3] = [right, bottom]
    retval, rvec, tvec = cv2.solvePnP(point3d, points_2d, K, distortion)
    distance = np.sqrt(np.sum(np.power(tvec, 2)))
    return distance

if __name__ == '__main__':
    top, left, bottom, right = 137, 344, 364, 443

    obstacle_points_3d = np.loadtxt("/home/hll/code_space/bike_ws/src/bike_xuan_vision/bike_vision/params/obstacle_rect.txt")
    astra_camera_k = np.loadtxt("/home/hll/code_space/bike_ws/src/bike_xuan_vision/bike_vision/params/camera_k.txt")
    astra_camera_d = np.loadtxt("/home/hll/code_space/bike_ws/src/bike_xuan_vision/bike_vision/params/dist_matrix.txt")

    rect_obstacle = np.array([top, left, bottom, right], dtype=np.float32)
    distance = solve_pnp_distance(rect_obstacle, obstacle_points_3d, astra_camera_k, astra_camera_d)
    print(distance) # True is 1.4
    
